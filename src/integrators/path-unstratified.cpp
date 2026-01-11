#include "core/Integrator.h"
#include "core/Thread.h"

class UnstratPathTracerIntegrator : public Integrator {
private:
    int max_depth;
    int rr_depth;
    bool hide_emitters;

    Float get_mis_weight_nee(const Intersection &isc, const EmitterSample &emitter_sample, uint32_t n_bsdf_samples) const {
        if ((emitter_sample.emitter_flags & EmitterFlags::DELTA_DIRECTION) != EmitterFlags::NONE || n_bsdf_samples == 0)
            return 1.0;
        if (!isc.shape->bsdf->has_flag(BSDFFlags::Delta)) {
            Vec3f wo_local = worldToLocal(-emitter_sample.direction, isc.normal);
            Float bsdf_sampling_pdf = isc.shape->bsdf->pdf(isc, wo_local);
            if (bsdf_sampling_pdf < 0.0)
                throw std::runtime_error("Negative BSDF pdf in MIS weight computation");

            return Sqr(emitter_sample.pdf) / (Sqr(emitter_sample.pdf) + Sqr(bsdf_sampling_pdf));
        }

        return 0.0;
    }
    Float get_mis_weight_bsdf(const Scene *scene, const Intersection &isc, const BSDFSample &bsdf_sample, uint32_t n_emitter_samples) const {
        if ((bsdf_sample.flags & BSDFSampleFlags::Delta) != BSDFSampleFlags::None || n_emitter_samples == 0)
            return 1.0;
        Float nee_pdf = scene->pdf_nee(isc, localToWorld(bsdf_sample.wo, isc.normal));
        if (nee_pdf < 0.0)
            throw std::runtime_error("Negative NEE pdf in MIS weight computation");
        if (nee_pdf <= Epsilon)
            return 1.0;

        return Sqr(bsdf_sample.pdf) / (Sqr(bsdf_sample.pdf) + Sqr(nee_pdf));
    }

public:
    UnstratPathTracerIntegrator(int max_depth, int rr_depth, bool hide_emitters) : max_depth(max_depth), rr_depth(rr_depth), hide_emitters(hide_emitters) {}

    void render(const Scene *scene, Sensor *sensor, uint32_t n_threads, bool show_progress) override;

    Vec3f sample_radiance(const Scene *scene, Sampler *sampler, const Ray &ray, int row, int col) const;

    std::string to_string() const override {
        throw std::runtime_error("unstrat path tracer to_string not implemented yet!");
    }
};

// ------------------- Registry functions -------------------
Integrator *createUnstratPathTracerIntegrator(const std::unordered_map<std::string, std::string> &properties) {
    int max_depth = -1;
    int rr_depth = 5;
    bool hide_emitters = false;

    for (const auto &[key, value] : properties) {
        if (key == "max_depth") {
            max_depth = std::stoi(value);
        } else if (key == "rr_depth") {
            rr_depth = std::stoi(value);
        } else if (key == "hide_emitters") {
            hide_emitters = (value == "true" || value == "1");
        } else {
            throw std::runtime_error("Unknown property '" + key + "' for Path Tracer integrator");
        }
    }

    return new UnstratPathTracerIntegrator(max_depth, rr_depth, hide_emitters);
}

namespace {
struct UnstratPathTracerIntegratorRegistrar {
    UnstratPathTracerIntegratorRegistrar() {
        IntegratorRegistry::registerIntegrator("path-unstrat", createUnstratPathTracerIntegrator);
    }
};

static UnstratPathTracerIntegratorRegistrar registrar;
}  // namespace

// ------------------ UnstratPathTracer function definitions ----------------------------
void UnstratPathTracerIntegrator::render(const Scene *scene, Sensor *sensor, uint32_t n_threads, bool show_progress) {
    uint32_t width = sensor->film.width;
    uint32_t height = sensor->film.height;
    uint32_t total_pixels = width * height;

    // Use sensor->sampler as the master RNG, then create n_threads Samplers with different seeds
    ThreadPool tpool{sensor->sampler, n_threads};
    std::vector<std::future<void>> results;
    std::atomic<size_t> n_rendered_samples{0};
    std::mutex print_mutex;

    auto start_time = std::chrono::high_resolution_clock::now();

    int n_all_samples = total_pixels * sensor->sampler.spp;
    for (int tidx = 0; tidx < n_threads; tidx++) {
        int samples_per_thread = n_all_samples / n_threads;
        if (tidx == n_threads - 1)
            samples_per_thread += n_all_samples % n_threads;
        results.emplace_back(
            tpool.enqueue([this, samples_per_thread, scene, &print_mutex, &n_rendered_samples, total_pixels, width, height, n_all_samples, show_progress](Sampler &sampler) {
                for (int smpl = 0; smpl < samples_per_thread; smpl++) {
                    Vec2f p_film = sampler.get_2D();
                    int row = static_cast<int>(p_film.y * height);
                    int col = static_cast<int>(p_film.x * width);
                    Vec3f sensor_dirn = scene->sensor->iplaneToWorld(p_film.x, p_film.y);
                    Vec3f We = scene->sensor->We(sensor_dirn);
                    Float pdf_We, unused;
                    scene->sensor->pdf_We(sensor_dirn, unused, pdf_We);
                    Ray sensor_ray{scene->sensor->origin_world, sensor_dirn, 1e-3, 1e6};
                    Vec3f incoming_radiance = sample_radiance(scene, &sampler, sensor_ray, row, col);
                    scene->sensor->film.commit_sample(incoming_radiance * We / pdf_We / (Float)(scene->sensor->sampler.spp), row, col, p_film.x, p_film.y);

                    if (show_progress && smpl % 400 == 0) {
                        if (smpl > 0)
                            n_rendered_samples.fetch_add(400);
                        {
                            std::lock_guard<std::mutex> lock(print_mutex);
                            std::cout << "\rProgress: " << std::format("{:.02f}", ((n_rendered_samples + 1) / static_cast<double>(n_all_samples)) * 100) << "%" << std::flush;
                        }
                    }
                }
            }));
    }
}

Vec3f UnstratPathTracerIntegrator::sample_radiance(const Scene *scene, Sampler *sampler, const Ray &ray, int row, int col) const {
    if (max_depth == 0)
        return Vec3f{0.0};
    if (max_depth < -1)
        throw std::runtime_error("max_depth must be -1 (infinite) or a non-negative integer");

    Vec3f throughput{1.0};
    Vec3f radiance{0.0};

    Ray curr_ray = ray;
    Intersection curr_isc;
    bool is_hit = scene->ray_intersect(curr_ray, curr_isc);

    // ----------------------- Visible emitters -----------------------
    if (!hide_emitters) {
        // envmap
        if (!is_hit) {
            if (scene->env_map == nullptr)
                return Vec3f{0.0};
            curr_isc.dirn = curr_ray.d;
            // TODO: BUG for hdr envmap maybe?
            Vec3f lightLi = scene->env_map->eval(curr_isc);
            if (std::isnan(lightLi.x) || std::isnan(lightLi.y) || std::isnan(lightLi.z))
                throw std::runtime_error("Env map returned NaN value in PathTracerIntegrator");
            if (std::isinf(lightLi.x) || std::isinf(lightLi.y) || std::isinf(lightLi.z))
                throw std::runtime_error("Env map returned Inf value in PathTracerIntegrator");
            return lightLi;
        }
        // area light
        if (curr_isc.shape->emitter && glm::dot(curr_isc.normal, curr_isc.dirn) > 0.0)
            radiance += throughput * curr_isc.shape->emitter->eval(curr_isc);
    }
    // blackout emitters if hide_emitter
    if (is_hit && hide_emitters && curr_isc.shape->emitter)
        return Vec3f{0};

    for (int depth = 1; depth < max_depth || max_depth == -1; depth++) {
        if (!is_hit ||
            (glm::dot(curr_isc.dirn, curr_isc.normal) <= 0.0 && !curr_isc.shape->bsdf->has_flag(BSDFFlags::TwoSided) && !curr_isc.shape->bsdf->has_flag(BSDFFlags::PassThrough)))
            break;

        // ----------------------- Emitter sampling -----------------------

        if (!curr_isc.shape->bsdf->has_flag(BSDFFlags::Delta)) {
            EmitterSample emitter_sample = scene->sample_emitter(curr_isc, sampler->get_1D(), sampler->get_3D());
            if (emitter_sample.is_visible) {
                Vec3f wo_local = worldToLocal(-emitter_sample.direction, curr_isc.normal);
                Vec3f bsdf_value = curr_isc.shape->bsdf->eval(curr_isc, wo_local);

                // TODO: debug
                if (bsdf_value.x < 0.0 || bsdf_value.y < 0.0 || bsdf_value.z < 0.0)
                    throw std::runtime_error("BSDF eval returned non-positive value in DirectLightingIntegrator");
                if (std::isnan(bsdf_value.x) || std::isnan(bsdf_value.y) || std::isnan(bsdf_value.z))
                    throw std::runtime_error("BSDF eval returned NaN value in DirectLightingIntegrator");
                if (std::isinf(bsdf_value.x) || std::isinf(bsdf_value.y) || std::isinf(bsdf_value.z))
                    throw std::runtime_error("BSDF eval returned Inf value in DirectLightingIntegrator");

                Float mis_weight = get_mis_weight_nee(curr_isc, emitter_sample, 1);
                radiance += mis_weight * throughput * emitter_sample.radiance * bsdf_value / emitter_sample.pdf;
            }
        }

        // ------------------------ BSDF sampling -------------------------

        auto [bsdf_sample, bsdf_value] = curr_isc.shape->bsdf->sample(curr_isc, sampler->get_1D(), sampler->get_2D());
        if (bsdf_sample.pdf < 0.0 || bsdf_value.x < 0.0 || bsdf_value.y < 0.0 || bsdf_value.z < 0.0) {
            std::cerr << curr_isc.shape->bsdf->to_string() << std::endl;
            throw std::runtime_error("BSDF sample returned invalid value in PathTracerIntegrator: " + std::to_string(bsdf_sample.pdf) + ", " + std::to_string(bsdf_value.x) + ", " + std::to_string(bsdf_value.y) + ", " + std::to_string(bsdf_value.z));
        }
        if (std::isnan(bsdf_sample.pdf) || std::isnan(bsdf_value.x) || std::isnan(bsdf_value.y) || std::isnan(bsdf_value.z))
            throw std::runtime_error("BSDF sample returned NaN value in PathTracerIntegrator");
        if (std::isinf(bsdf_sample.pdf) || std::isinf(bsdf_value.x) || std::isinf(bsdf_value.y) || std::isinf(bsdf_value.z))
            throw std::runtime_error("BSDF sample returned Inf value in PathTracerIntegrator");
        if (bsdf_sample.pdf <= Epsilon || glm::length(bsdf_value) <= Epsilon)
            break;

        Float mis_weight = get_mis_weight_bsdf(scene, curr_isc, bsdf_sample, 1);
        throughput *= mis_weight * bsdf_value / bsdf_sample.pdf;

        if (std::isnan(throughput.x) || std::isnan(throughput.y) || std::isnan(throughput.z))
            throw std::runtime_error("Throughput is NaN in PathTracerIntegrator");
        if (std::isinf(throughput.x) || std::isinf(throughput.y) || std::isinf(throughput.z))
            throw std::runtime_error("Throughput is Inf in PathTracerIntegrator");

        curr_ray = Ray{curr_isc.position + sign(glm::dot(localToWorld(bsdf_sample.wo, curr_isc.normal), curr_isc.normal)) * curr_isc.normal * Epsilon, localToWorld(bsdf_sample.wo, curr_isc.normal), Epsilon, 1e4};
        is_hit = scene->ray_intersect(curr_ray, curr_isc);
        Vec3f lightLi{0.0};
        if (is_hit) {
            if (curr_isc.shape->emitter == nullptr || glm::dot(curr_isc.dirn, curr_isc.normal) < 0)
                lightLi = Vec3f{0.0};
            else
                lightLi = curr_isc.shape->emitter->eval(curr_isc);
        } else {
            if (scene->env_map != nullptr) {
                Intersection tmp;
                tmp.dirn = curr_ray.d;
                lightLi = scene->env_map->eval(tmp);
            }
        }
        radiance += throughput * lightLi;

        // do RussianRoulette
        if (depth + 1 >= rr_depth) {
            Float rr_survive_prob = std::min(std::max(throughput.x, std::max(throughput.y, throughput.z)), Float(0.95));
            if (sampler->get_1D() > rr_survive_prob)
                break;
            throughput /= rr_survive_prob;
        }
    }

    return radiance;
}
