#include "core/Integrator.h"
#include "core/Thread.h"

class PrimarySample {
private:
    int cur_coord = 0;  // used for accessing the next sample requested by integrator
    int mut_cnt = 0;  // number of accepted mutations since the latest large step event
    Sampler *sampler;
    std::vector<Float> tent_samples{};
    std::vector<int> samples_mut_cnt{};  // number of accepted mutations of each sample since the last large step event

    // small mutation given a current sample
    Float mutate(Float cur_sample) {
        Float rnd = sampler->get_1D();
        Float s1 = 1./1024, s2 = 1./64;
        Float dv = s2 * exp(-log(s2/s1) * rnd);
        if (rnd < 0.5) {
            cur_sample += dv;
            if (cur_sample > 1)
                cur_sample -= 1;
        } else {
            cur_sample -= dv;
            if (cur_sample < 0)
                cur_sample += 1;
        }
        return cur_sample;
    }

public:
    std::vector<Float> samples{};

    PrimarySample(Sampler *sampler) : sampler(sampler) {}

    // get the next sample. Handle the lazy perterbations
    Float getSample(bool large_step) {
        tent_samples.push_back(0);
        if (large_step) {
            tent_samples[cur_coord] = sampler->get_1D();
        } else {  // small step
            Float old_sample = cur_coord >= samples.size() ? sampler->get_1D() : samples.at(cur_coord);
            int compensate_mut_cnt = mut_cnt - (cur_coord >= samples.size() ? 0 : samples_mut_cnt.at(cur_coord));
            for (int i = 0; i < compensate_mut_cnt; i++)
                old_sample = mutate(old_sample);
            // apply the small perturbation
            tent_samples[cur_coord] = mutate(old_sample);
        }

        cur_coord++;
        return tent_samples.at(cur_coord - 1);
    }

    void commit(bool accept, bool large_step) {
        if (accept) {
            if (large_step) {
                mut_cnt = 0;
                for (auto &i : samples_mut_cnt)
                    i = 0;
                for (int i = samples_mut_cnt.size(); i < tent_samples.size(); i++)
                    samples_mut_cnt.push_back(0);
                for (int i = tent_samples.size(); i < samples.size(); i++)  // the rest is handled in the last for loop
                    samples[i] = sampler->get_1D();
            } else {  // small step
                mut_cnt++;
                for (int i = 0; i < tent_samples.size(); i++)
                    if (i < samples_mut_cnt.size())
                        samples_mut_cnt[i] = mut_cnt;
                    else
                        samples_mut_cnt.push_back(mut_cnt);
            }
            for (int i = 0; i < tent_samples.size(); i++)
                if (i < samples.size())
                    samples[i] = tent_samples.at(i);
                else
                    samples.push_back(tent_samples.at(i));
        } else {  // mut not accepted

        }
        tent_samples.clear();
        cur_coord = 0;
    }
};

class PSSMLTIntegrator : public Integrator {
private:
    int b_samples;  // number of path samples to estimate the normalization factor
    int n_seeds;
    int chain_steps;
    int max_depth;
    int rr_depth;
    bool hide_emitters;
    Float p_large;

    Float estimate_b(const Scene *scene, Sampler *sampler, size_t n_threads, Float &estimate_time, bool show_progress) const;
    Vec3f sample_radiance(const Scene *scene, Sampler *sampler, const Ray &ray, int row, int col) const;
    std::vector<std::pair<PrimarySample, Vec3f>> init_seeds(const Scene *scene, Sensor *sensor) const;
    Vec3f sample_path(const Scene *scene, PrimarySample &ps, const Ray &ray, bool large_step=true) const;

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
    PSSMLTIntegrator(int b_samples, int n_seeds, int chain_steps, int max_depth, int rr_depth, bool hide_emitters, Float p_large) : b_samples(b_samples), n_seeds(n_seeds), chain_steps(chain_steps), max_depth(max_depth), rr_depth(rr_depth), hide_emitters(hide_emitters), p_large(p_large) {}

    void render(const Scene *scene, Sensor *sensor, uint32_t n_threads, bool show_progress) override;
    std::string to_string() const override {
        throw std::runtime_error("PSSMLT to_string not implemented yet!");
    }
};

// ------------------ PSSMLT function definitions ----------------------------
void PSSMLTIntegrator::render(const Scene *scene, Sensor *sensor, uint32_t n_threads, bool show_progress) {
    Float b_estimate_time;
    Float b = estimate_b(scene, &sensor->sampler, n_threads, b_estimate_time, show_progress);
    Float inv_b = 1.0 / b;

    std::vector<std::pair<PrimarySample, Vec3f>> seeds = init_seeds(scene, sensor);
    
    // run the Markov chain
    for (int step = 0; step < chain_steps; step++)
        for (int i = 0; i < seeds.size(); i++) {
            bool large_step = sensor->sampler.get_1D() < p_large;
            Vec2f pfilm{seeds[i].first.getSample(large_step), seeds[i].first.getSample(large_step)};
            Ray sensor_ray{sensor->origin_world, sensor->iplaneToWorld(pfilm.x, pfilm.y), 1e-3, 1e6};
            Vec3f tent_radiance = sample_path(scene, seeds[i].first, sensor_ray, large_step);
            Float I_new = average(tent_radiance);
            Float I_old = average(seeds[i].second);
            // TODO: handle the case where one or both of them are zero
            Float a;
            if (I_old == 0 && I_new != 0)
                a = 1;
            else if (I_old == 0 && I_new == 0)
                a = 0.5;
            else
                a = std::min(1.0f, I_new/I_old);
            
            Vec3f old_contrib = (1.0f - a) * seeds[i].second 
                    / ((I_old * inv_b + p_large) * chain_steps * seeds.size());
            Vec3f new_contrib = (a + (large_step?1:0)) * tent_radiance
                    / ((I_new * inv_b + p_large) * chain_steps * seeds.size());
            Float scale = 100000;
            sensor->film.commit_splat(scale * old_contrib, Vec2f{seeds[i].first.samples.at(0), seeds[i].first.samples.at(1)});
            sensor->film.commit_splat(scale * new_contrib, pfilm);

            bool accept = sensor->sampler.get_1D() < a;
            seeds[i].first.commit(accept, large_step);
            if (accept)
                seeds[i].second = tent_radiance;
        }

    sensor->film.normalize_pixels(1.0);
}

Float PSSMLTIntegrator::estimate_b(const Scene *scene, Sampler *sampler, size_t n_threads, Float &estimate_time, bool show_progress) const {
    std::atomic<Float> b{0};

    ThreadPool tpool{*sampler, n_threads};
    std::vector<std::future<void>> results;

    auto start_time = std::chrono::high_resolution_clock::now();

    for (int tidx = 0; tidx < n_threads; tidx++) {
        int samples_per_thread = b_samples / n_threads;
        if (tidx == n_threads - 1)
            samples_per_thread += b_samples % n_threads;
        results.emplace_back(tpool.enqueue([this, samples_per_thread, scene, &b](Sampler &sampler) {
            Float b_thread = 0;
            for (int smpl = 0; smpl < samples_per_thread; smpl++) {
                Vec2f p_film = sampler.get_2D();
                Vec3f sensor_dirn = scene->sensor->iplaneToWorld(p_film.x, p_film.y);
                Ray sensor_ray{scene->sensor->origin_world, sensor_dirn, 1e-3, 1e6};
                Vec3f incoming_radiance = sample_radiance(scene, &sampler, sensor_ray, -1, -1);
                b_thread += average(incoming_radiance) / b_samples;
            }
            b.fetch_add(b_thread);
        }));
    }

    for (auto &result : results)
        result.get();

    auto end_time = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> elapsed = end_time - start_time;
    std::cout << "b estimation took " << std::format("{:.02f}", elapsed.count()) << " seconds." << std::endl;

    return b;
}

Vec3f PSSMLTIntegrator::sample_radiance(const Scene *scene, Sampler *sampler, const Ray &ray, int row, int col) const {
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

// return a list of primarySamples with their corresponding contribs
std::vector<std::pair<PrimarySample, Vec3f>> PSSMLTIntegrator::init_seeds(const Scene *scene, Sensor *sensor) const {
    // list of (ps, scalar_contrib). used for sampling them according to their scalar_contrib
    int n_candids = int(n_seeds * 100);
    std::vector<std::pair<PrimarySample, Vec3f>> candids{};
    for (int i = 0; i < n_candids; i++) {
        // sample a (several) lightpath based on a sequence of random numbers and return their max contribution
        PrimarySample ps{&sensor->sampler};
        Vec2f pfilm{ps.getSample(true), ps.getSample(true)};
        Ray sensor_ray{sensor->origin_world, sensor->iplaneToWorld(pfilm.x, pfilm.y), 1e-3, 1e6};
        Vec3f radiance = sample_path(scene, ps, sensor_ray);
        ps.commit(true, true);
        candids.push_back({ps, radiance});
    }

    // TODO: sample n_seeds seeds from candids
    std::vector<std::pair<PrimarySample, Vec3f>> selected_candids;
    // TODO:
    selected_candids.push_back(candids.at(0));
    return selected_candids;
    for (int i = 0; i < n_seeds; i++) {
        throw std::runtime_error("selectign candids not implemented yet.");
    }
    
    return selected_candids;
}

Vec3f PSSMLTIntegrator::sample_path(const Scene *scene, PrimarySample &ps, const Ray &ray, bool large_step) const {
    Vec3f throughput{1.0};
    Vec3f radiance{0.0};

    Ray curr_ray = ray;
    Intersection curr_isc;
    bool is_hit = scene->ray_intersect(curr_ray, curr_isc);

    // ----------------------- Visible emitters -----------------------
    if (is_hit && hide_emitters && curr_isc.shape->emitter)
        return Vec3f{0};

    for (int depth = 1; depth < max_depth || max_depth == -1; depth++) {
        if (!is_hit ||
            (glm::dot(curr_isc.dirn, curr_isc.normal) <= 0.0 && !curr_isc.shape->bsdf->has_flag(BSDFFlags::TwoSided) && !curr_isc.shape->bsdf->has_flag(BSDFFlags::PassThrough)))
            break;

        // ----------------------- Emitter sampling -----------------------

        if (!curr_isc.shape->bsdf->has_flag(BSDFFlags::Delta)) {
            EmitterSample emitter_sample = scene->sample_emitter(curr_isc, ps.getSample(large_step), Vec3f{ps.getSample(large_step), ps.getSample(large_step), ps.getSample(large_step)});
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

                // Float mis_weight = get_mis_weight_nee(curr_isc, emitter_sample, 1);
                // radiance += mis_weight * throughput * emitter_sample.radiance * bsdf_value / emitter_sample.pdf;
                radiance += throughput * emitter_sample.radiance * bsdf_value / emitter_sample.pdf;
                if (ps.getSample(large_step) < 0.3)
                    return radiance;
            }
        }

        // ------------------------ BSDF sampling -------------------------

        auto [bsdf_sample, bsdf_value] = curr_isc.shape->bsdf->sample(curr_isc, ps.getSample(large_step), Vec2f{ps.getSample(large_step), ps.getSample(large_step)});
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

        // Float mis_weight = get_mis_weight_bsdf(scene, curr_isc, bsdf_sample, 1);
        // throughput *= mis_weight * bsdf_value / bsdf_sample.pdf;
        throughput *= bsdf_value / bsdf_sample.pdf;

        if (std::isnan(throughput.x) || std::isnan(throughput.y) || std::isnan(throughput.z))
            throw std::runtime_error("Throughput is NaN in PathTracerIntegrator");
        if (std::isinf(throughput.x) || std::isinf(throughput.y) || std::isinf(throughput.z))
            throw std::runtime_error("Throughput is Inf in PathTracerIntegrator");

        curr_ray = Ray{curr_isc.position + sign(glm::dot(localToWorld(bsdf_sample.wo, curr_isc.normal), curr_isc.normal)) * curr_isc.normal * Epsilon, localToWorld(bsdf_sample.wo, curr_isc.normal), Epsilon, 1e4};
        is_hit = scene->ray_intersect(curr_ray, curr_isc);
    }
    return Vec3f{0};
}

// ------------------- Registry functions -------------------
Integrator *createPSSMLTIntegrator(const std::unordered_map<std::string, std::string> &properties) {
    int b_samples = 10000;
    int n_seeds = 100;
    int chain_steps = 1024;
    int max_depth = 20;
    int rr_depth = 5;
    bool hide_emitters = false;
    Float p_large = 0.3;

    for (const auto &[key, value] : properties) {
        if (key == "b_samples") {
            b_samples = std::stoi(value);
        } else if (key == "n_seeds") {
            n_seeds = std::stoi(value);
        } else if (key == "chain_steps") {
            chain_steps = std::stoi(value);
        } else if (key == "max_depth") {
            max_depth = std::stoi(value);
        } else if (key == "rr_depth") {
            rr_depth = std::stoi(value);
        } else if (key == "hide_emitters") {
            hide_emitters = (value == "true" || value == "1");
        } else if (key == "p_large") {
            p_large = std::stod(value);
        } else {
            throw std::runtime_error("Unknown property '" + key + "' for Path Tracer integrator");
        }
    }

    return new PSSMLTIntegrator(b_samples, n_seeds, chain_steps, max_depth, rr_depth, hide_emitters, p_large);
}
namespace {
struct PSSMLTIntegratorRegistrar {
    PSSMLTIntegratorRegistrar() {
        IntegratorRegistry::registerIntegrator("pssmlt", createPSSMLTIntegrator);
    }
};

static PSSMLTIntegratorRegistrar registrar;
}  // namespace
