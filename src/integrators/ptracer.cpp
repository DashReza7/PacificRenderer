#include <sstream>

#include "core/Integrator.h"
#include "core/MathUtils.h"
#include "core/Registry.h"
#include "core/Scene.h"
#include "core/Thread.h"

class ParticleTracerIntegrator : public Integrator {
private:
    int max_depth;
    // depth to start Russian roulette. refer to mitsuba3 documentation for more details.
    int rr_depth;
    bool hide_emitters;

public:
    ParticleTracerIntegrator(int max_depth, int rr_depth, bool hide_emitters) : max_depth(max_depth), rr_depth(rr_depth), hide_emitters(hide_emitters) {}

    void render(const Scene* scene, Sensor* sensor, uint32_t n_threads, bool show_progress) override {
        extern bool g_DEBUG;
        // FIXME: right now directly visible lights are not captured

        if (max_depth < 0)
            max_depth = 100;

        // Use sensor->sampler as the master RNG, then create n_threads Samplers with different seeds
        ThreadPool tpool{sensor->sampler, n_threads};
        std::vector<std::future<void>> results;
        std::atomic<size_t> n_rendered_particles{0};
        std::mutex print_mutex;

        auto start_time = std::chrono::high_resolution_clock::now();

        Vec3f sensor_origin = sensor->origin_world;
        int n_all_samples = sensor->film.width * sensor->film.height * sensor->sampler.spp;
        for (size_t thread_idx = 0; thread_idx < n_threads; thread_idx++) {
            int samples_per_thread = n_all_samples / n_threads;
            if (thread_idx == n_threads - 1)
                samples_per_thread += n_all_samples % n_threads;
            results.emplace_back(
                tpool.enqueue([this, sensor, scene, show_progress, n_all_samples, sensor_origin, samples_per_thread, &n_rendered_particles, &print_mutex](Sampler& sampler) {
                    for (int smpl = 0; smpl < samples_per_thread; smpl++) {
                        // sample a posn & dirn on light source
                        Vec3f posn, normal, dirn;
                        Float pdf_posn, pdf_dirn;
                        const Shape *light_shape;
                        Vec3f T = scene->sample_emitter_ptrace(sensor->sampler.get_2D(), sensor->sampler.get_3D(), sensor->sampler.get_1D(),
                                                               posn, normal, dirn, light_shape, pdf_posn, pdf_dirn);

                        T *= std::abs(glm::dot(normal, dirn));
                        T /= (pdf_posn * pdf_dirn);

                        // ----------------- Directly connect to camera (if hide_emitters == false) -----------------
                        Vec3f sensor_dirn = glm::normalize(sensor_origin - posn);
                        if (!hide_emitters && glm::dot(normal, sensor_dirn) > 0) {
                            throw std::runtime_error("ptracer directly lights are not implemented. please enable hide_emitter");
                            
                            Float sensor_dist = glm::length(sensor_origin - posn);
                            // check if camera is visible
                            Intersection tmp_isc;
                            bool is_camera_occluded = scene->ray_intersect(Ray{posn + normal * Epsilon, sensor_dirn, 1e-4, sensor_dist, true}, tmp_isc);
                            if (!is_camera_occluded) {
                                // TODO: should I add a factor of 2Pi?
                                Vec3f camT = T / std::abs(glm::dot(normal, dirn)) 
                                                             * std::abs(glm::dot(normal, sensor_dirn));
                                // TODO: fix this. makes black pixels
                                if (std::abs(glm::dot(normal, dirn)) <= 1e-4)
                                    camT = Vec3f{0};
                                camT /= Sqr(sensor_dist);
                                // sensor->add_contrib(-sensor_dirn, camT);
                            }
                        }

                        // shoot a ray
                        Ray ray{posn + dirn * Epsilon, dirn, 1e-4, 1e6};
                        Intersection isc;
                        bool is_hit = scene->ray_intersect(ray, isc);

                        // loop over various lengths of paths
                        for (int i = 0; i < max_depth; i++) {
                            if (!is_hit)
                                break;

                            // --------------------------- Connect to camera ---------------------------
                            Vec3f sensor_dirn = glm::normalize(sensor_origin - isc.position);
                            Float sensor_dist = glm::length(sensor_origin - isc.position);
                            // check if camera is visible
                            Intersection tmp_isc;
                            bool is_camera_occluded = scene->ray_intersect(Ray{isc.position + sign(glm::dot(sensor_dirn, isc.normal)) * isc.normal * Epsilon, sensor_dirn, 1e-4, sensor_dist, true}, tmp_isc);
                            if (!is_camera_occluded) {
                                Vec3f w_sensor;
                                Vec2f p_film;
                                Float pdf_sensor;
                                Vec3f importance = sensor->sample_Wi(isc, w_sensor, pdf_sensor, p_film);
                                sensor->film.commit_splat(T * isc.shape->bsdf->eval(isc, worldToLocal(sensor_dirn, isc.normal))
                                                            * importance
                                                            / pdf_sensor,
                                                          p_film);
                            }

                            // ----------------- prepare posn & dirn for the next iter -----------------
                            auto [bsdf_sample, bsdf_val] = isc.shape->bsdf->sample(isc, sensor->sampler.get_1D(), sensor->sampler.get_2D());
                            T *= bsdf_val / bsdf_sample.pdf;
                            if (bsdf_sample.pdf == 0 || bsdf_val == Vec3f{0})
                                break;

                            dirn = localToWorld(bsdf_sample.wo, isc.normal);
                            ray = Ray{isc.position + dirn * Epsilon, dirn, 1e-4, 1e6};
                            is_hit = scene->ray_intersect(ray, isc);
                        }

                        if (show_progress && smpl % 400 == 0) {
                            if (smpl > 0)
                                n_rendered_particles.fetch_add(100);
                            {
                                std::lock_guard<std::mutex> lock(print_mutex);
                                std::cout << "\rProgress: " << std::format("{:.02f}", ((n_rendered_particles + 1) / static_cast<double>(n_all_samples)) * 400) << "%" << std::flush;
                            }
                        }
                    }
                }));
        }
        for (auto& result : results)
            result.get();
        std::cout << std::endl;

        // add splats to film
        sensor->film.normalize_pixels(1.0 / sensor->sampler.spp);
        
        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end_time - start_time;
        std::cout << "Rendering completed in " << std::format("{:.02f}", elapsed.count()) << " seconds.";
    }

    std::string to_string() const override {
        std::ostringstream oss;
        oss << "Integrator(ParticleTracer): [ max_depth=" << max_depth << ", rr_depth=" << rr_depth << ", hide_emitters=" << (hide_emitters ? "true" : "false") << " ]";
        return oss.str();
    }
};

// ------------------- Registry functions -------------------
Integrator* createParticleTracerIntegrator(const std::unordered_map<std::string, std::string>& properties) {
    int max_depth = -1;
    int rr_depth = 5;
    bool hide_emitters = false;

    for (const auto& [key, value] : properties) {
        if (key == "max_depth") {
            max_depth = std::stoi(value);
        } else if (key == "rr_depth") {
            rr_depth = std::stoi(value);
        } else if (key == "hide_emitters") {
            hide_emitters = (value == "true" || value == "1");
        } else {
            throw std::runtime_error("Unknown property '" + key + "' for Particle Tracer integrator");
        }
    }

    return new ParticleTracerIntegrator(max_depth, rr_depth, hide_emitters);
}

namespace {
struct ParticleTracerIntegratorRegistrar {
    ParticleTracerIntegratorRegistrar() {
        IntegratorRegistry::registerIntegrator("ptracer", createParticleTracerIntegrator);
    }
};

static ParticleTracerIntegratorRegistrar registrar;
}  // namespace
