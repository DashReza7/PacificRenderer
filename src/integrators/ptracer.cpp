#include <sstream>

#include "core/Integrator.h"
#include "core/MathUtils.h"
#include "core/Registry.h"
#include "core/Scene.h"

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

        int n_samples = sensor->film.width * sensor->film.height * sensor->sampler.spp;
        for (int smpl = 0; smpl < n_samples; smpl++) {
            // sample a posn & dirn on light source
            Vec3f posn, dirn;
            Float pdf;
            Vec3f T = scene->sample_emitter_ptrace(sensor->sampler.get_2D(), sensor->sampler.get_3D(), sensor->sampler.get_1D(),
                                                   posn, dirn, pdf);

            T /= sensor->sampler.spp;
            Vec3f sensor_origin = sensor->get_origin_world();

            // shoot a ray
            Ray ray{posn + dirn * Epsilon, dirn, 1e-4, 1e6};
            Intersection isc;
            bool is_hit = scene->ray_intersect(ray, isc);
            // convert the dirn part of pdf to area measure
            T /= pdf;

            // loop over various lengths of paths
            for (int i = 0; i < max_depth; i++) {
                if (!is_hit)
                    break;

                // ----------------- compute the contrib (connect to camera) -----------------
                Vec3f sensor_dirn = glm::normalize(sensor_origin - isc.position);
                Float sensor_dist = glm::length(sensor_origin - isc.position);
                // check if camera is visible
                Intersection tmp_isc;
                bool is_camera_occluded = scene->ray_intersect(Ray{isc.position + sensor_dirn * Epsilon, sensor_dirn, 1e-4, sensor_dist, true}, tmp_isc);
                if (!is_camera_occluded) {
                    Vec3f bsdfval_sensor = isc.shape->bsdf->eval(isc, worldToLocal(sensor_dirn, isc.normal));
                    Vec3f camT = T * bsdfval_sensor;
                    camT /= Sqr(sensor_dist);
                    sensor->add_contrib(-sensor_dirn, camT);
                }

                // ----------------- prepare posn & dirn for the next iter -----------------
                auto [bsdf_sample, bsdf_val] = isc.shape->bsdf->sample(isc, sensor->sampler.get_1D(), sensor->sampler.get_2D());
                posn = isc.position;
                dirn = localToWorld(bsdf_sample.wo, isc.normal);
                ray = Ray{isc.position + dirn * Epsilon, dirn, 1e-4, 1e6};
                is_hit = scene->ray_intersect(ray, isc);
                T *= bsdf_val / bsdf_sample.pdf;
            }

            if (show_progress)
                if (smpl % 100 == 0)
                    std::cout << "\rProgress: " << std::format("{:.02f}", ((smpl + 1) / static_cast<double>(n_samples)) * 100) << "%" << std::flush;
        }
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
