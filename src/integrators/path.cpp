#include <sstream>

#include "core/Integrator.h"
#include "core/MathUtils.h"
#include "core/Registry.h"
#include "core/Scene.h"

class PathTracerIntegrator : public MonteCarloIntegrator {
private:
    bool hide_emitters;

public:
    PathTracerIntegrator(int max_depth, int rr_depth, bool hide_emitters) : MonteCarloIntegrator(max_depth, rr_depth), hide_emitters(hide_emitters) {}

    Vec3f sample_radiance(const Scene *scene, Sampler *sampler, const Ray &ray) const override {
        Vec3f throughput{1.0};
        Vec3f radiance{0.0};

        Ray curr_ray = ray;
        Intersection curr_isc;
        bool is_hit = scene->ray_intersect(curr_ray, curr_isc);
        for (int depth = 0; depth < max_depth || max_depth < 0; depth++) {
            if (!is_hit) {
                // TODO: implement environment light sampling
                return radiance;
            }

            // ----------------------- Visible emitters -----------------------

            // only when directly seen by sensor. It's handled by BSDF sampling for later hits
            if (depth == 0 && curr_isc.shape->emitter) {
                radiance += throughput * curr_isc.shape->emitter->eval(curr_isc.position);
            }

            // ----------------------- Emitter sampling -----------------------

            // only if the BSDF is not of Delta type(e.g. perfectly specular)
            if (!curr_isc.shape->bsdf->has_flag(BSDFFlags::Delta)) {
                EmitterSample emitter_sample = scene->sample_emitter(curr_isc, sampler->get_1D(), sampler->get_3D());
                if (emitter_sample.is_valid) {
                    Vec3f wo_local = worldToLocal(-emitter_sample.direction, curr_isc.normal);
                    Vec3f wi_local = worldToLocal(curr_isc.dirn, curr_isc.normal);
                    Vec3f bsdf_value = curr_isc.shape->bsdf->eval(wi_local, wo_local);

                    Float mis_weight = get_mis_weight_nee(curr_isc, emitter_sample);
                    radiance += mis_weight * throughput * emitter_sample.radiance * bsdf_value / emitter_sample.pdf;
                }
            }

            // ------------------------ BSDF sampling -------------------------

            auto [bsdf_sample, bsdf_value] = curr_isc.shape->bsdf->sample(worldToLocal(curr_isc.dirn, curr_isc.normal), sampler->get_1D(), sampler->get_2D());
            if (bsdf_sample.pdf <= 0 || bsdf_value == Vec3f{0.0})
                break;
            
            Float mis_weight = get_mis_weight_bsdf(scene, curr_isc, bsdf_sample);
            throughput *= mis_weight * bsdf_value / bsdf_sample.pdf;

            curr_ray = Ray{curr_isc.position + bsdf_sample.normal_sign * curr_isc.normal * Epsilon, localToWorld(bsdf_sample.wo, curr_isc.normal), Epsilon, 1e4};
            is_hit = scene->ray_intersect(curr_ray, curr_isc);
            if (is_hit && curr_isc.shape->emitter) {
                radiance += throughput * curr_isc.shape->emitter->eval(curr_isc.position);
            }

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

    std::string to_string() const override {
        std::ostringstream oss;
        oss << "Integrator(PathTracer): [ max_depth=" << max_depth << ", rr_depth=" << rr_depth << ", hide_emitters=" << (hide_emitters ? "true" : "false") << " ]";
        return oss.str();
    }
};

// ------------------- Registry functions -------------------
Integrator *createPathTracerIntegrator(const std::unordered_map<std::string, std::string> &properties) {
    int max_depth = -1;
    int rr_depth = 5;
    bool hide_emitters = false;

    auto it = properties.find("max_depth");
    if (it != properties.end())
        max_depth = std::stoi(it->second);

    it = properties.find("rr_depth");
    if (it != properties.end())
        rr_depth = std::stoi(it->second);

    it = properties.find("hide_emitters");
    if (it != properties.end())
        hide_emitters = (it->second == "true");

    return new PathTracerIntegrator(max_depth, rr_depth, hide_emitters);
}

namespace {
struct PathTracerIntegratorRegistrar {
    PathTracerIntegratorRegistrar() {
        IntegratorRegistry::registerIntegrator("path", createPathTracerIntegrator);
    }
};

static PathTracerIntegratorRegistrar registrar;
}  // namespace
