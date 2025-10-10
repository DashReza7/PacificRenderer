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

    Vec3f sample_radiance(const Scene *scene, Sampler *sampler, const Ray &ray, int row, int col) const override {
        Vec3f throughput{1.0};
        Vec3f radiance{0.0};

        Ray curr_ray = ray;
        Intersection curr_isc;
        bool is_hit = scene->ray_intersect(curr_ray, curr_isc);
        for (int depth = 0; depth < max_depth || max_depth < 0; depth++) {
            if (!is_hit)  // TODO: implement environment light sampling
                break;

            // ----------------------- Visible emitters -----------------------

            if (depth == 0 && curr_isc.shape->emitter && glm::dot(curr_isc.normal, curr_isc.dirn) > 0.0)
                radiance += throughput * curr_isc.shape->emitter->eval(curr_isc.position);

            // ----------------------- Emitter sampling -----------------------
            
            if (!curr_isc.shape->bsdf->has_flag(BSDFFlags::Delta)) {
                if (curr_isc.shape->bsdf->has_flag(BSDFFlags::TwoSided) || glm::dot(curr_isc.normal, curr_isc.dirn) > 0.0) {
                    EmitterSample emitter_sample = scene->sample_emitter(curr_isc, sampler->get_1D(), sampler->get_3D());
                    if (emitter_sample.is_occluded) {
                        Vec3f wo_local = worldToLocal(-emitter_sample.direction, curr_isc.normal);
                        Vec3f wi_local = worldToLocal(curr_isc.dirn, curr_isc.normal);
                        Vec3f bsdf_value = curr_isc.shape->bsdf->eval(wi_local, wo_local);

                        Float mis_weight = get_mis_weight_nee(curr_isc, emitter_sample);
                        radiance += mis_weight * throughput * emitter_sample.radiance * bsdf_value / emitter_sample.pdf;
                    }
                }
            }

            // ------------------------ BSDF sampling -------------------------

            auto [bsdf_sample, bsdf_value] = curr_isc.shape->bsdf->sample(worldToLocal(curr_isc.dirn, curr_isc.normal), sampler->get_1D(), sampler->get_2D());
            if (bsdf_sample.pdf < 0.0 || bsdf_value.x < 0.0 || bsdf_value.y < 0.0 || bsdf_value.z < 0.0)
                throw std::runtime_error("damn. pdf: " + std::to_string(bsdf_sample.pdf) + ", bsdf_value: " + std::to_string(bsdf_value.x) + ", " + std::to_string(bsdf_value.y) + ", " + std::to_string(bsdf_value.z));
            if (bsdf_sample.pdf <= Epsilon || glm::length(bsdf_value) <= Epsilon)
                break;

            Float mis_weight = get_mis_weight_bsdf(scene, curr_isc, bsdf_sample);
            throughput *= mis_weight * bsdf_value / bsdf_sample.pdf;

            curr_ray = Ray{curr_isc.position + sign(glm::dot(localToWorld(bsdf_sample.wo, curr_isc.normal), curr_isc.normal)) * curr_isc.normal * Epsilon, localToWorld(bsdf_sample.wo, curr_isc.normal), Epsilon, 1e4};
            is_hit = scene->ray_intersect(curr_ray, curr_isc);
            if (is_hit && curr_isc.shape->emitter && glm::dot(curr_isc.normal, curr_ray.d) < 0.0)
                radiance += throughput * curr_isc.shape->emitter->eval(curr_isc.position);

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
