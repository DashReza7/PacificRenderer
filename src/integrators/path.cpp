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

    Vec3f sample_radiance(const Scene *scene, Sampler *sampler, const Ray &ray, int row, int col) const override;

    std::string to_string() const override {
        std::ostringstream oss;
        oss << "Integrator(PathTracer): [ max_depth=" << max_depth << ", rr_depth=" << rr_depth << ", hide_emitters=" << (hide_emitters ? "true" : "false") << " ]";
        return oss.str();
    }
};

// ------------------ PathTracer function definitions ----------------------------

Vec3f PathTracerIntegrator::sample_radiance(const Scene *scene, Sampler *sampler, const Ray &ray, int row, int col) const {
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
