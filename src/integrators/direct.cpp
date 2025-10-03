#include <sstream>

#include "core/Integrator.h"
#include "core/MathUtils.h"
#include "core/Registry.h"
#include "core/Scene.h"


class DirectLightingIntegrator : public SamplingIntegrator {
private:
    int emitter_samples;
    int bsdf_samples;
    bool hide_emitters;

public:
    DirectLightingIntegrator(int emitter_samples, int bsdf_samples, bool hide_emitters) : emitter_samples(emitter_samples), bsdf_samples(bsdf_samples), hide_emitters(hide_emitters) {}
    ~DirectLightingIntegrator() override = default;

    Vec3f sample_radiance(const Scene *scene, Sampler *sampler, const Ray &ray) const override {
        Intersection isc;
        bool is_hit = scene->ray_intersect(ray, isc);
        // TODO: maybe use and environment map???
        if (!is_hit)
            return Vec3f{0.0};

        Vec3f radiance{0.0};
        // ----------------------- Visible emitters -----------------------
        if (isc.shape->emitter != nullptr) {
            // hit an area light
            // area lights return their radiance, without considering isc
            radiance += isc.shape->emitter->eval(isc);
        }
        // ----------------------- Emitter sampling -----------------------
        Float nee_weight = 1.0 / Float(emitter_samples);
        for (size_t i = 0; i < emitter_samples; i++) {
            EmitterSample emitter_sample = scene->sample_emitter(isc, sampler->get_sample(), sampler->get_sample_3d());
            if (emitter_sample.is_valid) {
                Vec3f wo_world = glm::normalize(emitter_sample.position - isc.position);
                Vec3f wi_world = isc.dirn;
                Vec3f wo_local = worldToLocal(wo_world, isc.normal);
                Vec3f wi_local = worldToLocal(wi_world, isc.normal);
                
                // compute MIS weight
                Float bsdf_sampling_pdf = isc.shape->bsdf->pdf(wi_local, wo_local);
                // Float mis_weight = emitter_sample.pdf / (emitter_sample.pdf + bsdf_sampling_pdf);
                Float mis_weight = Sqr(emitter_sample.pdf) / (Sqr(emitter_sample.pdf) + Sqr(bsdf_sampling_pdf));

                radiance += mis_weight * nee_weight * emitter_sample.radiance * isc.shape->bsdf->eval(wi_local, wo_local) / emitter_sample.pdf;
            }
        }

        // ------------------------ BSDF sampling -------------------------
        Float bsdf_weight = 1.0 / Float(bsdf_samples);
        for (size_t i = 0; i < bsdf_samples; i++) {
            auto [bsdf_sample, bsdf_value] = isc.shape->bsdf->sample(isc, sampler->get_sample(), sampler->get_sample_2d());

            // check if the sample intersects any light
            Intersection tmp_isc{};
            bool is_occluded = scene->ray_intersect(Ray{isc.position + isc.normal * Epsilon, bsdf_sample.wo, Epsilon, 1e4}, tmp_isc);
            if (!is_occluded || tmp_isc.shape->emitter == nullptr || dot(isc.position - tmp_isc.position, tmp_isc.normal) < 0)
                continue;

            // compute MIS weight
            // TODO: check for zero division
            Float nee_pdf = scene->pdf_emitter_sampling(isc, bsdf_sample.wo);
            // Float mis_weight = bsdf_sample.pdf / (bsdf_sample.pdf + nee_pdf);
            Float mis_weight = Sqr(bsdf_sample.pdf) / (Sqr(bsdf_sample.pdf) + Sqr(nee_pdf));
            
            radiance += mis_weight * bsdf_weight * tmp_isc.shape->emitter->eval(isc) * bsdf_value / bsdf_sample.pdf;
        }

        // ----------------------------------------------------------------
        return radiance;
    }

    std::string to_string() const override {
        std::ostringstream oss;
        oss << "Integrator(DirectLighting): [ emitter_samples=" << emitter_samples << ", bsdf_samples=" << bsdf_samples << ", hide_emitters=" << (hide_emitters ? "true" : "false") << " ]";
        return oss.str();
    }
};

// ------------------- Registry functions -------------------
Integrator *createDirectLightingIntegrator(const std::unordered_map<std::string, std::string> &properties) {
    int emitter_samples = 1;
    int bsdf_samples = 1;
    bool hide_emitters = false;

    auto it = properties.find("shading_samples");
    if (it != properties.end()) {
        emitter_samples = std::stoi(it->second);
        bsdf_samples = emitter_samples;
    } else {
        it = properties.find("emitter_samples");
        if (it != properties.end())
            emitter_samples = std::stoi(it->second);

        it = properties.find("bsdf_samples");
        if (it != properties.end())
            bsdf_samples = std::stoi(it->second);
    }

    it = properties.find("hide_emitters");
    if (it != properties.end())
        hide_emitters = (it->second == "true");

    return new DirectLightingIntegrator(emitter_samples, bsdf_samples, hide_emitters);
}

namespace {
struct DirectLightingIntegratorRegistrar {
    DirectLightingIntegratorRegistrar() {
        IntegratorRegistry::registerIntegrator("direct", createDirectLightingIntegrator);
    }
};

static DirectLightingIntegratorRegistrar registrar;
}  // namespace
