#include "core/Integrator.h"
#include "core/Scene.h"
#include "core/MathUtils.h"
#include "core/Registry.h"
#include <sstream>

class DepthIntegrator : public SamplingIntegrator {
public:
    DepthIntegrator() = default;

    Vec3f sample_radiance(const Scene *scene, Sampler *sampler, const Ray &ray) const override {
        Intersection isc;
        bool is_hit = scene->ray_intersect(ray, isc);
        if (is_hit)
            return Vec3f{isc.distance};
        return Vec3f{0.0};
    }


    std::string to_string() const override {
        return "Integrator(Depth): [ ]";
    }
};


// ------------------- Registry functions -------------------
Integrator* createDepthIntegrator(const std::unordered_map<std::string, std::string>& properties) {
    if (properties.size() > 0) {
        throw std::runtime_error("Depth integrator does not take any properties");
    }
    
    return new DepthIntegrator();
}

namespace {
    struct DepthIntegratorRegistrar {
        DepthIntegratorRegistrar() {
            IntegratorRegistry::registerIntegrator("depth", createDepthIntegrator);
        }
    };
    
    static DepthIntegratorRegistrar registrar;
}

