#include "core/Integrator.h"
#include "core/Scene.h"
#include "core/MathUtils.h"
#include "core/Registry.h"
#include <sstream>

class AlbedoIntegrator : public SamplingIntegrator {
public:
    AlbedoIntegrator() = default;

    Vec3f sample_radiance(const Scene *scene, Sampler *sampler, const Ray &ray, int row, int col) const override {
        Intersection isc;
        bool is_hit = scene->ray_intersect(ray, isc);
        if (is_hit) {
            // reflectance in diffuse BSDFs
            throw std::runtime_error("Albedo integrator not implemented yet.");
            // return Pi * isc.shape->bsdf->eval(Vec3f{0.0, 0.0, 1.0}, Vec3f{0.0, 0.0, 1.0});
        }
        return Vec3f{0.0};
    }

    std::string to_string() const override {
        return "Integrator(Albedo): [ ]";
    }
};


// ------------------- Registry functions -------------------
Integrator* createAlbedoIntegrator(const std::unordered_map<std::string, std::string>& properties) {
    if (properties.size() > 0) {
        throw std::runtime_error("Albedo integrator does not take any properties");
    }
    
    return new AlbedoIntegrator();
}

namespace {
    struct AlbedoIntegratorRegistrar {
        AlbedoIntegratorRegistrar() {
            IntegratorRegistry::registerIntegrator("albedo", createAlbedoIntegrator);
        }
    };
    
    static AlbedoIntegratorRegistrar registrar;
}

