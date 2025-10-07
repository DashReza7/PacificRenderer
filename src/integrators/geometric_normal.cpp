#include "core/Integrator.h"
#include "core/Scene.h"
#include "core/MathUtils.h"
#include "core/Registry.h"
#include <sstream>


class GeometricNormalIntegrator : public SamplingIntegrator {
public:
    GeometricNormalIntegrator() = default;

    Vec3f sample_radiance(const Scene *scene, Sampler *sampler, const Ray &ray) const override {
        Intersection isc;
        bool is_hit = scene->ray_intersect(ray, isc);
        if (is_hit) {
            // map the normal to [0, 1]^3
            Vec3f radiance = glm::clamp(isc.normal, Float(0.0), Float(1.0));
            // radiance = radiance + Vec3f{1.0};
            // radiance /= 2.0;
            return radiance;
        }
        return Vec3f{0.0};

    }

    std::string to_string() const override {
        return "Integrator(GeometricNormal): [ ]";
    }
};


// ------------------- Registry functions -------------------
Integrator* createGeometricNormalIntegrator(const std::unordered_map<std::string, std::string>& properties) {
    if (properties.size() > 0) {
        throw std::runtime_error("Geometric Normal integrator does not take any properties");
    }
    
    return new GeometricNormalIntegrator();
}

namespace {
    struct GeometricNormalIntegratorRegistrar {
        GeometricNormalIntegratorRegistrar() {
            IntegratorRegistry::registerIntegrator("geometric_normal", createGeometricNormalIntegrator);
        }
    };
    
    static GeometricNormalIntegratorRegistrar registrar;
}
