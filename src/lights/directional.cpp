#include "core/Emitter.h"
#include "core/Registry.h"
#include "utils/Misc.h"
#include "core/Scene.h"


class DirectionalLight final : public Emitter {
public:
    const Vec3f irradiance;
    // direction of light propagation
    const Vec3f direction;

    DirectionalLight(const Vec3f &irradiance, const Vec3f &direction) : irradiance(irradiance), direction(direction) {}

    
    virtual Vec3f eval(const Intersection &isc) const override {
        return irradiance;
    }

    EmitterSample sampleLi(const Scene *scene, const Intersection &isc, const Vec3f &sample) const override {
        Intersection tmp_isc;
        bool is_hit = scene->ray_intersect(Ray{isc.position + sign(glm::dot(isc.normal, -direction)) * isc.normal * Epsilon, -direction, Epsilon, 1e4, true}, tmp_isc);

        return EmitterSample{1.0, direction, !is_hit, irradiance, EmitterFlags::DELTA_DIRECTION};
    }

    std::string to_string() const override {
        std::ostringstream oss;
        oss << "Emitter(DirectionalLight): [ irradiance=" << irradiance << ", direction=" << direction << " ]";
        return oss.str();
    }
};

// --------------------------- Registry functions ---------------------------
Emitter *createDirectionalLight(const std::unordered_map<std::string, std::string> &properties, const std::unordered_map<std::string, const Texture*>& textures) {
    Vec3f irradiance{1.0, 1.0, 1.0};
    Vec3f direction{0.0, 0.0, 1.0};

    for (const auto &[key, value] : properties) {
        if (key == "irradiance") {
            irradiance = strToVec3f(value);
        } else if (key == "direction") {
            direction = strToVec3f(value);
        } else if (key == "to_world") {
            direction = Vec3f{strToMat4f(value) * Vec4f{direction, 0.0}};
        } else {
            throw std::runtime_error("Unknown property '" + key + "' for Directional Light emitter");
        }
    }
        
    return new DirectionalLight{irradiance, direction};
}

namespace {
struct DirectionalLightRegistrar {
    DirectionalLightRegistrar() {
        EmitterRegistry::registerEmitter("directional", createDirectionalLight);
    }
};

static DirectionalLightRegistrar registrar;
}  // namespace
