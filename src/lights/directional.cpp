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

    
    virtual Vec3f eval(const Vec3f &shading_posn) const override {
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
Emitter *createDirectionalLight(const std::unordered_map<std::string, std::string> &properties) {
    Vec3f irradiance{1.0, 1.0, 1.0};
    Vec3f direction{0.0, 0.0, 1.0};

    auto it = properties.find("irradiance");
    if (it != properties.end())
        irradiance = strToVec3f(it->second);

    it = properties.find("direction");
    if (it != properties.end())
        direction = strToVec3f(it->second);

    it = properties.find("to_world");
    if (it != properties.end())
        direction = Vec3f{strToMat4f(it->second) * Vec4f{direction, 0.0}};

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
