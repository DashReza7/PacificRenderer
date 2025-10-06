#include "core/Emitter.h"
#include "core/Registry.h"
#include "utils/Misc.h"
#include "core/Scene.h"


class PointLight final : public Emitter {
public:
    const Vec3f intensity;
    const Vec3f position;

    PointLight(const Vec3f &intensity, const Vec3f &position) : intensity(intensity), position(position) {}

    
    virtual Vec3f eval(const Vec3f &shading_posn) const override {
        return intensity / glm::dot(position - shading_posn, position - shading_posn);
    }

    EmitterSample sampleLi(const Scene *scene, const Intersection &isc, const Vec3f &sample) const override {
        Vec3f dirn = position - isc.position;
        Float distance = glm::length(dirn);
        dirn = glm::normalize(dirn);
        bool is_valid = true;
        if (is_valid) {
            // check for occlusion
            Ray shadow_ray{isc.position + sign(glm::dot(isc.normal, dirn)) * isc.normal * Epsilon, dirn, Epsilon, distance - 2 * Epsilon, true};
            Intersection tmp_isc{};
            is_valid = !scene->ray_intersect(shadow_ray, tmp_isc);
        }

        return EmitterSample{1.0, -dirn, is_valid, intensity / Sqr(distance), EmitterFlags::DELTA_POSITION};
    }

    std::string to_string() const override {
        std::ostringstream oss;
        oss << "Emitter(PointLight): [ intensity=" << intensity << ", position=" << position << " ]";
        return oss.str();
    }
};

// --------------------------- Registry functions ---------------------------
Emitter *createPointLight(const std::unordered_map<std::string, std::string> &properties) {
    Vec3f intensity{1.0, 1.0, 1.0};
    Vec3f position{0.0, 0.0, 0.0};

    auto it = properties.find("intensity");
    if (it != properties.end())
        intensity = strToVec3f(it->second);

    it = properties.find("position");
    if (it != properties.end())
        position = strToVec3f(it->second);

    it = properties.find("to_world");
    if (it != properties.end())
        position = Vec3f{strToMat4f(it->second) * Vec4f{position, 1.0}};

    return new PointLight{intensity, position};
}

namespace {
struct PointLightRegistrar {
    PointLightRegistrar() {
        EmitterRegistry::registerEmitter("point", createPointLight);
    }
};

static PointLightRegistrar registrar;
}  // namespace
