#include "core/Emitter.h"
#include "core/Registry.h"
#include "utils/Misc.h"
#include "core/Scene.h"


class PointLight final : public Emitter {
public:
    const Vec3f intensity;
    const Vec3f position;

    PointLight(const Vec3f &intensity, const Vec3f &position) : intensity(intensity), position(position) {}

    
    virtual Vec3f eval(const Intersection &isc) const override {
        return intensity / glm::dot(position - isc.position, position - isc.position);
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

    Vec3f sampleLe(const Vec2f &sample1, const Vec3f &sample2, 
                   Vec3f &posn, Vec3f &normal, Vec3f &dirn, Float &pdf) const override {
        // TODO        
        throw std::runtime_error("point light doesn't support sampleLe yet.");
        
        posn = position;
        dirn = uniformSphereSample(sample1);
        normal = dirn;
        pdf = Inv4Pi;
        // TODO: this only returns intensity. Must be divided by distance squared to be converted to radiance
        return intensity;
    }
    
    std::string to_string() const override {
        std::ostringstream oss;
        oss << "Emitter(PointLight): [ intensity=" << intensity << ", position=" << position << " ]";
        return oss.str();
    }
};

// --------------------------- Registry functions ---------------------------
Emitter *createPointLight(const std::unordered_map<std::string, std::string> &properties, const std::unordered_map<std::string, const Texture*>& textures) {
    Vec3f intensity{1.0, 1.0, 1.0};
    Vec3f position{0.0, 0.0, 0.0};

    for (const auto &[key, value] : properties) {
        if (key == "intensity") {
            intensity = strToVec3f(value);
        } else if (key == "position") {
            position = strToVec3f(value);
        } else if (key == "to_world") {
            position = Vec3f{strToMat4f(value) * Vec4f{position, 1.0}};
        } else {
            throw std::runtime_error("Unknown property '" + key + "' for Point Light emitter");
        }
    }
    
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
