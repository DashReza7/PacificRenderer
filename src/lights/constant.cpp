#include "core/Emitter.h"
#include "core/Registry.h"
#include "utils/Misc.h"
#include "core/Scene.h"
#include "core/Bitmap.h"


class ConstantLight final : public Emitter {
public:
    Vec3f radiance;

    ConstantLight(const Vec3f &radiance) : radiance(radiance) {}

    
    virtual Vec3f eval(const Intersection &isc) const override {
        return radiance;
    }

    EmitterSample sampleLi(const Scene *scene, const Intersection &isc, const Vec3f &sample) const override {
        Vec3f w = uniformSphereSample(Vec2f{sample.y, sample.z});
        // check for occlusion
        Ray shadow_ray{isc.position + sign(glm::dot(isc.normal, w)) * isc.normal * Epsilon, w, Epsilon, 1e4, true};
        Intersection tmp_isc{};
        bool is_valid = !scene->ray_intersect(shadow_ray, tmp_isc);
        tmp_isc.dirn = w;

        return EmitterSample{Inv4Pi, -w, is_valid, eval(tmp_isc), EmitterFlags::NONE};
    }

    Vec3f sampleLe(const Vec2f &sample1, const Vec3f &sample2, 
                   Vec3f &posn, Vec3f &normal, Vec3f &dirn, Float &pdf) const override {
        throw std::runtime_error("Constant light does not support sampleLe() yet.");
    }
    
    std::string to_string() const override {
        std::ostringstream oss;
        oss << "Emitter(ConstantLight): []";
        return oss.str();
    }
};

// --------------------------- Registry functions ---------------------------
Emitter *createConstantLight(const std::unordered_map<std::string, std::string> &properties, const std::unordered_map<std::string, const Texture*>& textures) {
    Vec3f radiance = Vec3f{1.0f};  // Default to white light

    for (const auto &[key, value] : properties) {
        if (key == "radiance") {
            radiance = strToVec3f(value);
        } else {
            throw std::runtime_error("Unknown property '" + key + "' for Envmap Light emitter");
        }
    }

    return new ConstantLight{radiance};
}

namespace {
struct ConstantLightRegistrar {
    ConstantLightRegistrar() {
        EmitterRegistry::registerEmitter("constant", createConstantLight);
    }
};

static ConstantLightRegistrar registrar;
}  // namespace
