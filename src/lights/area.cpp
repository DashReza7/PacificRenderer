#include "core/Emitter.h"
#include "core/Registry.h"
#include "utils/Misc.h"
#include "core/Shape.h"
#include "core/Scene.h"

// Right now only uniform area emitters are supported.
// (texture emitters are not supported)
class AreaLight final : public Emitter {
private:
    Vec3f radiance;

public:
    const Shape *shape;  // the shape that this area light is attached to

    AreaLight(const Vec3f &radiance) : radiance(radiance) {}


    void set_shape(const Shape *shape) override {
        this->shape = shape;
    }

    // FIXME: validate the physical correctness
    virtual Vec3f eval(const Vec3f &shading_posn) const override {
        return radiance;
    }

    EmitterSample sampleLi(const Scene *scene, const Intersection &isc, const Vec3f &sample) const override {
        auto [position, normal, pdf] = shape->sample_point_on_surface(sample.x, Vec2f{sample.y, sample.z});
        Vec3f dirn = position - isc.position;
        Float distance = glm::length(dirn);
        dirn = glm::normalize(dirn);
        bool is_valid = glm::dot(normal, dirn) < 0 && glm::dot(isc.normal, dirn) > 0;
        if (is_valid) {
            // check for occlusion
            Ray shadow_ray{isc.position + isc.normal * Epsilon, dirn, Epsilon, distance - 2 * Epsilon, false};
            Intersection tmp_isc;
            bool is_hit = scene->ray_intersect(shadow_ray, tmp_isc);
            if (is_hit) {
                // might be a false positive
                if (tmp_isc.shape != shape || glm::length(tmp_isc.position - position) >= Epsilon)
                    is_valid = false;
            }
        }
        pdf *= Sqr(distance) / glm::dot(normal, -dirn);
        return EmitterSample{pdf, -dirn, is_valid, radiance, EmitterFlags::AREA};
    }

    std::string to_string() const override {
        std::ostringstream oss;
        oss << "Emitter(AreaLight): [ radiance=" << radiance << " ]";
        return oss.str();
    }
};

// --------------------------- Registry functions ---------------------------
Emitter *createAreaLight(const std::unordered_map<std::string, std::string> &properties) {
    Vec3f radiance{0.5, 0.5, 0.5};

    auto it = properties.find("radiance");
    if (it != properties.end())
        radiance = strToVec3f(it->second);

    return new AreaLight{radiance};
}

namespace {
struct AreaLightRegistrar {
    AreaLightRegistrar() {
        EmitterRegistry::registerEmitter("area", createAreaLight);
    }
};

static AreaLightRegistrar registrar;
}  // namespace
