#include "core/Emitter.h"
#include "core/Registry.h"
#include "utils/Misc.h"
#include "core/Shape.h"
#include "core/Scene.h"
#include "core/Texture.h"

// Right now only uniform area emitters are supported.
// (texture emitters are not supported)
class AreaLight final : public Emitter {
private:
    const Texture *radiance;

public:
    const Shape *shape;  // the shape that this area light is attached to

    AreaLight(const Texture *radiance) : radiance(radiance) {}


    void set_shape(const Shape *shape) override {
        this->shape = shape;
    }

    // FIXME: validate the physical correctness
    virtual Vec3f eval(const Intersection &isc) const override {
        return radiance->eval(isc);
    }

    EmitterSample sampleLi(const Scene *scene, const Intersection &isc, const Vec3f &sample) const override {
        auto [position, normal, pdf] = shape->sample_point_on_surface(sample.x, Vec2f{sample.y, sample.z});
        Vec3f dirn = position - isc.position;
        Float distance = glm::length(dirn);
        dirn = glm::normalize(dirn);
        
        Vec3f radiance_val{0.5};
        bool is_valid = glm::dot(normal, dirn) < 0.0;
        if (is_valid) {
            // check for occlusion
            Ray shadow_ray{isc.position + sign(glm::dot(isc.normal, dirn)) * isc.normal * Epsilon, dirn, Epsilon, distance - 2 * Epsilon};
            Intersection light_isc;
            bool is_hit = scene->ray_intersect(shadow_ray, light_isc);
            if (is_hit) {
                // might be a false positive
                // TODO: make this more robust
                if (light_isc.shape != shape || glm::length(light_isc.position - position) >= 1e-2)
                    is_valid = false;
            }
            if (is_valid)
                radiance_val = radiance->eval(light_isc);
        }
        pdf *= Sqr(distance) / std::abs(glm::dot(normal, dirn));
        return EmitterSample{pdf, -dirn, is_valid, radiance_val, EmitterFlags::AREA};
    }

    // Sample a posn on surface uniformly (area measure),
    // and a uniform dirn from hemisphere (solid angle measure)
    Vec3f sampleLe(const Vec2f &sample1, const Vec3f &sample2, 
                   Vec3f &posn, Vec3f &normal, Vec3f &dirn, Float &pdf) const override {
        auto [position, nml, pdf_shape] = shape->sample_point_on_surface(sample2.x, Vec2f{sample2.y, sample2.z});
        posn = position;
        normal = nml;
        dirn = localToWorld(uniformHemisphereSample(sample1), normal);
        pdf = pdf_shape * Inv2Pi;

        // TODO: this only works for constant Area light
        Intersection isc_tmp;
        // multiply by cosine term, used for area measure in particle tracing
        return eval(isc_tmp) * std::abs(glm::dot(normal, dirn));
    }
    
    std::string to_string() const override {
        std::ostringstream oss;
        oss << "Emitter(AreaLight): [ radiance=" << radiance << " ]";
        return oss.str();
    }
};

// --------------------------- Registry functions ---------------------------
Emitter *createAreaLight(const std::unordered_map<std::string, std::string> &properties, const std::unordered_map<std::string, const Texture*>& textures) {
    const Texture *radiance = TextureRegistry::createTexture("constant", {});

    for (const auto &[key, value] : properties) {
        if (key == "radiance") {
            delete radiance;
            radiance = TextureRegistry::createTexture("constant", {{"albedo", value}});
        } else {
            throw std::runtime_error("Unknown property '" + key + "' for Area Light emitter");
        }
    }

    for (const auto &[key, tex_ptr] : textures) {
        if (key == "radiance") {
            delete radiance;
            radiance = tex_ptr;
        } else {
            throw std::runtime_error("Unknown texture slot '" + key + "' for Area Light emitter");
        }
    }
    
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
