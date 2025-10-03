#pragma once
#include "core/Geometry.h"
#include "core/MathUtils.h"

// Forward declarations
class Shape;
class Scene;


struct EmitterSample {
    Float pdf;
    Vec3f position;
    bool is_valid;
    Vec3f radiance;

    EmitterSample(Float pdf, const Vec3f &position, bool is_valid, const Vec3f &radiance) : pdf(pdf), position(position), is_valid(is_valid), radiance(radiance) {}
};

// Right now only RGB emitters are supported
class Emitter {
public:
    enum class Type {
        POINT,
        AREA
    };
    Type type;

    Emitter(Emitter::Type type) : type(type) {}

    /// Returns the radiance to the isc point.
    /// Remark: this function does not account for occlusions
    virtual Vec3f eval(const Intersection &isc) const = 0;

    /// Used for NEE. Returned pdf is in solid angle measure(or 1 for Delta light sources)
    virtual EmitterSample sampleLi(const Scene *scene, const Intersection &isc, const Vec3f &sample) const = 0;

    virtual std::string to_string() const = 0;
};

class PointLight final : public Emitter {
public:
    Vec3f intensity;
    Vec3f position;

    PointLight(const Vec3f &intensity, const Vec3f &position) : Emitter(Emitter::Type::POINT), intensity(intensity), position(position) {}

    virtual Vec3f eval(const Intersection &isc) const override {
        return intensity / glm::dot(position - isc.position, position - isc.position);
    }

    EmitterSample sampleLi(const Scene *scene, const Intersection &isc, const Vec3f &sample) const override;

    std::string to_string() const override {
        std::ostringstream oss;
        oss << "Emitter(PointLight): [ intensity=" << intensity << ", position=" << position << " ]";
        return oss.str();
    }
};

// Right now only uniform area emitters are supported.
// (texture emitters are not supported)
class AreaLight final : public Emitter {
private:
    Vec3f radiance;

public:
    Shape *shape;  // the shape that this area light is attached to

    AreaLight(const Vec3f &radiance) : Emitter(Emitter::Type::AREA), radiance(radiance) {}

    // FIXME: validate the physical correctness
    virtual Vec3f eval(const Intersection &isc) const override {
        return radiance;
    }

    virtual EmitterSample sampleLi(const Scene *scene, const Intersection &isc, const Vec3f &sample) const override;
    
    std::string to_string() const override {
        std::ostringstream oss;
        oss << "Emitter(AreaLight): [ radiance=" << radiance << " ]";
        return oss.str();
    }
};
