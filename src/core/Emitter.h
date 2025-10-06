#pragma once
#include "core/Geometry.h"
#include "core/MathUtils.h"

// Forward declarations
class Shape;
class Scene;

enum class EmitterFlags {
    NONE = 0,
    DELTA_POSITION = 1 << 0,
    AREA = 1 << 1,
    DELTA_DIRECTION = 1 << 2
};

// TODO: change the position member to dirn
struct EmitterSample {
    Float pdf;
    // direction towards the shading point
    Vec3f direction;
    // Whether there's an occlusion between the isc and the emitter sample point
    // Remark: This does not handle whether the isc.normal is in the correct side
    bool is_valid;
    Vec3f radiance;
    EmitterFlags emitter_flags;

    EmitterSample(Float pdf, const Vec3f &direction, bool is_valid, const Vec3f &radiance, EmitterFlags emitter_flags)
        : pdf(pdf), direction(direction), is_valid(is_valid), radiance(radiance), emitter_flags(emitter_flags) {}
};

class Emitter {
public:
    /// Only used for area lights to set their corresponding shape
    virtual void set_shape(const Shape *shape) {}

    /// Returns the radiance to the shading point.
    /// Remark: this function does not account for occlusions
    virtual Vec3f eval(const Vec3f &shading_posn) const = 0;

    /// Used for NEE. Returned pdf is in solid angle measure(or 1 for Delta light sources)
    virtual EmitterSample sampleLi(const Scene *scene, const Intersection &isc, const Vec3f &sample) const = 0;

    virtual std::string to_string() const = 0;
};
