#pragma once
#include "core/MathUtils.h"

class Shape;

// Right now only RGB emitters are supported
class Emitter {
public:
    enum class Type {
        POINT,
        AREA
    };
    Type type;

    Emitter(Emitter::Type type) : type(type) {}
    virtual std::string to_string() = 0;
};

class PointLight final : public Emitter {
public:
    Vec3f intensity;
    Vec3f position;
    Mat4f to_world;

    PointLight(const Vec3f &intensity, const Vec3f &position, const Mat4f &to_world) : Emitter(Emitter::Type::POINT), intensity(intensity), position(position), to_world(to_world) {}

    std::string to_string() override {
        std::ostringstream oss;
        oss << "Emitter(PointLight): [ intensity=" << intensity << ", position=" << position << " ]";
        return oss.str();
    }
};

// Right now only uniform area emitters are supported.
// (texture emitters are not supported)
class AreaLight final : public Emitter {
public:
    Vec3f radiance;
    Shape *shape; // the shape that this area light is attached to

    AreaLight(const Vec3f &radiance) : Emitter(Emitter::Type::AREA), radiance(radiance) {}

    std::string to_string() override {
        std::ostringstream oss;
        oss << "Emitter(AreaLight): [ radiance=" << radiance << " ]";
        return oss.str();
    }
};
