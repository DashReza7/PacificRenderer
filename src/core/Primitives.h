#pragma once

#include "core/Shape.h"

class Ray {
   public:
    /// Ray origin
    Vec3f o;
    /// Ray direction
    Vec3f d;
    /// Minimum and maximum hittable distance
    Float tmin, tmax;
    /// Whether this is a shadow ray (any hit)
    bool shadow_ray = false;

    Ray() = default;
    Ray(const Vec3f& o, const Vec3f& d, Float tmin, Float tmax)
        : o(o), d(d), tmin(tmin), tmax(tmax) {}
    ~Ray() = default;

    /// return the position of the point with distance t along the ray
    Vec3f operator()(Float t) { return o + t * d; }

    /// return the same ray with reversed direrction
    Ray reverse() const { return Ray{o, -d, tmin, tmax}; }
};

class AABB {
   public:
    Vec3f min_corner, max_corner;

    AABB() = default;
    AABB(const Vec3f& min_corner, const Vec3f& max_corner)
        : min_corner(min_corner), max_corner(max_corner) {}
    ~AABB() = default;
};

class Intersection {
   public:
    Vec3f position, normal;
    std::shared_ptr<Shape> shape;

    Intersection() = default;
    Intersection(const Vec3f& position, const Vec3f& normal, const std::shared_ptr<Shape> shape)
        : position(position), normal(normal), shape(shape) {}
    ~Intersection() = default;
};
