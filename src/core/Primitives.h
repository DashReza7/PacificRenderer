#pragma once

#include <memory>
#include <string>

#include "core/MathUtils.h"
#include "core/Pacific.h"

class Shape;

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
    Vec3f operator()(Float t) const { return o + t * d; }

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

    // union operator
    AABB operator+(const AABB& other) const {
        return AABB{Vec3f{std::min(min_corner.x, other.min_corner.x),
                          std::min(min_corner.y, other.min_corner.y),
                          std::min(min_corner.z, other.min_corner.z)},
                    Vec3f{std::max(max_corner.x, other.max_corner.x),
                          std::max(max_corner.y, other.max_corner.y),
                          std::max(max_corner.z, other.max_corner.z)}};
    }
    
};

class Intersection {
public:
    Vec3f position, normal;
    Shape* shape;

    Intersection() = default;
    Intersection(const Vec3f& position, const Vec3f& normal, Shape* shape) : position(position), normal(normal), shape(shape) {}
    ~Intersection() = default;
};

class Geometry {
public:
    Shape *parent_shape;

    Geometry() = default;
    virtual ~Geometry() = default;

    enum class Type { Triangle,
                      Quad,
                      Sphere };

    virtual Type get_type() = 0;
    virtual AABB get_bbox() = 0;
    virtual bool intersect(const Ray &ray, Intersection &isc) = 0;
    virtual Vec3f get_normal(const Vec3f &position) = 0;
    virtual std::string to_string() = 0;
};

class BVHNode {
public:
    BVHNode *left, *right;
    AABB bbox;
    std::vector<Geometry *> geoms{};

    BVHNode() = default;
    BVHNode(BVHNode* left_, BVHNode* right_, AABB bbox_) : left(left_), right(right_), bbox(bbox_) {}

    bool intersect(const Ray& ray, Intersection& isc);
};
