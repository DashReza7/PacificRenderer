#pragma once

#include <tiny_obj_loader.h>

#include <array>
#include <format>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "core/MathUtils.h"
#include "core/Pacific.h"
#include "core/Registry.h"

class Shape;
class Geometry;

bool load_mesh_from_file(const std::string &file_path, const Shape *parent_shape, std::vector<Geometry *> &output_mesh, std::vector<Vec3f *> &vertices, std::vector<Vec3f *> &normals, std::vector<Vec2f *> &texcoords);


/// @brief A ray in 3D space, defined by an origin and a direction
/// d must be normalized. tmin and tmax define the valid interval along the ray.
/// shadow_ray indicates whether this ray is a shadow ray (for optimization purposes)
struct Ray {
    Vec3f o;
    Vec3f d;
    Float tmin, tmax;
    bool shadow_ray;

    Ray(const Vec3f &o, const Vec3f &d, Float tmin, Float tmax, bool shadow_ray = false)
        : o(o), d(d), tmin(tmin), tmax(tmax), shadow_ray(shadow_ray) {}

    /// return the position of the point with distance t along the ray
    Vec3f operator()(Float t) const { return o + t * d; }
};

struct AABB {
    Vec3f min_corner, max_corner;

    AABB() = default;
    AABB(const Vec3f &min_corner, const Vec3f &max_corner)
        : min_corner(min_corner), max_corner(max_corner) {}

    // union operator
    AABB operator+(const AABB &other) const {
        return AABB{Vec3f{std::min(min_corner.x, other.min_corner.x),
                          std::min(min_corner.y, other.min_corner.y),
                          std::min(min_corner.z, other.min_corner.z)},
                    Vec3f{std::max(max_corner.x, other.max_corner.x),
                          std::max(max_corner.y, other.max_corner.y),
                          std::max(max_corner.z, other.max_corner.z)}};
    }
};

struct Intersection {
    Float distance;
    Vec3f position, normal;
    /// Normalized direction, from the hit position to the ray origin
    Vec3f dirn;
    const Shape *shape;
    const Geometry *geom;
};

// Context for creating a geometry from a mesh file
struct GeometryCreationContext {
    std::array<const Vec3f *, 3> vp = {nullptr, nullptr, nullptr};
    std::array<const Vec3f *, 3> vn = {nullptr, nullptr, nullptr};
    std::array<const Vec2f *, 3> vt = {nullptr, nullptr, nullptr};

    GeometryCreationContext() = default;
    GeometryCreationContext(const Vec3f *v0, const Vec3f *v1, const Vec3f *v2, const Vec3f *n0 = nullptr, const Vec3f *n1 = nullptr, const Vec3f *n2 = nullptr, const Vec2f *t0 = nullptr, const Vec2f *t1 = nullptr, const Vec2f *t2 = nullptr)
        : vp{v0, v1, v2}, vn{n0, n1, n2}, vt{t0, t1, t2} {}
};

class Geometry {
public:
    const Shape *parent_shape;

    virtual AABB get_bbox() const = 0;
    virtual bool intersect(const Ray &ray, Intersection &isc) const = 0;
    virtual Vec3f get_normal(const Vec3f &position) const = 0;
    virtual Float area() const = 0;
    /// @brief Samples a point on the surface of the geometry.
    /// @param sample a 2D sample point in [0, 1]^2.
    /// @return A tuple containing the position, normal, and PDF of the sampled point.
    virtual std::tuple<Vec3f, Vec3f, Float> sample_point_on_surface(const Vec2f &sample) const = 0;
    virtual std::string to_string() const = 0;
};

enum class AccelerationType {
    NONE,
    BVH,
};

class BVHNode {
public:
    BVHNode *left, *right;
    AABB bbox;
    std::vector<Geometry *> geoms{};

    BVHNode() = default;
    BVHNode(BVHNode *left_, BVHNode *right_, AABB bbox_) : left(left_), right(right_), bbox(bbox_) {}

    bool intersect(const Ray &ray, Intersection &isc);
};
