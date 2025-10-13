#include <algorithm>

#include "core/Geometry.h"
#include "core/Registry.h"
#include "utils/Misc.h"


class Sphere : public Geometry {
private:
    Mat4f inv_transform;

public:
    Vec3f center;
    Float radius;
    Float radius_world;  // radius in world space
    // the center and radius are in local space, `transform` is used to transform. used for intersection and bbox building
    Mat4f transform;
    bool flip_normals;

    Sphere(const Vec3f &center, Float radius, const Mat4f &transform, const Mat4f &inv_transform, const Shape *parent_shape, bool flip_normals) : transform(transform), inv_transform(inv_transform), center(center), radius(radius), flip_normals(flip_normals) {
        Vec3f scaled_x = Vec3f{transform[0]};
        radius_world = radius * glm::length(scaled_x);
        this->parent_shape = parent_shape;
    }

    bool intersect(const Ray &ray, Intersection &isc) const override {
        // transform ray to local space
        Vec3f o_local = Vec3f{inv_transform * Vec4f{ray.o, 1.0}};
        Vec3f d_local = glm::normalize(Vec3f{inv_transform * Vec4f{ray.d, 0.0}});

        Vec3f o_minus_c = o_local - center;
        Float o_minus_c_length2 = dot(o_minus_c, o_minus_c);
        Float b_prime = dot(o_minus_c, d_local);
        Float delta_prime = Sqr(b_prime) - o_minus_c_length2 + Sqr(radius);

        // no intersection (in any direction)
        if (delta_prime <= 0.0)
            return false;

        // tangent to the sphere
        if (delta_prime < Epsilon) {
            Float t_local = -b_prime;
            // hit from behind
            if (t_local < 0.0)
                return false;
            Vec3f local_hit_pos = o_local + t_local * d_local;
            isc.position = Vec3f{transform * Vec4f{local_hit_pos, 1.0}};
            isc.normal = get_normal(isc.position);
        } else {  // hit the sphere twice (might be in the back or front of ray)
            Float delta_prime_sqrt = std::sqrt(delta_prime);
            Float t1_local = -b_prime - delta_prime_sqrt;
            Float t2_local = -b_prime + delta_prime_sqrt;
            // both hitpoints are behind the ray
            if (t2_local <= 0.0)
                return false;
            if (t1_local >= 0.0) {
                Vec3f local_hit_pos = o_local + t1_local * d_local;
                isc.position = Vec3f{transform * Vec4f{local_hit_pos, 1.0}};
                isc.distance = glm::length(isc.position - ray.o);
                if (isc.distance < ray.tmin || isc.distance > ray.tmax)
                    goto lbl1;
                isc.normal = get_normal(isc.position);
            } else {  // if (t2_local >= 0.0)
            lbl1:
                Vec3f local_hit_pos = o_local + t2_local * d_local;
                isc.position = Vec3f{transform * Vec4f{o_local + t2_local * d_local, 1.0}};
                isc.distance = glm::length(isc.position - ray.o);
                if (isc.distance > ray.tmax || isc.distance < ray.tmin)
                    return false;
                isc.normal = get_normal(isc.position);
            }
        }

        isc.dirn = glm::normalize(ray.o - isc.position);
        isc.shape = parent_shape;
        isc.geom = this;
        return true;
    }

    AABB get_bbox() const override {
        Vec3f corner_1 = center + Vec3f{radius, radius, radius};
        Vec3f corner_2 = center + Vec3f{radius, radius, -radius};
        Vec3f corner_3 = center + Vec3f{radius, -radius, radius};
        Vec3f corner_4 = center + Vec3f{radius, -radius, -radius};
        Vec3f corner_5 = center + Vec3f{-radius, radius, radius};
        Vec3f corner_6 = center + Vec3f{-radius, radius, -radius};
        Vec3f corner_7 = center + Vec3f{-radius, -radius, radius};
        Vec3f corner_8 = center + Vec3f{-radius, -radius, -radius};
        corner_1 = Vec3f{transform * Vec4f{corner_1, 1.0}};
        corner_2 = Vec3f{transform * Vec4f{corner_2, 1.0}};
        corner_3 = Vec3f{transform * Vec4f{corner_3, 1.0}};
        corner_4 = Vec3f{transform * Vec4f{corner_4, 1.0}};
        corner_5 = Vec3f{transform * Vec4f{corner_5, 1.0}};
        corner_6 = Vec3f{transform * Vec4f{corner_6, 1.0}};
        corner_7 = Vec3f{transform * Vec4f{corner_7, 1.0}};
        corner_8 = Vec3f{transform * Vec4f{corner_8, 1.0}};
        return AABB{Vec3f{std::min(std::min(std::min(corner_1.x, corner_2.x), std::min(corner_3.x, corner_4.x)), std::min(std::min(corner_5.x, corner_6.x), std::min(corner_7.x, corner_8.x))) - Epsilon,
                          std::min(std::min(std::min(corner_1.y, corner_2.y), std::min(corner_3.y, corner_4.y)), std::min(std::min(corner_5.y, corner_6.y), std::min(corner_7.y, corner_8.y))) - Epsilon,
                          std::min(std::min(std::min(corner_1.z, corner_2.z), std::min(corner_3.z, corner_4.z)), std::min(std::min(corner_5.z, corner_6.z), std::min(corner_7.z, corner_8.z))) - Epsilon},
                    Vec3f{std::max(std::max(std::max(corner_1.x, corner_2.x), std::max(corner_3.x, corner_4.x)), std::max(std::max(corner_5.x, corner_6.x), std::max(corner_7.x, corner_8.x))) + Epsilon,
                          std::max(std::max(std::max(corner_1.y, corner_2.y), std::max(corner_3.y, corner_4.y)), std::max(std::max(corner_5.y, corner_6.y), std::max(corner_7.y, corner_8.y))) + Epsilon,
                          std::max(std::max(std::max(corner_1.z, corner_2.z), std::max(corner_3.z, corner_4.z)), std::max(std::max(corner_5.z, corner_6.z), std::max(corner_7.z, corner_8.z))) + Epsilon}};
    }

    Vec3f get_normal(const Vec3f &position) const override {
        Vec3f world_center = Vec3f{transform * Vec4f{center, 1.0}};
        Vec3f normal = glm::normalize(position - world_center);
        return flip_normals ? -normal : normal;
    }

    Float area() const override {
        return 4.0 * Pi * Sqr(radius_world);
    }

    std::tuple<Vec3f, Vec3f, Float> sample_point_on_surface(const Vec2f &sample) const override {
        // uniform sampling on sphere surface
        Float phi = 2.0 * Pi * sample.x;
        Float theta = acos(1.0 - 2.0 * sample.y);
        Vec3f normal{
            sin(theta) * cos(phi),
            sin(theta) * sin(phi),
            cos(theta)};
        Vec3f position = center + radius * normal;
        position = Vec3f{transform * Vec4f{position, 1.0}};
        Float pdf = 1.0 / (4.0 * Pi * Sqr(radius_world));

        return {position, get_normal(position), pdf};
    }

    Vec2f get_uv(const Vec3f &posn) const override {
        Vec3f posn_local = Vec3f{inv_transform * Vec4f{posn, 1.0}};
        posn_local -= center;
        posn_local /= radius;
        Float cosTheta = std::clamp(posn_local.z, Float(-1.0), Float(1.0));
        Float theta = std::acos(cosTheta);
        Float phi = std::atan2(posn_local.y, posn_local.x);  // phi ∈ [-pi, pi]
        return Vec2f{phi / (2.0 * Pi) + 0.5, theta / Pi};
    }

    std::string to_string() const override {
        std::ostringstream oss;
        oss << "Geometry(Sphere): [ local_center=" << center << ", local_radius=" << radius << " ]";
        return oss.str();
    }
};

// --------------------------- Registry functions ---------------------------
Geometry *createSphere(const std::unordered_map<std::string, std::string> &properties, const Shape *parent_shape, const GeometryCreationContext *ctx) {
    Vec3f center{0.0, 0.0, 0.0};
    Float radius = 1.0;
    Mat4f transform{1.0};
    Mat4f inv_transform{1.0};
    bool flip_normals = false;

    for (const auto &[key, value] : properties) {
        if (key == "center") {
            center = strToVec3f(value);
        } else if (key == "radius") {
            radius = std::stod(value);
        } else if (key == "to_world") {
            if (!properties.contains("inv_to_world"))
                throw std::runtime_error("Sphere geometry requires 'inv_to_world' property when 'to_world' is provided");
            transform = strToMat4f(value);
        } else if (key == "inv_to_world") {
            if (!properties.contains("to_world"))
                throw std::runtime_error("Sphere geometry requires 'to_world' property when 'inv_to_world' is provided");
            inv_transform = strToMat4f(value);
        } else if (key == "flip_normals") {
            flip_normals = (value == "true" || value == "1");
        } else {
            throw std::runtime_error("Unknown property '" + key + "' for sphere geometry");
        }
    }

    return new Sphere{center, radius, transform, inv_transform, parent_shape, flip_normals};
}

namespace {
struct SphereRegistrar {
    SphereRegistrar() {
        GeometryRegistry::registerGeometry("sphere", createSphere);
    }
};

static SphereRegistrar registrar;
}  // namespace
