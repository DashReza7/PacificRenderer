#include "core/Geometry.h"
#include "core/Registry.h"
#include "utils/Misc.h"

class Triangle : public Geometry {
public:
    const std::array<const Vec3f *, 3> positions;
    const std::array<const Vec3f *, 3> normals;
    const std::array<const Vec2f *, 3> tex_coords;

    Triangle(const std::array<const Vec3f *, 3> &positions_, const std::array<const Vec3f *, 3> &normals_, const std::array<const Vec2f *, 3> &tex_coords_, const Shape *parent_shape) : positions(positions_), normals(normals_), tex_coords(tex_coords_) {
        this->parent_shape = parent_shape;
    }

    bool intersect(const Ray &ray, Intersection &isc) const override {
        Vec3f edge1 = *positions[1] - *positions[0];
        Vec3f edge2 = *positions[2] - *positions[0];
        Vec3f h = glm::cross(ray.d, edge2);
        Float a = glm::dot(edge1, h);
        if (a > -Epsilon && a < Epsilon)
            return false;  // parallel to triangle
        Float f = 1.0 / a;
        Vec3f s = ray.o - *positions[0];
        Float u = f * glm::dot(s, h);
        if (u < 0.0 || u > 1.0)
            return false;
        Vec3f q = glm::cross(s, edge1);
        Float v = f * glm::dot(ray.d, q);
        // if (v < 0.0 || u + v > 1.0)
        if (v < -Epsilon || u + v > 1.0 + Epsilon)
            return false;
        Float t = f * glm::dot(edge2, q);
        if (t >= ray.tmin && t <= ray.tmax)  // ray intersection
        {
            isc.distance = t;
            isc.position = ray(t);
            isc.normal = get_normal(isc.position);
            isc.dirn = -ray.d;
            isc.shape = parent_shape;
            isc.geom = this;
            return true;
        } else  // isc point is behind the ray
            return false;
    }

    AABB get_bbox() const override {
        return AABB{Vec3f{std::min(positions[0]->x, std::min(positions[1]->x, positions[2]->x)) - Epsilon, std::min(positions[0]->y, std::min(positions[1]->y, positions[2]->y)) - Epsilon, std::min(positions[0]->z, std::min(positions[1]->z, positions[2]->z)) - Epsilon},
                    Vec3f{std::max(positions[0]->x, std::max(positions[1]->x, positions[2]->x)) + Epsilon, std::max(positions[0]->y, std::max(positions[1]->y, positions[2]->y)) + Epsilon, std::max(positions[0]->z, std::max(positions[1]->z, positions[2]->z)) + Epsilon}};
    }

    Vec3f get_normal(const Vec3f &position) const override {
        // based on face_normals, either interpolate vn's or compute the
        // (constant) normal manually
        if (normals[0] != nullptr) {
            Vec3f bary_coords = barycentric(*positions[0], *positions[1], *positions[2], position);
            return bary_coords.x * (*normals[0]) + bary_coords.y * (*normals[1]) + bary_coords.z * (*normals[2]);
        } else {
            return glm::normalize(glm::cross(*positions[1] - *positions[0], *positions[2] - *positions[0]));
        }
    }

    Float area() const override {
        return triangle_area(*positions[0], *positions[1], *positions[2]);
    }

    std::tuple<Vec3f, Vec3f, Float> sample_point_on_surface(const Vec2f &sample) const override {
        // uniform sampling on triangle surface
        Float r_sqrd = std::sqrt(sample.x);
        Vec3f rnd_pt = *positions[0] * (Float(1.0) - sample.y) * r_sqrd
                     + *positions[1] * (Float(1.0) - r_sqrd)
                     + *positions[2] * sample.y * r_sqrd;
        Vec3f normal = get_normal(rnd_pt);
        Float pdf = 1.0 / triangle_area(*positions[0], *positions[1], *positions[2]);
        
        return {rnd_pt, normal, pdf};
    }

    std::string to_string() const override {
        std::ostringstream oss;
        oss << "Geometry(Triangle): [";
        oss << " positions=" << std::format("[{}, {}, {}] - [{}, {}, {}] - [{}, {}, {}]", positions[0]->x, positions[0]->y, positions[0]->z, positions[1]->x, positions[1]->y, positions[1]->z, positions[2]->x, positions[2]->y, positions[2]->z);
        if (normals[0] != nullptr)
            oss << " --- normals=" << std::format("[{}, {}, {}] - [{}, {}, {}] - [{}, {}, {}]", normals[0]->x, normals[0]->y, normals[0]->z, normals[1]->x, normals[1]->y, normals[1]->z, normals[2]->x, normals[2]->y, normals[2]->z);
        if (tex_coords[0] != nullptr)
            oss << " --- texcoords=" << std::format("[{}, {}] - [{}, {}] - [{}, {}] ", tex_coords[0]->x, tex_coords[0]->y, tex_coords[1]->x, tex_coords[1]->y, tex_coords[2]->x, tex_coords[2]->y);
        oss << "]";
        return oss.str();
    }
};

// --------------------------- Registry functions ---------------------------
Geometry *createTriangle(const std::unordered_map<std::string, std::string> &properties, const Shape *parent_shape, const GeometryCreationContext *ctx) {
    if (properties.size() > 0) {
        throw std::runtime_error("Triangle geometry does not take any properties");
    }
    auto triangle = new Triangle{ctx->vp, ctx->vn, ctx->vt, parent_shape};
    return triangle;
}

namespace {
struct TriangleRegistrar {
    TriangleRegistrar() {
        GeometryRegistry::registerGeometry("triangle", createTriangle);
    }
};

static TriangleRegistrar registrar;
}  // namespace
