#include "core/Geometry.h"
#include "core/Registry.h"
#include "utils/Misc.h"

class Disk : public Geometry {
public:
    Mat4f transform;
    Mat4f inv_transform;
    Float world_radius;
    Vec3f world_normal;
    bool flip_normals;

    Disk(const Mat4f &transform, const Mat4f &inv_transform, bool flip_normals, const Shape *parent_shape) {
        this->parent_shape = parent_shape;
        this->transform = transform;
        // check for non-uniform scaling
        Float sx = glm::length(Vec3f{transform[0]});
        Float sy = glm::length(Vec3f{transform[1]});
        Float sz = glm::length(Vec3f{transform[2]});
        if (std::abs(sx - sy) > Epsilon || std::abs(sx - sz) > Epsilon || std::abs(sy - sz) > Epsilon)
            throw std::runtime_error("Disk geometry does not support non-uniform scaling: " + std::to_string(sx) + ", " + std::to_string(sy) + ", " + std::to_string(sz));
        world_radius = sx;
        this->inv_transform = inv_transform;
        this->flip_normals = flip_normals;
        world_normal = glm::normalize(Vec3f{glm::transpose(inv_transform)[2]});
    }

    bool intersect(const Ray &ray, Intersection &isc) const override {
        Vec3f local_o = Vec3f{inv_transform * Vec4f{ray.o, 1.0}};
        Vec3f local_d = Vec3f{inv_transform * Vec4f{ray.d, 0.0}};
        if (std::abs(local_d.z) < Epsilon)
            return false;
        Float t = -local_o.z / local_d.z;
        if (t <= 0.0)
            return false;
        Vec3f local_posn = local_o + t * local_d;
        if (Sqr(local_posn.x) + Sqr(local_posn.y) > 1.0)
            return false;
        Vec3f world_posn = Vec3f{transform * Vec4f{local_posn, 1.0}};
        Float distance = glm::length(world_posn - ray.o);
        if (distance < ray.tmin || distance > ray.tmax)
            return false;

        isc.dirn = -ray.d;
        isc.distance = distance;
        isc.position = world_posn;
        isc.normal = get_normal(world_posn);
        isc.shape = parent_shape;
        isc.geom = this;
        return true;
    }

    AABB get_bbox() const override {
        std::vector<Vec3f> vertices = {
            Vec3f{-1.0, -1.0, -0.01},
            Vec3f{-1.0, 1.0, -0.01},
            Vec3f{1.0, -1.0, -0.01},
            Vec3f{1.0, 1.0, -0.01},
            Vec3f{-1.0, -1.0, 0.01},
            Vec3f{-1.0, 1.0, 0.01},
            Vec3f{1.0, -1.0, 0.01},
            Vec3f{1.0, 1.0, 0.01},
        };
        for (auto &vertex : vertices)
            vertex = Vec3f{transform * Vec4f{vertex, 1.0}};
        // find the min and max coordinates
        Vec3f min = vertices[0];
        Vec3f max = vertices[0];
        for (const auto &vertex : vertices) {
            min = glm::min(min, vertex);
            max = glm::max(max, vertex);
        }
        return AABB{min, max};
    }

    Vec3f get_normal(const Vec3f &position) const override {
        return flip_normals ? -world_normal : world_normal;
    }

    Float area() const override {
        return Pi * Sqr(world_radius);
    }

    std::tuple<Vec3f, Vec3f, Float> sample_point_on_surface(const Vec2f &sample) const override {
        Float r = std::sqrt(sample.x);
        Float theta = 2 * Pi * sample.y;
        Vec3f posn{r * std::cos(theta), r * std::sin(theta), 0.0};
        posn = Vec3f{transform * Vec4f{posn, 1.0}};
        Float pdf = 1.0 / (Pi * Sqr(world_radius));

        return {posn, get_normal(posn), pdf};
    }

    Vec2f get_uv(const Vec3f &posn) const override {
        throw std::runtime_error("disk get_uv not implemented");
    }

    std::string to_string() const override {
        std::ostringstream oss;
        oss << "Geometry(Disk): [ center=" << transform[3] << ", normal=" << world_normal << ", radius=" << world_radius << " ]";
        return oss.str();
    }
};

// --------------------------- Registry functions ---------------------------
Geometry *createDisk(const std::unordered_map<std::string, std::string> &properties, const Shape *parent_shape, const GeometryCreationContext *ctx) {
    Mat4f transform{1.0};
    Mat4f inv_transform{1.0};
    bool flip_normals = false;

    for (const auto &[key, value] : properties) {
        if (key == "to_world") {
            if (!properties.contains("inv_to_world"))
                throw std::runtime_error("Disk geometry requires 'inv_to_world' property when 'to_world' is provided");
            transform = strToMat4f(value);
        } else if (key == "inv_to_world") {
            if (!properties.contains("to_world"))
                throw std::runtime_error("Disk geometry requires 'to_world' property when 'inv_to_world' is provided");
            inv_transform = strToMat4f(value);
        } else if (key == "flip_normals") {
            flip_normals = (value == "true" || value == "1");
        } else {
            throw std::runtime_error("Unknown property '" + key + "' for disk geometry");
        }
    }

    return new Disk{transform, inv_transform, flip_normals, parent_shape};
}

namespace {
struct DiskRegistrar {
    DiskRegistrar() {
        GeometryRegistry::registerGeometry("disk", createDisk);
    }
};

static DiskRegistrar registrar;
}  // namespace
