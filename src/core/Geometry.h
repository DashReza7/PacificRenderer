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

class BVHNode {
public:
    BVHNode *left, *right;
    AABB bbox;
    std::vector<Geometry *> geoms{};

    BVHNode() = default;
    BVHNode(BVHNode *left_, BVHNode *right_, AABB bbox_) : left(left_), right(right_), bbox(bbox_) {}

    bool intersect(const Ray &ray, Intersection &isc);
};

enum class AccelerationType {
    NONE,
    BVH,
};

struct GeometryCreationContext {
    std::array<const Vec3f *, 3> vp = {nullptr, nullptr, nullptr};
    std::array<const Vec3f *, 3> vn = {nullptr, nullptr, nullptr};
    std::array<const Vec2f *, 3> vt = {nullptr, nullptr, nullptr};
};

// TODO: must clean up memory allocated for vertices, normals and texcoords after the rendering is done
class MeshLoader {
public:
    // load a triangle/quad mesh
    static bool load_mesh_from_file(const std::string &file_path, const Shape *parent_shape, std::vector<Geometry *> &output_mesh, std::vector<Vec3f *> &vertices, std::vector<Vec3f *> &normals, std::vector<Vec2f *> &texcoords) {
        tinyobj::ObjReader obj_reader;
        if (!obj_reader.ParseFromFile(file_path)) {
            if (!obj_reader.Error().empty())
                // LOG_ERROR("TinyObjReader error reading file: {}; {}", file_path, obj_reader.Error());
                std::cerr << "TinyObjReader error reading file: " << file_path << "; " << obj_reader.Error() << "\n";
            return false;
        }
        if (!obj_reader.Warning().empty()) {
            std::cout << "TinyObjReader warning reading file: " << file_path << "; " << obj_reader.Warning() << "\n";
        }

        // Only support objs with one shape
        const std::vector<tinyobj::shape_t> &shapes = obj_reader.GetShapes();
        if (shapes.size() == 0) {
            std::cerr << "TinyObjReader warning reading file: " << file_path << "; " << "No shapes in this file\n";
            return false;
        }
        if (shapes.size() > 1) {
            std::cerr << "TinyObjReader warning reading file: " << file_path << "; " << "More than one shape in .obj file not supported\n";
            return false;
        }

        const tinyobj::attrib_t &attrib = obj_reader.GetAttrib();

        // Store vertices, normals and texcoords;
        size_t num_vertices = attrib.vertices.size() / 3;
        for (size_t i = 0; i < num_vertices * 3; i += 3)
            vertices.emplace_back(new Vec3f(attrib.vertices[i], attrib.vertices[i + 1], attrib.vertices[i + 2]));
        size_t num_normals = attrib.normals.size() / 3;
        for (size_t i = 0; i < num_normals * 3; i += 3)
            normals.emplace_back(new Vec3f(attrib.normals[i], attrib.normals[i + 1], attrib.normals[i + 2]));
        size_t num_texcoords = attrib.texcoords.size() / 2;
        for (size_t i = 0; i < num_texcoords * 2; i += 2)
            texcoords.emplace_back(new Vec2f(attrib.texcoords[i], attrib.texcoords[i + 1]));

        // Iterate through faces and fill-up the output_mesh
        tinyobj::mesh_t mesh = shapes[0].mesh;
        size_t index_offset = 0;
        for (size_t f = 0; f < mesh.num_face_vertices.size(); f++) {
            auto num_face_vertices = mesh.num_face_vertices[f];
            if (num_face_vertices == 3) {
                // Triangle
                std::array<int, 3> vertex_indices;
                std::array<int, 3> normal_indices;
                std::array<int, 3> texcoord_indices;
                for (size_t v = 0; v < 3; v++) {
                    tinyobj::index_t index = mesh.indices[index_offset + v];
                    vertex_indices[v] = index.vertex_index;
                    normal_indices[v] = index.normal_index;
                    texcoord_indices[v] = index.texcoord_index;
                }

                GeometryCreationContext gctx{};
                gctx.vp = {vertices[vertex_indices[0]], vertices[vertex_indices[1]], vertices[vertex_indices[2]]};
                if (normal_indices[0] != -1)
                    gctx.vn = {normals[normal_indices[0]], normals[normal_indices[1]], normals[normal_indices[2]]};
                if (texcoord_indices[0] != -1)
                    gctx.vt = {texcoords[texcoord_indices[0]], texcoords[texcoord_indices[1]], texcoords[texcoord_indices[2]]};
                output_mesh.emplace_back(GeometryRegistry::createGeometry("triangle", {}, parent_shape, &gctx));
            } else {
                // Other polygons
                throw std::runtime_error("Quad not implemented");
            }

            index_offset += num_face_vertices;
        }

        return true;
    }
};
