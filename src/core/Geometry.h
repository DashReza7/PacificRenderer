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

class Shape;
class Geometry;

struct Ray {
    /// Ray origin
    Vec3f o;
    /// Ray direction
    Vec3f d;
    /// Minimum and maximum hittable distance
    Float tmin, tmax;
    /// Whether this is a shadow ray (any hit)
    bool shadow_ray;

    Ray() = default;
    Ray(const Vec3f &o, const Vec3f &d, Float tmin, Float tmax, bool shadow_ray = false)
        : o(o), d(d), tmin(tmin), tmax(tmax), shadow_ray(shadow_ray) {}
    ~Ray() = default;

    /// return the position of the point with distance t along the ray
    Vec3f operator()(Float t) const { return o + t * d; }

    /// return the same ray with reversed direrction
    Ray reverse() const { return Ray{o, -d, tmin, tmax}; }
};

struct AABB {
    Vec3f min_corner, max_corner;

    AABB() = default;
    AABB(const Vec3f &min_corner, const Vec3f &max_corner)
        : min_corner(min_corner), max_corner(max_corner) {}
    ~AABB() = default;

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
    Shape *shape;
    Geometry *geom;
};

class Geometry {
public:
    Shape *parent_shape;

    virtual AABB get_bbox() = 0;
    virtual bool intersect(const Ray &ray, Intersection &isc) = 0;
    virtual Vec3f get_normal(const Vec3f &position) = 0;
    virtual Float area() const = 0;
    virtual std::string to_string() = 0;
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

class Triangle : public Geometry {
public:
    const std::array<const Vec3f *, 3> positions;
    const std::optional<std::array<const Vec3f *, 3>> normals;
    const std::optional<std::array<const Vec2f *, 3>> tex_coords;

    Triangle(const Vec3f *a, const Vec3f *b, const Vec3f *c) : positions{a, b, c} {}
    Triangle(const Vec3f *a, const Vec3f *b, const Vec3f *c, const Vec3f *an, const Vec3f *bn, const Vec3f *cn) : positions{a, b, c}, normals{std::array<const Vec3f *, 3>{an, bn, cn}} {}
    Triangle(const Vec3f *a, const Vec3f *b, const Vec3f *c, const Vec2f *at, const Vec2f *bt, const Vec2f *ct) : positions{a, b, c}, tex_coords{std::array<const Vec2f *, 3>{at, bt, ct}} {}
    Triangle(const Vec3f *a, const Vec3f *b, const Vec3f *c, const Vec3f *an, const Vec3f *bn, const Vec3f *cn, const Vec2f *at, const Vec2f *bt, const Vec2f *ct) : positions{a, b, c}, normals{std::array<const Vec3f *, 3>{an, bn, cn}}, tex_coords{std::array<const Vec2f *, 3>{at, bt, ct}} {}

    bool intersect(const Ray &ray, Intersection &isc) override {
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

    AABB get_bbox() override {
        return AABB{Vec3f{std::min(positions[0]->x, std::min(positions[1]->x, positions[2]->x)) - Epsilon, std::min(positions[0]->y, std::min(positions[1]->y, positions[2]->y)) - Epsilon, std::min(positions[0]->z, std::min(positions[1]->z, positions[2]->z)) - Epsilon},
                    Vec3f{std::max(positions[0]->x, std::max(positions[1]->x, positions[2]->x)) + Epsilon, std::max(positions[0]->y, std::max(positions[1]->y, positions[2]->y)) + Epsilon, std::max(positions[0]->z, std::max(positions[1]->z, positions[2]->z)) + Epsilon}};
    }

    Vec3f get_normal(const Vec3f &position) override {
        // based on face_normals, either interpolate vn's or compute the
        // (constant) normal manually
        if (normals.has_value())  // TODO: implement the interpolation
            return glm::normalize(glm::cross(*positions[1] - *positions[0], *positions[2] - *positions[0]));
        else
            return glm::normalize(glm::cross(*positions[1] - *positions[0], *positions[2] - *positions[0]));
    }

    Float area() const override {
        return triangle_area(*positions[0], *positions[1], *positions[2]);
    }
    
    std::string to_string() override {
        std::ostringstream oss;
        oss << "Geometry(Triangle): [";
        oss << " positions=" << std::format("[{}, {}, {}] - [{}, {}, {}] - [{}, {}, {}]", positions[0]->x, positions[0]->y, positions[0]->z, positions[1]->x, positions[1]->y, positions[1]->z, positions[2]->x, positions[2]->y, positions[2]->z);
        if (normals.has_value())
            oss << " --- normals=" << std::format("[{}, {}, {}] - [{}, {}, {}] - [{}, {}, {}]", normals.value()[0]->x, normals.value()[0]->y, normals.value()[0]->z, normals.value()[1]->x, normals.value()[1]->y, normals.value()[1]->z, normals.value()[2]->x, normals.value()[2]->y, normals.value()[2]->z);
        if (tex_coords.has_value())
            oss << " --- texcoords=" << std::format("[{}, {}] - [{}, {}] - [{}, {}] ", tex_coords.value()[0]->x, tex_coords.value()[0]->y, tex_coords.value()[1]->x, tex_coords.value()[1]->y, tex_coords.value()[2]->x, tex_coords.value()[2]->y);
        oss << "]";
        return oss.str();
    }
};

class Quad : public Geometry {
public:
    const std::array<const Vec3f *, 4> positions;
    const std::optional<std::array<const Vec3f *, 4>> normals;
    const std::optional<std::array<const Vec2f *, 4>> tex_coords;

    Quad(const Vec3f *a, const Vec3f *b, const Vec3f *c, const Vec3f *d) : positions{a, b, c} {}
    Quad(const Vec3f *a, const Vec3f *b, const Vec3f *c, const Vec3f *d, const Vec3f *an, const Vec3f *bn, Vec3f *cn, const Vec3f *dn) : positions{a, b, c, d}, normals{std::array<const Vec3f *, 4>{an, bn, cn, dn}} {}
    Quad(const Vec3f *a, const Vec3f *b, const Vec3f *c, const Vec3f *d, const Vec2f *at, const Vec2f *bt, const Vec2f *ct, const Vec2f *dt) : positions{a, b, c, d}, tex_coords{std::array<const Vec2f *, 4>{at, bt, ct, dt}} {}
    Quad(const Vec3f *a, const Vec3f *b, const Vec3f *c, const Vec3f *d, const Vec3f *an, const Vec3f *bn, Vec3f *cn, const Vec3f *dn, const Vec2f *at, const Vec2f *bt, const Vec2f *ct, const Vec2f *dt) : positions{a, b, c, d}, normals{std::array<const Vec3f *, 4>{an, bn, cn, dn}}, tex_coords{std::array<const Vec2f *, 4>{at, bt, ct, dt}} {}

    bool intersect(const Ray &ray, Intersection &isc) override {
        throw std::runtime_error("Quad intersect not implemented");
    }

    AABB get_bbox() override {
        return AABB{Vec3f{std::min(std::min(positions[0]->x, positions[1]->x), std::min(positions[2]->x, positions[3]->x)) - Epsilon,
                          std::min(std::min(positions[0]->y, positions[1]->y), std::min(positions[2]->y, positions[3]->y)) - Epsilon,
                          std::min(std::min(positions[0]->z, positions[1]->z), std::min(positions[2]->z, positions[3]->z)) - Epsilon},
                    Vec3f{std::max(std::max(positions[0]->x, positions[1]->x), std::max(positions[2]->x, positions[3]->x)) + Epsilon,
                          std::max(std::max(positions[0]->y, positions[1]->y), std::max(positions[2]->y, positions[3]->y)) + Epsilon,
                          std::max(std::max(positions[0]->z, positions[1]->z), std::max(positions[2]->z, positions[3]->z)) + Epsilon}};
    }

    Vec3f get_normal(const Vec3f &position) override {
        // based on face_normals, either interpolate vn's or compute the
        // (constant) normal manually
        if (normals.has_value())  // TODO:
            throw std::runtime_error("Quad get_normal not implemented");
        else
            return glm::normalize(glm::cross(*positions[1] - *positions[0], *positions[2] - *positions[0]));
    }

    Float area() const override {
        throw std::runtime_error("Quad area not implemented");
    }
    
    std::string to_string() override {
        throw std::runtime_error("Quad to_string not implemented");
    }
};

class Sphere : public Geometry {
private:
    Mat4f inv_transform;
    
public:
    Vec3f center;
    Float radius;
    Float radius_world;  // radius in world space
    // the center and radius are in local space, `transform` is used to transform. used for intersection and bbox building
    Mat4f transform = glm::mat<4, 4, Float>(1);

    Sphere(const Vec3f &center, Float radius, const Mat4f &transform = Mat4f(1)) : transform(transform), center(center), radius(radius) {
        inv_transform = glm::inverse(transform);
        Vec3f scaled_x = Vec3f{transform * Vec4f{1, 0, 0, 0}};
        radius_world = radius * glm::length(scaled_x);
    }

    // TODO: check for validity
    bool intersect(const Ray &ray, Intersection &isc) override {
        // transform ray to local space
        Vec3f o_local = Vec3f{inv_transform * Vec4f{ray.o, 1.0}};
        Vec3f d_local = glm::normalize(Vec3f{inv_transform * Vec4f{ray.d, 0.0}});

        Vec3f o_minus_c = o_local - center;
        Float o_minus_c_length2 = dot(o_minus_c, o_minus_c);
        Float b_prime = dot(o_minus_c, d_local);
        Float delta_prime = Sqr(b_prime) - o_minus_c_length2 + Sqr(radius);

        // no intersection (in any direction)
        if (delta_prime <= 0)
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
                isc.normal = get_normal(isc.position);
            } else {  // if (t2_local >= 0.0)
                Vec3f local_hit_pos = o_local + t2_local * d_local;
                isc.position = Vec3f{transform * Vec4f{o_local + t2_local * d_local, 1.0}};
                isc.normal = get_normal(isc.position);
            }
        }

        isc.dirn = ray.o - isc.position;
        isc.distance = glm::length(isc.dirn);
        isc.dirn = glm::normalize(isc.dirn);
        isc.shape = parent_shape;
        isc.geom = this;

        // FIXME: there's a bug here. suppose the ray hits the sphere twice. the first hit is really close. then isc.distance may be smaller than ray.tmin
        return (isc.distance >= ray.tmin && isc.distance <= ray.tmax);
    }

    AABB get_bbox() override {
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

    Vec3f get_normal(const Vec3f &position) override {
        Vec3f local_position = Vec3f{inv_transform * Vec4f{position, 1.0}};
        Vec3f local_normal = local_position - center;
        Vec3f normal = Vec3f{glm::transpose(inv_transform) * Vec4f{local_normal, 0.0}};
        return glm::normalize(normal);
    }

    Float area() const override {
        return 4.0 * Pi * Sqr(radius_world);
    }
    
    std::string to_string() override {
        std::ostringstream oss;
        oss << "Geometry(Sphere): [ center=" << center << ", radius=" << radius << " ]";
        return oss.str();
    }
};

// TODO: must clean up memory allocated for vertices, normals and texcoords after the rendering is done
class MeshLoader {
public:
    // load a triangle/quad mesh
    static bool load_mesh_from_file(const std::string &file_path, std::vector<Geometry *> &output_mesh, std::vector<Vec3f *> &vertices, std::vector<Vec3f *> &normals, std::vector<Vec2f *> &texcoords) {
        tinyobj::ObjReader obj_reader;
        if (!obj_reader.ParseFromFile(file_path)) {
            if (!obj_reader.Error().empty())
                // LOG_ERROR("TinyObjReader error reading file: {}; {}", file_path, obj_reader.Error());
                std::cerr << "TinyObjReader error reading file: " << file_path << "; " << obj_reader.Error() << "\n";
            return false;
        }
        if (!obj_reader.Warning().empty())
            std::cout << "TinyObjReader warning reading file: " << file_path << "; " << obj_reader.Warning() << "\n";

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

                if (normal_indices[0] == -1 && texcoord_indices[0] == -1)
                    output_mesh.push_back(new Triangle{vertices[vertex_indices[0]], vertices[vertex_indices[1]], vertices[vertex_indices[2]]});
                else if (normal_indices[0] == -1)
                    output_mesh.push_back(new Triangle{vertices[vertex_indices[0]], vertices[vertex_indices[1]], vertices[vertex_indices[2]], texcoords[texcoord_indices[0]], texcoords[texcoord_indices[1]], texcoords[texcoord_indices[2]]});
                else if (texcoord_indices[0] == -1)
                    output_mesh.push_back(new Triangle{vertices[vertex_indices[0]], vertices[vertex_indices[1]], vertices[vertex_indices[2]], normals[normal_indices[0]], normals[normal_indices[1]], normals[normal_indices[2]]});
                else
                    output_mesh.push_back(new Triangle{vertices[vertex_indices[0]], vertices[vertex_indices[1]], vertices[vertex_indices[2]], normals[normal_indices[0]], normals[normal_indices[1]], normals[normal_indices[2]], texcoords[texcoord_indices[0]], texcoords[texcoord_indices[1]], texcoords[texcoord_indices[2]]});

            } else if (num_face_vertices == 4) {
                // Quad
                throw std::runtime_error("Quad not implemented");
                
                std::array<int, 4> vertex_indices;
                std::array<int, 4> normal_indices;
                std::array<int, 4> texcoord_indices;
                for (size_t v = 0; v < 4; v++) {
                    tinyobj::index_t index = mesh.indices[index_offset + v];
                    vertex_indices[v] = index.vertex_index;
                    normal_indices[v] = index.normal_index;
                    texcoord_indices[v] = index.texcoord_index;
                }

                if (normal_indices[0] == -1 && texcoord_indices[0] == -1)
                    output_mesh.push_back(new Quad{vertices[vertex_indices[0]], vertices[vertex_indices[1]], vertices[vertex_indices[2]], vertices[vertex_indices[3]]});
                else if (normal_indices[0] == -1)
                    output_mesh.push_back(new Quad{vertices[vertex_indices[0]], vertices[vertex_indices[1]], vertices[vertex_indices[2]], vertices[vertex_indices[3]], texcoords[texcoord_indices[0]], texcoords[texcoord_indices[1]], texcoords[texcoord_indices[2]], texcoords[texcoord_indices[3]]});
                else if (texcoord_indices[0] == -1)
                    output_mesh.push_back(new Quad{vertices[vertex_indices[0]], vertices[vertex_indices[1]], vertices[vertex_indices[2]], vertices[vertex_indices[3]], normals[normal_indices[0]], normals[normal_indices[1]], normals[normal_indices[2]], normals[normal_indices[3]]});
                else
                    output_mesh.push_back(new Quad{vertices[vertex_indices[0]], vertices[vertex_indices[1]], vertices[vertex_indices[2]], vertices[vertex_indices[3]], normals[normal_indices[0]], normals[normal_indices[1]], normals[normal_indices[2]], normals[normal_indices[3]], texcoords[texcoord_indices[0]], texcoords[texcoord_indices[1]], texcoords[texcoord_indices[2]], texcoords[texcoord_indices[3]]});
            }

            index_offset += num_face_vertices;
        }

        return true;
    }
};
