#pragma once

#include <tiny_obj_loader.h>

#include <array>
#include <format>
#include <memory>
#include <optional>
#include <sstream>
#include <vector>

#include "core/MathUtils.h"
#include "core/Pacific.h"
#include "core/Primitives.h"

class Geometry {
public:
    std::shared_ptr<AABB> bbox;

    Geometry() = default;
    virtual ~Geometry() = default;

    enum class Type { Triangle,
                      Quad,
                      Sphere };

    virtual Type get_type() = 0;
    virtual bool intersect(const Ray &ray, Intersection &isc) = 0;
    virtual void build_bbox() = 0;
    virtual Vec3f get_normal(const Vec3f &position) = 0;
    virtual std::string to_string() = 0;
};

class Triangle : public Geometry {
public:
    std::array<std::shared_ptr<Vec3f>, 3> positions;
    std::optional<std::array<std::shared_ptr<Vec3f>, 3>> normals;
    std::optional<std::array<std::shared_ptr<Vec2f>, 3>> tex_coords;

    Triangle() = default;
    Triangle(const std::shared_ptr<Vec3f> &a, const std::shared_ptr<Vec3f> &b, const std::shared_ptr<Vec3f> &c) {
        positions = std::array<std::shared_ptr<Vec3f>, 3>{a, b, c};
    }
    Triangle(const std::shared_ptr<Vec3f> &a, const std::shared_ptr<Vec3f> &b, const std::shared_ptr<Vec3f> &c, const std::shared_ptr<Vec3f> &an, const std::shared_ptr<Vec3f> &bn, const std::shared_ptr<Vec3f> &cn) {
        positions = std::array<std::shared_ptr<Vec3f>, 3>{a, b, c};
        normals = std::array<std::shared_ptr<Vec3f>, 3>{an, bn, cn};
    }
    Triangle(const std::shared_ptr<Vec3f> &a, const std::shared_ptr<Vec3f> &b, const std::shared_ptr<Vec3f> &c, const std::shared_ptr<Vec2f> &at, const std::shared_ptr<Vec2f> &bt, const std::shared_ptr<Vec2f> &ct) {
        positions = std::array<std::shared_ptr<Vec3f>, 3>{a, b, c};
        tex_coords = std::array<std::shared_ptr<Vec2f>, 3>{at, bt, ct};
    }
    Triangle(const std::shared_ptr<Vec3f> &a, const std::shared_ptr<Vec3f> &b, const std::shared_ptr<Vec3f> &c, const std::shared_ptr<Vec3f> &an, const std::shared_ptr<Vec3f> &bn, const std::shared_ptr<Vec3f> &cn, const std::shared_ptr<Vec2f> &at, const std::shared_ptr<Vec2f> &bt, const std::shared_ptr<Vec2f> &ct) {
        positions = std::array<std::shared_ptr<Vec3f>, 3>{a, b, c};
        normals = std::array<std::shared_ptr<Vec3f>, 3>{an, bn, cn};
        tex_coords = std::array<std::shared_ptr<Vec2f>, 3>{at, bt, ct};
    }

    Geometry::Type get_type() override {
        return Geometry::Type::Triangle;
    }

    bool intersect(const Ray &ray, Intersection &isc) override {
        throw std::runtime_error("Triangle intersection not implemented");
    }

    void build_bbox() override {
        throw std::runtime_error("Triangle build_bbox not implemented");
    }

    Vec3f get_normal(const Vec3f &position) override {
        // based on face_normals, either interpolate vn's or compute the
        // (constant) normal manually
        throw std::runtime_error("Triangle get_normal not implemented");
    }

    std::string to_string() override {
        std::ostringstream oss;
        oss << "Triangle: [\n";
        oss << "  vertex positions: " << std::format("[{}, {}, {}] - [{}, {}, {}], [{}, {}, {}]\n", positions[0]->x, positions[0]->y, positions[0]->z, positions[1]->x, positions[1]->y, positions[1]->z, positions[2]->x, positions[2]->y, positions[2]->z);
        if (normals.has_value())
            oss << "  vertex normals: " << std::format("[{}, {}, {}] - [{}, {}, {}], [{}, {}, {}]\n", normals.value()[0]->x, normals.value()[0]->y, normals.value()[0]->z, normals.value()[1]->x, normals.value()[1]->y, normals.value()[1]->z, normals.value()[2]->x, normals.value()[2]->y, normals.value()[2]->z);
        if (tex_coords.has_value())
            oss << "  vertex texcoords: " << std::format("[{}, {}] - [{}, {}], [{}, {}]\n", tex_coords.value()[0]->x, tex_coords.value()[0]->y, tex_coords.value()[1]->x, tex_coords.value()[1]->y, tex_coords.value()[2]->x, tex_coords.value()[2]->y);
        oss << "]";
        return oss.str();
    }
};

class Quad : public Geometry {
public:
    std::array<std::shared_ptr<Vec3f>, 4> positions;
    std::optional<std::array<std::shared_ptr<Vec3f>, 4>> normals;
    std::optional<std::array<std::shared_ptr<Vec2f>, 4>> tex_coords;

    Quad() = default;
    Quad(const std::shared_ptr<Vec3f> &a, const std::shared_ptr<Vec3f> &b, const std::shared_ptr<Vec3f> &c, const std::shared_ptr<Vec3f> &d) {
        positions = std::array<std::shared_ptr<Vec3f>, 4>{a, b, c, d};
    }
    Quad(const std::shared_ptr<Vec3f> &a, const std::shared_ptr<Vec3f> &b, const std::shared_ptr<Vec3f> &c, const std::shared_ptr<Vec3f> &d, const std::shared_ptr<Vec3f> &an, const std::shared_ptr<Vec3f> &bn, const std::shared_ptr<Vec3f> &cn, const std::shared_ptr<Vec3f> &dn) {
        positions = std::array<std::shared_ptr<Vec3f>, 4>{a, b, c, d};
        normals = std::array<std::shared_ptr<Vec3f>, 4>{an, bn, cn, dn};
    }
    Quad(const std::shared_ptr<Vec3f> &a, const std::shared_ptr<Vec3f> &b, const std::shared_ptr<Vec3f> &c, const std::shared_ptr<Vec3f> &d, const std::shared_ptr<Vec2f> &at, const std::shared_ptr<Vec2f> &bt, const std::shared_ptr<Vec2f> &ct, const std::shared_ptr<Vec2f> &dt) {
        positions = std::array<std::shared_ptr<Vec3f>, 4>{a, b, c, d};
        tex_coords = std::array<std::shared_ptr<Vec2f>, 4>{at, bt, ct, dt};
    }
    Quad(const std::shared_ptr<Vec3f> &a, const std::shared_ptr<Vec3f> &b, const std::shared_ptr<Vec3f> &c, const std::shared_ptr<Vec3f> &d, const std::shared_ptr<Vec3f> &an, const std::shared_ptr<Vec3f> &bn, const std::shared_ptr<Vec3f> &cn, const std::shared_ptr<Vec3f> &dn, const std::shared_ptr<Vec2f> &at, const std::shared_ptr<Vec2f> &bt, const std::shared_ptr<Vec2f> &ct, const std::shared_ptr<Vec2f> &dt) {
        positions = std::array<std::shared_ptr<Vec3f>, 4>{a, b, c, d};
        normals = std::array<std::shared_ptr<Vec3f>, 4>{an, bn, cn, dn};
        tex_coords = std::array<std::shared_ptr<Vec2f>, 4>{at, bt, ct, dt};
    }

    Geometry::Type get_type() override {
        return Geometry::Type::Quad;
    }

    bool intersect(const Ray &ray, Intersection &isc) override {
        throw std::runtime_error("Quad intersection not implemented");
    }

    void build_bbox() override {
        throw std::runtime_error("Quad build_bbox not implemented");
    }

    Vec3f get_normal(const Vec3f &position) override {
        // based on face_normals, either interpolate vn's or compute the
        // (constant) normal manually
        throw std::runtime_error("Quad get_normal not implemented");
    }

    std::string to_string() override {
        throw std::runtime_error("Quad to_string not implemented!");
    }
};

class Sphere : public Geometry {
public:
    Vec3f center;
    Float radius;

    Sphere() = default;
    Sphere(const Vec3f &center, Float radius)
        : center(center), radius(radius) {}
    ~Sphere() = default;

    Geometry::Type get_type() override {
        return Geometry::Type::Sphere;
    }

    bool intersect(const Ray &ray, Intersection &isc) override {
        throw std::runtime_error("Sphere intersect not implemented");
    }

    void build_bbox() override {
        throw std::runtime_error("Sphere build_bbox not implemented");
    }

    Vec3f get_normal(const Vec3f &position) override {
        throw std::runtime_error("Sphere get_normal not implemented");
    }

    std::string to_string() override {
        throw std::runtime_error("Sphere to_string not implemented");
    }
};

class MeshLoader {
public:
    // load a triangle/quad mesh
    static bool load_mesh_from_file(const std::string &file_path, std::vector<std::unique_ptr<Geometry>> &output_mesh) {
        tinyobj::ObjReader obj_reader;
        if (!obj_reader.ParseFromFile(file_path)) {
            if (!obj_reader.Error().empty())
                std::cerr << "TinyObjReader error reading file: " << file_path << "; " << obj_reader.Error() << "\n";
            return false;
        }
        if (!obj_reader.Warning().empty())
            std::cerr << "TinyObjReader warning reading file: " << file_path << "; " << obj_reader.Warning() << "\n";

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

        output_mesh = std::vector<std::unique_ptr<Geometry>>{};

        // Store vertices, normals and texcoords;
        size_t num_vertices = attrib.vertices.size() / 3;
        std::vector<std::shared_ptr<Vec3f>> vertices;
        for (size_t i = 0; i < num_vertices * 3; i += 3)
            vertices.emplace_back(std::make_shared<Vec3f>(attrib.vertices[i], attrib.vertices[i + 1], attrib.vertices[i + 2]));
        size_t num_normals = attrib.normals.size() / 3;
        std::vector<std::shared_ptr<Vec3f>> normals;
        for (size_t i = 0; i < num_normals * 3; i += 3)
            normals.emplace_back(std::make_shared<Vec3f>(attrib.normals[i], attrib.normals[i + 1], attrib.normals[i + 2]));
        size_t num_texcoords = attrib.texcoords.size() / 2;
        std::vector<std::shared_ptr<Vec2f>> texcoords;
        for (size_t i = 0; i < num_texcoords * 2; i += 2)
            texcoords.emplace_back(std::make_shared<Vec2f>(attrib.texcoords[i], attrib.texcoords[i + 1]));

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
                    output_mesh.push_back(std::make_unique<Triangle>(vertices[vertex_indices[0]], vertices[vertex_indices[1]], vertices[vertex_indices[2]]));
                else if (normal_indices[0] == -1)
                    output_mesh.push_back(std::make_unique<Triangle>(vertices[vertex_indices[0]], vertices[vertex_indices[1]], vertices[vertex_indices[2]], texcoords[texcoord_indices[0]], texcoords[texcoord_indices[1]], texcoords[texcoord_indices[2]]));
                else if (texcoord_indices[0] == -1)
                    output_mesh.push_back(std::make_unique<Triangle>(vertices[vertex_indices[0]], vertices[vertex_indices[1]], vertices[vertex_indices[2]], normals[normal_indices[0]], normals[normal_indices[1]], normals[normal_indices[2]]));
                else
                    output_mesh.push_back(std::make_unique<Triangle>(vertices[vertex_indices[0]], vertices[vertex_indices[1]], vertices[vertex_indices[2]], normals[normal_indices[0]], normals[normal_indices[1]], normals[normal_indices[2]], texcoords[texcoord_indices[0]], texcoords[texcoord_indices[1]], texcoords[texcoord_indices[2]]));

            } else if (num_face_vertices == 4) {
                // Quad
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
                    output_mesh.push_back(std::make_unique<Quad>(vertices[vertex_indices[0]], vertices[vertex_indices[1]], vertices[vertex_indices[2]], vertices[vertex_indices[3]]));
                else if (normal_indices[0] == -1)
                    output_mesh.push_back(std::make_unique<Quad>(vertices[vertex_indices[0]], vertices[vertex_indices[1]], vertices[vertex_indices[2]], vertices[vertex_indices[3]], texcoords[texcoord_indices[0]], texcoords[texcoord_indices[1]], texcoords[texcoord_indices[2]], texcoords[texcoord_indices[3]]));
                else if (texcoord_indices[0] == -1)
                    output_mesh.push_back(std::make_unique<Quad>(vertices[vertex_indices[0]], vertices[vertex_indices[1]], vertices[vertex_indices[2]], vertices[vertex_indices[3]], normals[normal_indices[0]], normals[normal_indices[1]], normals[normal_indices[2]], normals[normal_indices[3]]));
                else
                    output_mesh.push_back(std::make_unique<Quad>(vertices[vertex_indices[0]], vertices[vertex_indices[1]], vertices[vertex_indices[2]], vertices[vertex_indices[3]], normals[normal_indices[0]], normals[normal_indices[1]], normals[normal_indices[2]], normals[normal_indices[3]], texcoords[texcoord_indices[0]], texcoords[texcoord_indices[1]], texcoords[texcoord_indices[2]], texcoords[texcoord_indices[3]]));
            }

            index_offset += num_face_vertices;
        }

        return true;
    }
};
