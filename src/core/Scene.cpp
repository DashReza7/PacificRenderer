#include "core/Scene.h"

#include "core/Registry.h"
#include "happly.h"
#include "utils/FileUtils.h"
#include "utils/Logger.h"
#include "utils/SceneParser.h"

// BUG: memory leak for shapes, geometries, bsdfs, emitters, sensor

std::unordered_map<const TextureDesc*, Texture*> load_textures(const std::vector<const TextureDesc*>& texs_desc) {
    std::unordered_map<const TextureDesc*, Texture*> textures{};
    for (const auto& tex_desc : texs_desc)
        textures[tex_desc] = TextureRegistry::createTexture(tex_desc->type, tex_desc->properties);
    return textures;
}

std::unordered_map<BSDFDesc*, BSDF*> load_bsdfs(const std::vector<BSDFDesc*>& bsdfs_desc, const std::unordered_map<const TextureDesc*, Texture*>& texs_desc) {
    std::unordered_map<BSDFDesc*, BSDF*> bsdfs{};
    for (const auto& bsdf_desc : bsdfs_desc) {
        std::unordered_map<std::string, const Texture*> textures{};
        for (const auto& [name, tex_desc] : bsdf_desc->textures)
            textures[name] = texs_desc.at(tex_desc);

        BSDF* bsdf = BSDFRegistry::createBSDF(bsdf_desc->type, bsdf_desc->properties, textures);
        bsdfs[bsdf_desc] = bsdf;
    }

    return bsdfs;
}

std::unordered_map<EmitterDesc*, Emitter*> load_emitters(const std::vector<EmitterDesc*>& emitters_desc, const std::unordered_map<const TextureDesc*, Texture*>& texs_desc, std::vector<Emitter*>& emitters) {
    std::unordered_map<EmitterDesc*, Emitter*> emitters_dict{};
    for (const auto& emitter_desc : emitters_desc) {
        std::unordered_map<std::string, const Texture*> textures{};
        for (const auto& [name, tex_desc] : emitter_desc->textures)
            textures[name] = texs_desc.at(tex_desc);

        Emitter* emitter = EmitterRegistry::createEmitter(emitter_desc->type, emitter_desc->properties, textures);
        emitters.push_back(emitter);
        emitters_dict[emitter_desc] = emitter;
    }

    return emitters_dict;
}

bool load_mesh_from_file(const std::string& file_path, const Shape* parent_shape, std::vector<Geometry*>& output_mesh, std::vector<Vec3f*>& vertices, std::vector<Vec3f*>& normals, std::vector<Vec2f*>& texcoords, const std::unordered_map<std::string, std::string>& properties) {
    tinyobj::ObjReader obj_reader;
    if (!obj_reader.ParseFromFile(file_path)) {
        if (!obj_reader.Error().empty())
            std::cerr << "TinyObjReader error reading file: " << file_path << "; " << obj_reader.Error() << "\n";
        return false;
    }
    if (!obj_reader.Warning().empty()) {
        std::cout << "TinyObjReader warning reading file: " << file_path << "; " << obj_reader.Warning() << "\n";
    }

    // Only support objs with one shape
    const std::vector<tinyobj::shape_t>& shapes = obj_reader.GetShapes();
    if (shapes.size() == 0) {
        std::cerr << "TinyObjReader warning reading file: " << file_path << "; " << "No shapes in this file\n";
        return false;
    }
    if (shapes.size() > 1) {
        std::cerr << "TinyObjReader warning reading file: " << file_path << "; " << "More than one shape in .obj file not supported\n";
        return false;
    }

    const tinyobj::attrib_t& attrib = obj_reader.GetAttrib();

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
            output_mesh.emplace_back(GeometryRegistry::createGeometry("triangle", properties, parent_shape, &gctx));
        } else {
            // Other polygons
            throw std::runtime_error("Quad not implemented");
        }

        index_offset += num_face_vertices;
    }

    return true;
}

void load_obj(const ShapeDesc* shape_desc, Shape* shape) {
    std::vector<Vec3f*> vertices;
    std::vector<Vec3f*> normals;
    std::vector<Vec2f*> texcoords;
    std::string filepath = (scene_file_path.parent_path() / shape_desc->properties.at("filename")).string();

    tinyobj::ObjReader obj_reader;
    if (!obj_reader.ParseFromFile(filepath) && !obj_reader.Error().empty())
        throw std::runtime_error("TinyObjReader error reading file: " + filepath + "; " + obj_reader.Error());
    if (!obj_reader.Warning().empty())
        throw std::runtime_error("TinyObjReader warning reading file: " + filepath + "; " + obj_reader.Warning());

    // Only support objs with one shape
    const std::vector<tinyobj::shape_t>& shapes = obj_reader.GetShapes();
    if (shapes.size() == 0)
        throw std::runtime_error("TinyObjReader warning reading file: " + filepath + "; " + "No shapes in this file");
    if (shapes.size() > 1)
        throw std::runtime_error("TinyObjReader warning reading file: " + filepath + "; " + "More than one shape in .obj file not supported");

    const tinyobj::attrib_t& attrib = obj_reader.GetAttrib();

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
            shape->geometries.emplace_back(GeometryRegistry::createGeometry("triangle", shape_desc->properties, shape, &gctx));
        } else {
            // Other polygons
            throw std::runtime_error("Quad not implemented");
        }

        index_offset += num_face_vertices;
    }

    // apply transform
    Mat4f to_world = strToMat4f(shape_desc->properties.at("to_world"));
    Mat4f tsp_inv_to_world = glm::transpose(strToMat4f(shape_desc->properties.at("inv_to_world")));
    for (auto& vertex : vertices)
        *vertex = Vec3f{to_world * Vec4f{*vertex, 1.0}};
    // use the inverse transpose of the upper-left 3x3 part of the matrix
    for (auto& normal : normals)
        *normal = glm::normalize(Vec3f{tsp_inv_to_world * Vec4f{*normal, 0.0}});
}

void load_ply(const ShapeDesc* shape_desc, Shape* shape) {
    happly::PLYData mesh{(scene_file_path.parent_path() / shape_desc->properties.at("filename")).string(), false};
    auto element_names = mesh.getElementNames();
    if (std::find(element_names.begin(), element_names.end(), "vertex") == element_names.end())
        throw std::runtime_error("PLY mesh must have vertex elements");
    if (std::find(element_names.begin(), element_names.end(), "face") == element_names.end())
        throw std::runtime_error("PLY mesh must have face elements");
    auto& vertex_element = mesh.getElement("vertex");
    if (!mesh.getElement("vertex").hasProperty("x") || !mesh.getElement("vertex").hasProperty("y") || !mesh.getElement("vertex").hasProperty("z"))
        throw std::runtime_error("PLY mesh vertex elements must have x, y, z properties");
    auto vp_x = vertex_element.getProperty<Float>("x");
    auto vp_y = vertex_element.getProperty<Float>("y");
    auto vp_z = vertex_element.getProperty<Float>("z");
    auto vn_x = vertex_element.hasProperty("nx") ? vertex_element.getProperty<Float>("nx") : std::vector<Float>{0};
    auto vn_y = vertex_element.hasProperty("ny") ? vertex_element.getProperty<Float>("ny") : std::vector<Float>{0};
    auto vn_z = vertex_element.hasProperty("nz") ? vertex_element.getProperty<Float>("nz") : std::vector<Float>{0};
    auto vt_u = vertex_element.hasProperty("u") ? vertex_element.getProperty<Float>("u") : vertex_element.hasProperty("s") ? vertex_element.getProperty<Float>("s")
                                                                                                                           : std::vector<Float>{0};
    auto vt_v = vertex_element.hasProperty("v") ? vertex_element.getProperty<Float>("v") : vertex_element.hasProperty("t") ? vertex_element.getProperty<Float>("t")
                                                                                                                           : std::vector<Float>{0};
    std::vector<Vec3f*> vertices{};
    std::vector<Vec3f*> normals{};
    std::vector<Vec2f*> tex_coords{};

    for (size_t i = 0; i < vp_x.size(); ++i)
        vertices.push_back(new Vec3f{vp_x[i], vp_y[i], vp_z[i]});
    for (size_t i = 0; i < vn_x.size(); ++i)
        if (vn_x.size() != 0)
            normals.push_back(new Vec3f{vn_x[i], vn_y[i], vn_z[i]});
    for (size_t i = 0; i < vt_u.size(); ++i)
        if (vt_u.size() != 0)
            tex_coords.push_back(new Vec2f{vt_u[i], vt_v[i]});
    // apply transform
    Mat4f to_world = strToMat4f(shape_desc->properties.at("to_world"));
    Mat4f tsp_inv_to_world = glm::transpose(strToMat4f(shape_desc->properties.at("inv_to_world")));
    for (auto& vertex : vertices)
        *vertex = Vec3f{to_world * Vec4f{*vertex, 1.0}};
    for (auto& normal : normals)
        *normal = glm::normalize(Vec3f{tsp_inv_to_world * Vec4f{*normal, 0.0}});
    // process faces
    auto& face_element = mesh.getElement("face");
    if (!face_element.hasProperty("vertex_indices"))
        throw std::runtime_error("PLY mesh face elements must have vertex_indices property");
    auto face_vertex_indices = face_element.getListProperty<int>("vertex_indices");
    for (const auto& face : face_vertex_indices) {
        if (face.size() == 3) {
            GeometryCreationContext gctx{};
            gctx.vp = {vertices[face[0]], vertices[face[1]], vertices[face[2]]};
            if (normals.size() == vertices.size())
                gctx.vn = {normals[face[0]], normals[face[1]], normals[face[2]]};
            if (tex_coords.size() == vertices.size())
                gctx.vt = {tex_coords[face[0]], tex_coords[face[1]], tex_coords[face[2]]};
            shape->geometries.push_back(GeometryRegistry::createGeometry("triangle", shape_desc->properties, shape, new GeometryCreationContext{gctx}));
        } else if (face.size() == 4) {
            // quad -> 2 triangles
            GeometryCreationContext gctx1{};
            gctx1.vp = {vertices[face[0]], vertices[face[1]], vertices[face[2]]};
            GeometryCreationContext gctx2{};
            gctx2.vp = {vertices[face[0]], vertices[face[2]], vertices[face[3]]};
            if (normals.size() == vertices.size()) {
                gctx1.vn = {normals[face[0]], normals[face[1]], normals[face[2]]};
                gctx2.vn = {normals[face[0]], normals[face[2]], normals[face[3]]};
            }
            if (tex_coords.size() == vertices.size()) {
                gctx1.vt = {tex_coords[face[0]], tex_coords[face[1]], tex_coords[face[2]]};
                gctx2.vt = {tex_coords[face[0]], tex_coords[face[2]], tex_coords[face[3]]};
            }
            shape->geometries.push_back(GeometryRegistry::createGeometry("triangle", shape_desc->properties, shape, &gctx1));
            shape->geometries.push_back(GeometryRegistry::createGeometry("triangle", shape_desc->properties, shape, &gctx2));
        } else {
            throw std::runtime_error("Only triangle and quad faces are supported in PLY mesh");
        }
    }
}

void load_serialized(const ShapeDesc* shape_desc, Shape* shape) {
    int shape_index = 0;
    if (shape_desc->properties.find("shape_index") != shape_desc->properties.end())
        shape_index = std::stoi(shape_desc->properties.find("shape_index")->second);
    if (shape_index < 0)
        throw std::runtime_error("shape_index must be non-negative");

    std::string filepath = (scene_file_path.parent_path() / shape_desc->properties.at("filename")).string();
    std::ifstream file(filepath, std::ios::binary);
    if (!file)
        throw std::runtime_error("Failed to open file");

    uint16_t id = read_u16_le(file);
    uint16_t version = read_u16_le(file);

    // read the last 4 bytes as a uint32_t
    file.seekg(-4, std::ios::end);
    uint32_t n_shapes = read_u32_le(file);
    if (shape_index >= n_shapes)
        throw std::runtime_error("shape_index out of range");
    file.seekg(0, std::ios::end);
    std::streamoff file_size = file.tellg();
    std::vector<uint64_t> offsets(n_shapes);
    uint64_t last_offset;
    if (version == 4) {
        last_offset = file_size - (4 + n_shapes * 8);
        for (size_t i = 0; i < n_shapes; i++) {
            file.seekg(-4 - (n_shapes - i) * 8, std::ios::end);
            offsets[i] = read_u64_le(file);
        }
    } else if (version == 3) {
        last_offset = file_size - 4 * (n_shapes + 1);
        for (size_t i = 0; i < n_shapes; i++) {
            file.seekg(-(n_shapes - i + 1) * 4, std::ios::end);
            offsets[i] = read_u32_le(file);
        }
    } else {
        throw std::runtime_error("unsupported version");
    }

    file.seekg(offsets[shape_index] + 4, std::ios::beg);
    std::vector<uint8_t> compressed((shape_index == n_shapes - 1 ? last_offset : offsets[shape_index + 1]) - offsets[shape_index] - 4);
    file.read(reinterpret_cast<char*>(compressed.data()), compressed.size());
    uLongf decompressed_size = compressed.size() * 4;  // rough guess; will expand if needed
    std::vector<uint8_t> decompressed(decompressed_size);
    int status = uncompress(decompressed.data(), &decompressed_size, compressed.data(), compressed.size());
    if (status == Z_BUF_ERROR) {
        // Try again with larger buffer. TODO: implement a more robust way
        decompressed_size = compressed.size() * 10;
        decompressed.resize(decompressed_size);
        status = uncompress(decompressed.data(), &decompressed_size, compressed.data(), compressed.size());
    }
    if (status != Z_OK)
        throw std::runtime_error("Decompression failed: " + std::to_string(status));
    decompressed.resize(decompressed_size);

    // read contents
    std::vector<Vec3f*> vertices{};
    std::vector<Vec3f*> normals{};
    std::vector<Vec2f*> texcoords{};
    size_t offset = 0;
    uint32_t flags = read_u32_le_buffer(decompressed.data(), offset);
    std::string name;
    if (version == 4)
        name = read_utf8_string(decompressed.data(), offset);
    uint64_t num_vertices = read_u64_le_buffer(decompressed.data(), offset);
    uint64_t num_triangles = read_u64_le_buffer(decompressed.data(), offset);
    bool single_precision = (flags & 0x1000) != 0;
    bool double_precision = (flags & 0x2000) != 0;
    if (single_precision && double_precision)
        throw std::runtime_error("Both single and double precision flags are set");
    if (!single_precision && !double_precision)
        throw std::runtime_error("Neither single nor double precision flags are set");
    bool has_normals = (flags & 0x1) != 0;
    bool has_texcoords = (flags & 0x2) != 0;
    bool has_colors = (flags & 0x8) != 0;
    if (has_colors)  // TODO
        throw std::runtime_error("Serialized mesh with vertex colors not supported");

    if (double_precision) {
        for (uint64_t i = 0; i < num_vertices; ++i) {
            Float x = read_double_le_buffer(decompressed.data(), offset);
            Float y = read_double_le_buffer(decompressed.data(), offset);
            Float z = read_double_le_buffer(decompressed.data(), offset);
            vertices.push_back(new Vec3f{x, y, z});
        }
        if (has_normals) {
            for (uint64_t i = 0; i < num_vertices; ++i) {
                Float x = read_double_le_buffer(decompressed.data(), offset);
                Float y = read_double_le_buffer(decompressed.data(), offset);
                Float z = read_double_le_buffer(decompressed.data(), offset);
                normals.push_back(new Vec3f{x, y, z});
            }
        }
        if (has_texcoords) {
            for (uint64_t i = 0; i < num_vertices; ++i) {
                Float u = read_double_le_buffer(decompressed.data(), offset);
                Float v = read_double_le_buffer(decompressed.data(), offset);
                texcoords.push_back(new Vec2f{u, v});
            }
        }
        // TODO: colors
    } else {
        for (uint32_t i = 0; i < num_vertices; ++i) {
            Float x = read_float_le_buffer(decompressed.data(), offset);
            Float y = read_float_le_buffer(decompressed.data(), offset);
            Float z = read_float_le_buffer(decompressed.data(), offset);
            vertices.push_back(new Vec3f{x, y, z});
        }
        if (has_normals) {
            for (uint32_t i = 0; i < num_vertices; ++i) {
                Float x = read_float_le_buffer(decompressed.data(), offset);
                Float y = read_float_le_buffer(decompressed.data(), offset);
                Float z = read_float_le_buffer(decompressed.data(), offset);
                normals.push_back(new Vec3f{x, y, z});
            }
        }
        if (has_texcoords) {
            for (uint32_t i = 0; i < num_vertices; ++i) {
                Float u = read_float_le_buffer(decompressed.data(), offset);
                Float v = read_float_le_buffer(decompressed.data(), offset);
                texcoords.push_back(new Vec2f{u, v});
            }
        }
        // TODO: colors
    }

    // apply transform
    Mat4f to_world = strToMat4f(shape_desc->properties.at("to_world"));
    Mat4f tsp_inv_to_world = glm::transpose(strToMat4f(shape_desc->properties.at("inv_to_world")));
    for (auto& vertex : vertices)
        *vertex = Vec3f{to_world * Vec4f{*vertex, 1.0}};
    for (auto& normal : normals)
        *normal = glm::normalize(Vec3f{tsp_inv_to_world * Vec4f{*normal, 0.0}});
    // read face indices
    if (num_vertices > 0xFFFFFFFF) {
        // use uint64_t for indices
        for (uint32_t i = 0; i < num_triangles; ++i) {
            uint64_t idx0 = read_u64_le_buffer(decompressed.data(), offset);
            uint64_t idx1 = read_u64_le_buffer(decompressed.data(), offset);
            uint64_t idx2 = read_u64_le_buffer(decompressed.data(), offset);
            GeometryCreationContext gctx{};
            gctx.vp = {vertices[idx0], vertices[idx1], vertices[idx2]};
            if (has_normals)
                gctx.vn = {normals[idx0], normals[idx1], normals[idx2]};
            if (has_texcoords)
                gctx.vt = {texcoords[idx0], texcoords[idx1], texcoords[idx2]};
            shape->geometries.push_back(GeometryRegistry::createGeometry("triangle", shape_desc->properties, shape, &gctx));
        }
    } else {
        // use uint32_t for indices
        for (uint32_t i = 0; i < num_triangles; ++i) {
            uint32_t idx0 = read_u32_le_buffer(decompressed.data(), offset);
            uint32_t idx1 = read_u32_le_buffer(decompressed.data(), offset);
            uint32_t idx2 = read_u32_le_buffer(decompressed.data(), offset);
            GeometryCreationContext gctx{};
            gctx.vp = {vertices[idx0], vertices[idx1], vertices[idx2]};
            if (has_normals)
                gctx.vn = {normals[idx0], normals[idx1], normals[idx2]};
            if (has_texcoords)
                gctx.vt = {texcoords[idx0], texcoords[idx1], texcoords[idx2]};
            shape->geometries.push_back(GeometryRegistry::createGeometry("triangle", shape_desc->properties, shape, &gctx));
        }
    }
}

void load_shapes(const std::vector<ShapeDesc*> shapes_desc, const std::unordered_map<BSDFDesc*, BSDF*>& bsdfs_dict, const std::unordered_map<EmitterDesc*, Emitter*>& emitters_dict, std::vector<Shape*>& shapes) {
    for (const auto& shape_desc : shapes_desc) {
        auto shape = new Shape{};
        if (shape_desc->bsdf == nullptr)
            throw std::runtime_error("Shape missing BSDF");
        shape->bsdf = bsdfs_dict.at(shape_desc->bsdf);

        if (shape_desc->type == "serialized") {
            shape->type = Shape::Type::Mesh;
            load_serialized(shape_desc, shape);
        } else if (shape_desc->type == "ply") {
            shape->type = Shape::Type::Mesh;
            load_ply(shape_desc, shape);
        } else if (shape_desc->type == "obj") {
            shape->type = Shape::Type::Mesh;
            load_obj(shape_desc, shape);
        } else if (shape_desc->type == "sphere") {
            shape->type = Shape::Type::Sphere;
            shape->geometries.push_back(GeometryRegistry::createGeometry("sphere", shape_desc->properties, shape, nullptr));
        } else if (shape_desc->type == "disk") {
            shape->type = Shape::Type::Disk;
            shape->geometries.push_back(GeometryRegistry::createGeometry("disk", shape_desc->properties, shape, nullptr));
        } else if (shape_desc->type == "cube") {
            // BUG: texture coordinates are not correct
            shape->type = Shape::Type::Mesh;
            auto properties = shape_desc->properties;
            properties["face_normals"] = "true";

            std::vector<Vec3f*> vertices;
            vertices.push_back(new Vec3f{-1.0, -1.0, -1.0});
            vertices.push_back(new Vec3f{-1.0, -1.0, 1.0});
            vertices.push_back(new Vec3f{-1.0, 1.0, -1.0});
            vertices.push_back(new Vec3f{-1.0, 1.0, 1.0});
            vertices.push_back(new Vec3f{1.0, -1.0, -1.0});
            vertices.push_back(new Vec3f{1.0, -1.0, 1.0});
            vertices.push_back(new Vec3f{1.0, 1.0, -1.0});
            vertices.push_back(new Vec3f{1.0, 1.0, 1.0});
            auto it = shape_desc->properties.find("to_world");
            if (it != shape_desc->properties.end()) {
                auto to_world = strToMat4f(it->second);
                for (auto& vertex : vertices)
                    *vertex = Vec3f{to_world * Vec4f{*vertex, 1.0}};
            }
            GeometryCreationContext gctx01{vertices[0], vertices[1], vertices[2]};
            GeometryCreationContext gctx02{vertices[1], vertices[3], vertices[2]};
            GeometryCreationContext gctx03{vertices[4], vertices[6], vertices[5]};
            GeometryCreationContext gctx04{vertices[5], vertices[6], vertices[7]};
            GeometryCreationContext gctx05{vertices[0], vertices[4], vertices[1]};
            GeometryCreationContext gctx06{vertices[1], vertices[4], vertices[5]};
            GeometryCreationContext gctx07{vertices[2], vertices[3], vertices[6]};
            GeometryCreationContext gctx08{vertices[3], vertices[7], vertices[6]};
            GeometryCreationContext gctx09{vertices[0], vertices[2], vertices[4]};
            GeometryCreationContext gctx10{vertices[2], vertices[6], vertices[4]};
            GeometryCreationContext gctx11{vertices[1], vertices[5], vertices[3]};
            GeometryCreationContext gctx12{vertices[3], vertices[5], vertices[7]};
            shape->geometries.push_back(GeometryRegistry::createGeometry("triangle", properties, shape, &gctx01));
            shape->geometries.push_back(GeometryRegistry::createGeometry("triangle", properties, shape, &gctx02));
            shape->geometries.push_back(GeometryRegistry::createGeometry("triangle", properties, shape, &gctx03));
            shape->geometries.push_back(GeometryRegistry::createGeometry("triangle", properties, shape, &gctx04));
            shape->geometries.push_back(GeometryRegistry::createGeometry("triangle", properties, shape, &gctx05));
            shape->geometries.push_back(GeometryRegistry::createGeometry("triangle", properties, shape, &gctx06));
            shape->geometries.push_back(GeometryRegistry::createGeometry("triangle", properties, shape, &gctx07));
            shape->geometries.push_back(GeometryRegistry::createGeometry("triangle", properties, shape, &gctx08));
            shape->geometries.push_back(GeometryRegistry::createGeometry("triangle", properties, shape, &gctx09));
            shape->geometries.push_back(GeometryRegistry::createGeometry("triangle", properties, shape, &gctx10));
            shape->geometries.push_back(GeometryRegistry::createGeometry("triangle", properties, shape, &gctx11));
            shape->geometries.push_back(GeometryRegistry::createGeometry("triangle", properties, shape, &gctx12));
        } else if (shape_desc->type == "rectangle") {
            shape->type = Shape::Type::Mesh;
            auto properties = shape_desc->properties;
            properties["face_normals"] = "true";

            std::vector<Vec3f*> vertices{};
            vertices.push_back(new Vec3f{-1.0, -1.0, 0.0});
            vertices.push_back(new Vec3f{-1.0, 1.0, 0.0});
            vertices.push_back(new Vec3f{1.0, -1.0, 0.0});
            vertices.push_back(new Vec3f{1.0, 1.0, 0.0});
            auto it = shape_desc->properties.find("to_world");
            if (it != shape_desc->properties.end()) {
                auto to_world = strToMat4f(it->second);
                for (auto& vertex : vertices)
                    *vertex = Vec3f{to_world * Vec4f{*vertex, 1.0}};
            }
            std::vector<Vec2f*> tex_coords{};
            tex_coords.push_back(new Vec2f{0.0, 0.0});
            tex_coords.push_back(new Vec2f{0.0, 1.0});
            tex_coords.push_back(new Vec2f{1.0, 0.0});
            tex_coords.push_back(new Vec2f{1.0, 1.0});

            GeometryCreationContext gctx1{vertices[0], vertices[2], vertices[1], nullptr, nullptr, nullptr, tex_coords[0], tex_coords[2], tex_coords[1]};
            GeometryCreationContext gctx2{vertices[3], vertices[1], vertices[2], nullptr, nullptr, nullptr, tex_coords[3], tex_coords[1], tex_coords[2]};
            shape->geometries.push_back(GeometryRegistry::createGeometry("triangle", properties, shape, &gctx1));
            shape->geometries.push_back(GeometryRegistry::createGeometry("triangle", properties, shape, &gctx2));
        } else {
            throw std::runtime_error("Unsupported shape type: " + shape_desc->type);
        }

        // if the shape has an emitter(it's an area_light), link them
        if (shape_desc->emitter != nullptr) {
            shape->emitter = emitters_dict.at(shape_desc->emitter);
            shape->emitter->set_shape(shape);
        }

        shapes.push_back(shape);
    }
}

void load_sensor(const SensorDesc* sensor_desc, Sensor*& sensor) {
    Float fov = 45.0;
    Float near_clip = 1e-2, far_clip = 1e4;
    uint32_t width = 800, height = 600;
    uint32_t spp = 4;
    RFilter* rfilter;
    uint32_t seed = 0;

    if (sensor_desc->properties.find("fov") != sensor_desc->properties.end())
        fov = static_cast<Float>(std::stod(sensor_desc->properties.at("fov")));
    if (sensor_desc->properties.find("near_clip") != sensor_desc->properties.end())
        near_clip = static_cast<Float>(std::stod(sensor_desc->properties.at("near_clip")));
    if (sensor_desc->properties.find("far_clip") != sensor_desc->properties.end())
        far_clip = static_cast<Float>(std::stod(sensor_desc->properties.at("far_clip")));
    // parse Film
    if (sensor_desc->film->properties.find("width") != sensor_desc->film->properties.end())
        width = static_cast<uint32_t>(std::stoi(sensor_desc->film->properties.at("width")));
    if (sensor_desc->film->properties.find("height") != sensor_desc->film->properties.end())
        height = static_cast<uint32_t>(std::stoi(sensor_desc->film->properties.at("height")));
    // parse Film's RFilter
    rfilter = RFilterRegistry::createRFilter(sensor_desc->film->rfilter->type, sensor_desc->film->rfilter->properties);
    // parse Sampler
    if (sensor_desc->sampler->properties.find("sample_count") != sensor_desc->sampler->properties.end())
        spp = static_cast<uint32_t>(std::stoi(sensor_desc->sampler->properties.at("sample_count")));
    if (sensor_desc->sampler->properties.find("seed") != sensor_desc->sampler->properties.end())
        seed = static_cast<uint32_t>(std::stoi(sensor_desc->sampler->properties.at("seed")));

    sensor = new Sensor{sensor_desc->to_world, fov, seed, width, height, spp, near_clip, far_clip, rfilter};
}

void build_bvh(BVHNode* node, const std::vector<Geometry*>& contained_geoms) {
    if (contained_geoms.size() == 0)
        return;

    node->bbox = contained_geoms[0]->get_bbox();
    for (const auto& geom : contained_geoms)
        node->bbox = node->bbox + geom->get_bbox();
    // determine longest axis
    Float dx = node->bbox.max_corner.x - node->bbox.min_corner.x;
    Float dy = node->bbox.max_corner.y - node->bbox.min_corner.y;
    Float dz = node->bbox.max_corner.z - node->bbox.min_corner.z;
    int longest_axis = 0;
    if (dy > dx && dy > dz)
        longest_axis = 1;
    else if (dz > dx && dz > dy)
        longest_axis = 2;

    // if the longest axis doesn't split, use other axes.
    // if that also doesn't work, add all the geometries to the list and make this a leaf node
    for (int i = 0; i < 3; i++) {
        int axis = (longest_axis + i) % 3;
        Float threshold = (node->bbox.max_corner[axis] + node->bbox.min_corner[axis]) / 2.0;
        std::vector<Geometry*> left_geoms, right_geoms;
        for (const auto& geom : contained_geoms) {
            bool left = false;
            Float center = (geom->get_bbox().min_corner[axis] + geom->get_bbox().max_corner[axis]) / 2.0;
            if (center <= threshold)
                left = true;

            if (left)
                left_geoms.push_back(geom);
            else
                right_geoms.push_back(geom);
        }

        if (!left_geoms.empty() && !right_geoms.empty()) {
            // successfully splitted
            node->left = new BVHNode{};
            node->right = new BVHNode{};
            build_bvh(node->left, left_geoms);
            build_bvh(node->right, right_geoms);
            break;
        }

        if (i == 2) {
            // not able to split. make this a leaf node
            node->geoms.insert(node->geoms.end(), contained_geoms.begin(), contained_geoms.end());
        }
    }
}

void Scene::load_scene(const SceneDesc& scene_desc) {
    auto texs_dict = load_textures(scene_desc.textures);
    auto bsdfs_dict = load_bsdfs(scene_desc.bsdfs, texs_dict);
    auto emitters_dict = load_emitters(scene_desc.emitters, texs_dict, emitters);
    if (scene_desc.has_envmap) {
        for (const auto& [emitter_desc, emitter] : emitters_dict)
            if (emitter_desc->type == "envmap") {
                env_map = emitter;
                break;
            }
    }

    load_shapes(scene_desc.shapes, bsdfs_dict, emitters_dict, shapes);
    bvh_root = new BVHNode{};
    build_bvh(bvh_root, get_all_geoms());

    load_sensor(scene_desc.sensor, sensor);
}

std::vector<Geometry*> Scene::get_all_geoms() const {
    std::vector<Geometry*> all_geoms;
    for (const auto& shape : shapes)
        all_geoms.insert(all_geoms.end(), shape->geometries.begin(), shape->geometries.end());
    return all_geoms;
}

std::string Scene::get_bvh_str(BVHNode* node, int idt) const {
    if (node == nullptr)
        node = bvh_root;

    std::string indent(idt, ' ');
    std::string s;
    s += indent + "BVHNode: \n";
    s += indent + "  BBox: [(" + std::to_string(node->bbox.min_corner.x) + ", " + std::to_string(node->bbox.min_corner.y) + ", " + std::to_string(node->bbox.min_corner.z) + "), " + "(" + std::to_string(node->bbox.max_corner.x) + ", " + std::to_string(node->bbox.max_corner.y) + ", " + std::to_string(node->bbox.max_corner.z) + ")]\n";
    if (node->left == nullptr && node->right == nullptr) {
        s += indent + "  Leaf Node with " + std::to_string(node->geoms.size()) + " geometries:\n";
        for (const auto& geom : node->geoms) {
            s += indent + "    - " + geom->to_string() + "\n";
        }
    } else {
        if (node->left) {
            s += indent + "  Left Child:\n";
            s += get_bvh_str(node->left, idt + 4);
        }

        if (node->right) {
            s += indent + "  Right Child:\n";
            s += get_bvh_str(node->right, idt + 4);
        }
    }
    return s;
}

std::string Scene::get_bvh_statistics() const {
    // compute statistics like number of nodes, depth, average number of geometries per leaf node, etc.
    // also check if any non-leaf node has geometries with size > 0 (which is an error)

    std::ostringstream oss;
    if (!bvh_root) {
        oss << "BVH not built yet." << std::endl;
        return oss.str();
    }
    int num_nodes = 0;
    int num_leaf_nodes = 0;
    int max_depth = 0;
    int total_geoms_in_leaves = 0;
    int max_geoms_in_leaf = 0;
    std::function<void(BVHNode*, int)> traverse = [&](BVHNode* node, int depth) {
        if (!node)
            return;
        num_nodes++;
        if (!node->left && !node->right) {
            // leaf node
            num_leaf_nodes++;
            total_geoms_in_leaves += node->geoms.size();
            max_geoms_in_leaf = std::max(max_geoms_in_leaf, (int)node->geoms.size());
            if (depth > max_depth)
                max_depth = depth;
        } else {
            // non-leaf node
            if (!node->geoms.empty()) {
                throw std::runtime_error("Non-leaf node with geometries found.");
            }
        }
        traverse(node->left, depth + 1);
        traverse(node->right, depth + 1);
    };
    traverse(bvh_root, 1);
    oss << "BVH Statistics:" << std::endl;
    oss << "  Number of nodes: " << num_nodes << std::endl;
    oss << "  Number of leaf nodes: " << num_leaf_nodes << std::endl;
    oss << "  Max depth: " << max_depth << std::endl;
    if (num_leaf_nodes > 0)
        oss << "  Average geometries per leaf: " << (total_geoms_in_leaves / num_leaf_nodes) << std::endl;
    else
        oss << "  Average geometries per leaf: N/A" << std::endl;
    oss << "  Max geometries in leaf: " << max_geoms_in_leaf << std::endl;
    return oss.str();
}

bool Scene::ray_intersect_bruteforce(const Ray& ray, Intersection& isc) const {
    bool is_hit = false;
    Float best_dist = ray.tmax;
    for (const auto& shape : shapes)
        for (const auto& geom : shape->geometries) {
            Intersection temp_isc;
            if (geom->intersect(ray, temp_isc)) {
                if (ray.shadow_ray)
                    return true;
                if (temp_isc.distance < best_dist) {
                    best_dist = temp_isc.distance;
                    isc = temp_isc;
                    is_hit = true;
                }
            }
        }
    return is_hit;
}

bool Scene::ray_intersect_bvh(const Ray& ray, Intersection& isc) const {
    if (bvh_root == nullptr)
        throw std::runtime_error("BVH not built");
    return bvh_root->intersect(ray, isc);
}

bool Scene::ray_intersect(const Ray& ray, Intersection& isc) const {
    if (accel_type == AccelerationType::NONE)
        return ray_intersect_bruteforce(ray, isc);
    else if (accel_type == AccelerationType::BVH)
        return ray_intersect_bvh(ray, isc);
    else
        throw std::runtime_error("Unknown acceleration type");
}

EmitterSample Scene::sample_emitter(const Intersection& isc, Float sample1, const Vec3f& sample2) const {
    // sample an emitter index
    uint32_t emitter_index = std::min(int(sample1 * emitters.size()), int(emitters.size() - 1));
    Float emitter_index_pmf = 1.0 / emitters.size();

    auto emitter = emitters[emitter_index];

    EmitterSample emitter_sample = emitter->sampleLi(this, isc, sample2);
    emitter_sample.pdf *= emitter_index_pmf;

    return emitter_sample;
}

Float Scene::pdf_nee(const Intersection& isc, const Vec3f& w) const {
    Ray traced_ray{isc.position + sign(glm::dot(isc.normal, w)) * isc.normal * Epsilon, w, Epsilon, 1e4};
    Intersection traced_isc;
    bool is_hit = this->ray_intersect(traced_ray, traced_isc);

    // Environment light
    if (!is_hit) {
        if (env_map == nullptr)
            return 0.0;
        return Inv4Pi / emitters.size();
    }
    if (traced_isc.shape->emitter == nullptr || glm::dot(traced_isc.dirn, traced_isc.normal) < 0.0)
        return 0.0;

    // traced ray hit an emitter. Find its probability
    Float pdf = 1.0 / emitters.size();
    pdf /= traced_isc.shape->geometries.size();
    pdf /= traced_isc.geom->area();

    // convert to solid angle measure
    Float distance_sqrd = glm::length(traced_isc.position - isc.position);
    distance_sqrd *= distance_sqrd;
    Float abs_cos_theta = std::abs(glm::dot(traced_isc.normal, w));
    if (abs_cos_theta <= Epsilon)
        return 0.0;
    pdf *= distance_sqrd / abs_cos_theta;

    return pdf;
}

std::string Scene::to_string() const {
    std::ostringstream oss;
    oss << "Scene: " << scene_file_path.string() << "\n";
    oss << "  " << sensor->to_string() << "\n";
    oss << "  Emitters" << "(" << emitters.size() << "):\n";
    for (const auto& emitter : emitters)
        oss << "    " << emitter->to_string() << "\n";
    oss << "  Shapes" << "(" << shapes.size() << "):\n";
    for (const auto& shape : shapes) {
        auto shape_str = shape->to_string();
        // indent shape_str by 2 spaces.
        std::istringstream shape_iss(shape_str);
        std::string line;
        while (std::getline(shape_iss, line))
            oss << "    " << line << "\n";
    }
    return oss.str();
}
