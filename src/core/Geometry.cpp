#include "core/Geometry.h"

bool BVHNode::intersect(const Ray &ray, Intersection &isc) {
    // AABB-ray intersection test
    Float tmin = (bbox.min_corner.x - ray.o.x) / ray.d.x;
    Float tmax = (bbox.max_corner.x - ray.o.x) / ray.d.x;
    if (tmin > tmax)
        std::swap(tmin, tmax);
    Float tymin = (bbox.min_corner.y - ray.o.y) / ray.d.y;
    Float tymax = (bbox.max_corner.y - ray.o.y) / ray.d.y;
    if (tymin > tymax)
        std::swap(tymin, tymax);
    if ((tmin > tymax) || (tymin > tmax))
        return false;
    if (tymin > tmin)
        tmin = tymin;
    if (tymax < tmax)
        tmax = tymax;
    Float tzmin = (bbox.min_corner.z - ray.o.z) / ray.d.z;
    Float tzmax = (bbox.max_corner.z - ray.o.z) / ray.d.z;
    if (tzmin > tzmax)
        std::swap(tzmin, tzmax);
    if ((tmin > tzmax) || (tzmin > tmax))
        return false;
    if (tzmin > tmin)
        tmin = tzmin;
    if (tzmax < tmax)
        tmax = tzmax;
    // Check if AABB intersection is within ray bounds
    if (tmax < ray.tmin || tmin > ray.tmax)
        return false;

    // Leaf node
    if (left == nullptr && right == nullptr) {
        bool is_hit = false;
        Float best_dist = INFINITY;
        for (const auto &geom : geoms) {
            Intersection isc_tmp{};
            bool is_hit_tmp = geom->intersect(ray, isc_tmp);
            if (is_hit_tmp) {
                if (ray.shadow_ray)
                    return true;
                is_hit = true;
                if (isc_tmp.distance < best_dist) {
                    best_dist = isc_tmp.distance;
                    isc = isc_tmp;
                }
            }
        }
        return is_hit;
    }

    bool hit_left = false, hit_right = false;
    Intersection isc_left, isc_right;
    hit_left = left->intersect(ray, isc_left);
    hit_right = right->intersect(ray, isc_right);

    if (hit_left && hit_right) {
        if (isc_left.distance < isc_right.distance)
            isc = isc_left;
        else
            isc = isc_right;
        return true;
    } else if (hit_left) {
        isc = isc_left;
        return true;
    } else if (hit_right) {
        isc = isc_right;
        return true;
    } else {
        return false;
    }
}

bool load_mesh_from_file(const std::string &file_path, const Shape *parent_shape, std::vector<Geometry *> &output_mesh, std::vector<Vec3f *> &vertices, std::vector<Vec3f *> &normals, std::vector<Vec2f *> &texcoords, const std::unordered_map<std::string, std::string> &properties) {
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
            output_mesh.emplace_back(GeometryRegistry::createGeometry("triangle", properties, parent_shape, &gctx));
        } else {
            // Other polygons
            throw std::runtime_error("Quad not implemented");
        }

        index_offset += num_face_vertices;
    }

    return true;
}
