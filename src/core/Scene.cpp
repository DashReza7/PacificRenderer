#include "core/Scene.h"

#include "utils/SceneParser.h"

void Scene::load_shapes(const std::vector<ShapeDesc*> shapes_desc) {
    for (const auto& shape_desc : shapes_desc) {
        auto shape = new Shape{};
        // TODO:
        shape->bsdf = nullptr;

        if (shape_desc->type == "obj") {
            // TODO:
            std::vector<Vec3f*> vertices;
            std::vector<Vec3f*> normals;
            std::vector<Vec2f*> texcoords;
            bool success = MeshLoader::load_mesh_from_file((scene_file_directory / shape_desc->properties["filename"]).string(),
                                                           shape->geometries, vertices, normals, texcoords);
            if (!success) {
                throw std::runtime_error("Failed to load OBJ mesh: " + shape_desc->properties["filename"]);
            }

            // apply transform
            for (auto& vertex : vertices)
                *vertex = Vec3f{shape_desc->to_world * Vec4f{*vertex, 1.0}};
            // use the inverse transpose of the upper-left 3x3 part of the matrix
            // TODO: check for correctness
            for (auto& normal : normals)
                *normal = glm::normalize(Vec3f{glm::transpose(glm::inverse(shape_desc->to_world)) * Vec4f{*normal, 0.0}});

            for (auto& geom : shape->geometries) {
                geom->parent_shape = shape;
            }

        } else if (shape_desc->type == "sphere") {
            auto sphere = new Sphere{Vec3f{0}, 1.0, shape_desc->to_world};
            sphere->parent_shape = shape;
            if (shape_desc->properties.find("center") != shape_desc->properties.end()) {
                // split the shape_desc->properties["center"] by comma and space, and convert to double.
                // TODO: right now only support comma. xyz format not supported.
                std::string center_str = shape_desc->properties["center"];
                std::istringstream ss(center_str);
                std::string token;
                std::vector<double> center;
                while (std::getline(ss, token, ',')) {
                    center.push_back(std::stod(token));
                }
                if (center.size() == 3)
                    sphere->center = Vec3f{center[0], center[1], center[2]};
            }
            if (shape_desc->properties.find("radius") != shape_desc->properties.end())
                sphere->radius = std::stod(shape_desc->properties["radius"]);
            shape->geometries.push_back(sphere);

        } else {
            throw std::runtime_error("Unsupported shape type: " + shape_desc->type);
        }

        shapes.push_back(shape);
    }
}

void Scene::build_bvh(BVHNode* node, const std::vector<Geometry*>& contained_geoms) {
    // TODO: check for empty list of shapes
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

// TODO: complete this
void Scene::load_scene(const SceneDesc& scene_desc) {
    load_shapes(scene_desc.shapes);

    bvh_root = new BVHNode{};
    build_bvh(bvh_root, get_all_geoms());
}

std::vector<Geometry*> Scene::get_all_geoms() {
    std::vector<Geometry*> all_geoms;
    for (const auto& shape : shapes)
        all_geoms.insert(all_geoms.end(), shape->geometries.begin(), shape->geometries.end());
    return all_geoms;
}

std::string Scene::get_bvh_str(BVHNode* node, int idt) {
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

void Scene::print_bvh_statistics() {
    // compute statistics like number of nodes, depth, average number of geometries per leaf node, etc.
    // also check if any non-leaf node has geometries with size > 0
    if (!bvh_root) {
        std::cout << "BVH not built yet." << std::endl;
        return;
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
                std::cout << "Non-leaf node with geometries found." << std::endl;
                exit(EXIT_FAILURE);
            }
        }
        traverse(node->left, depth + 1);
        traverse(node->right, depth + 1);
    };
    traverse(bvh_root, 1);
    std::cout << "BVH Statistics:" << std::endl;
    std::cout << "  Number of nodes: " << num_nodes << std::endl;
    std::cout << "  Number of leaf nodes: " << num_leaf_nodes << std::endl;
    std::cout << "  Max depth: " << max_depth << std::endl;
    if (num_leaf_nodes > 0)
        std::cout << "  Average geometries per leaf: " << (total_geoms_in_leaves / num_leaf_nodes) << std::endl;
    else
        std::cout << "  Average geometries per leaf: N/A" << std::endl;
    std::cout << "  Max geometries in leaf: " << max_geoms_in_leaf << std::endl;
}

bool Scene::intersect_brute_force(const Ray &ray, Intersection &isc) {
    bool is_hit = false;
    Float closest_t = ray.tmax;

    for (const auto& shape : shapes)
        for (const auto& geom : shape->geometries) {
            Intersection temp_isc;
            if (geom->intersect(ray, temp_isc)) {
                Float t = glm::length(temp_isc.position - ray.o);
                if (t < closest_t) {
                    closest_t = t;
                    isc = temp_isc;
                    is_hit = true;
                }
            }
        }

    return is_hit;
}
