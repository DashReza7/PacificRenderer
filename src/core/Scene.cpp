#include "core/Scene.h"

#include "utils/SceneParser.h"

Integrator* Scene::load_scene(const SceneDesc& scene_desc) {
    auto bsdfs_dict = load_bsdfs(scene_desc.bsdfs);
    auto emitters_dict = load_emitters(scene_desc.emitters);

    load_shapes(scene_desc.shapes, bsdfs_dict, emitters_dict);
    bvh_root = new BVHNode{};
    build_bvh(bvh_root, get_all_geoms());

    load_sensor(scene_desc.sensor);

    return load_integrator(scene_desc.integrator);
}

std::unordered_map<BSDFDesc*, BSDF*> Scene::load_bsdfs(const std::vector<BSDFDesc*>& bsdfs_desc) {
    std::unordered_map<BSDFDesc*, BSDF*> bsdfs{};
    for (const auto& bsdf_desc : bsdfs_desc) {
        BSDF* bsdf = nullptr;
        if (bsdf_desc->type == "diffuse") {
            Vec3f reflectance{0.5};
            if (bsdf_desc->properties.find("reflectance") != bsdf_desc->properties.end())
                reflectance = strToVec3f(bsdf_desc->properties["reflectance"]);
            bsdf = new DiffuseBSDF{reflectance};

        } else if (bsdf_desc->type == "dielectric") {
            Float int_ior = 1.5046;
            Float ext_ior = 1.000277;
            if (bsdf_desc->properties.find("int_ior") != bsdf_desc->properties.end())
                int_ior = static_cast<Float>(std::stod(bsdf_desc->properties["int_ior"]));
            if (bsdf_desc->properties.find("ext_ior") != bsdf_desc->properties.end())
                ext_ior = static_cast<Float>(std::stod(bsdf_desc->properties["ext_ior"]));
            bsdf = new SmoothDielectricBSDF{int_ior, ext_ior};
        } else {
            throw std::runtime_error("Unsupported BSDF type: " + bsdf_desc->type);
        }

        bsdfs[bsdf_desc] = bsdf;
    }

    return bsdfs;
}

void Scene::load_shapes(const std::vector<ShapeDesc*> shapes_desc, const std::unordered_map<BSDFDesc*, BSDF*>& bsdfs_dict, const std::unordered_map<EmitterDesc*, Emitter*>& emitters_dict) {
    for (const auto& shape_desc : shapes_desc) {
        auto shape = new Shape{};
        if (shape_desc->bsdf == nullptr) {
            throw std::runtime_error("Shape missing BSDF");
        }
        shape->bsdf = bsdfs_dict.at(shape_desc->bsdf);

        if (shape_desc->type == "obj") {
            shape->type = Shape::Type::OBJ;
            std::vector<Vec3f*> vertices;
            std::vector<Vec3f*> normals;
            std::vector<Vec2f*> texcoords;
            bool success = MeshLoader::load_mesh_from_file((scene_file_path.parent_path() / shape_desc->properties["filename"]).string(),
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
            shape->type = Shape::Type::SPHERE;
            auto sphere = new Sphere{Vec3f{0}, 1.0, shape_desc->to_world};
            sphere->parent_shape = shape;
            if (shape_desc->properties.find("center") != shape_desc->properties.end())
                sphere->center = strToVec3f(shape_desc->properties["center"]);
            if (shape_desc->properties.find("radius") != shape_desc->properties.end())
                sphere->radius = std::stod(shape_desc->properties["radius"]);
            shape->geometries.push_back(sphere);

        } else {
            throw std::runtime_error("Unsupported shape type: " + shape_desc->type);
        }

        // if the shape has an emitter(it's an area_light), link them
        if (shape_desc->emitter != nullptr) {
            shape->emitter = emitters_dict.at(shape_desc->emitter);
            dynamic_cast<AreaLight*>(shape->emitter)->shape = shape;
        }

        shapes.push_back(shape);
    }
}

std::unordered_map<EmitterDesc*, Emitter*> Scene::load_emitters(const std::vector<EmitterDesc*>& emitters_desc) {
    std::unordered_map<EmitterDesc*, Emitter*> emitters_dict{};
    Emitter* emitter = nullptr;
    for (const auto& emitter_desc : emitters_desc) {
        if (emitter_desc->type == "point") {
            Vec3f intensity{1.0};
            Vec3f position{0.0};
            Mat4f to_world = Mat4f(1.0);
            if (emitter_desc->properties.find("intensity") != emitter_desc->properties.end())
                intensity = strToVec3f(emitter_desc->properties["intensity"]);
            if (emitter_desc->properties.find("position") != emitter_desc->properties.end())
                position = strToVec3f(emitter_desc->properties["position"]);
            if (emitter_desc->properties.find("to_world") != emitter_desc->properties.end())
                to_world = strToMat4f(emitter_desc->properties["to_world"]);

            emitter = new PointLight{intensity, position, to_world};
        } else if (emitter_desc->type == "area") {
            Vec3f radiance{1.0};
            if (emitter_desc->properties.find("radiance") != emitter_desc->properties.end())
                radiance = strToVec3f(emitter_desc->properties["radiance"]);

            emitter = new AreaLight{radiance};
        } else {
            throw std::runtime_error("Unsupported emitter type: " + emitter_desc->type);
        }
        emitters.push_back(emitter);
        emitters_dict[emitter_desc] = emitter;
    }

    return emitters_dict;
}

Integrator* Scene::load_integrator(const IntegratorDesc* integrator_desc) {
    Integrator* integrator = nullptr;
    if (integrator_desc->type == "direct") {
        int emitter_samples = 1;
        int bsdf_samples = 1;
        bool hide_emitters = false;
        if (integrator_desc->properties.find("emitter_samples") != integrator_desc->properties.end())
            emitter_samples = std::stoi(integrator_desc->properties.at("emitter_samples"));
        if (integrator_desc->properties.find("bsdf_samples") != integrator_desc->properties.end())
            bsdf_samples = std::stoi(integrator_desc->properties.at("bsdf_samples"));
        if (integrator_desc->properties.find("hide_emitters") != integrator_desc->properties.end())
            hide_emitters = (integrator_desc->properties.at("hide_emitters") == "true");
        integrator = new DirectLightingIntegrator{this, emitter_samples, bsdf_samples, hide_emitters};

    } else if (integrator_desc->type == "path") {
        int max_depth = -1;
        int rr_depth = 5;
        if (integrator_desc->properties.find("max_depth") != integrator_desc->properties.end())
            max_depth = std::stoi(integrator_desc->properties.at("max_depth"));
        if (integrator_desc->properties.find("rr_depth") != integrator_desc->properties.end())
            rr_depth = std::stoi(integrator_desc->properties.at("rr_depth"));
        integrator = new PathTracer{this, max_depth, rr_depth};

    } else {
        throw std::runtime_error("Unsupported integrator type: " + integrator_desc->type);
    }

    return integrator;
}

void Scene::load_sensor(const SensorDesc* sensor_desc) {
    Float fov = 45.0;
    Float near_clip = 1e-2, far_clip = 1e4;
    uint32_t width = 800, height = 600;
    uint32_t spp = 1;
    if (sensor_desc->properties.find("fov") != sensor_desc->properties.end())
        fov = static_cast<Float>(std::stod(sensor_desc->properties.at("fov")));
    if (sensor_desc->properties.find("near_clip") != sensor_desc->properties.end())
        near_clip = static_cast<Float>(std::stod(sensor_desc->properties.at("near_clip")));
    if (sensor_desc->properties.find("far_clip") != sensor_desc->properties.end())
        far_clip = static_cast<Float>(std::stod(sensor_desc->properties.at("far_clip")));
    if (sensor_desc->film->properties.find("width") != sensor_desc->film->properties.end())
        width = static_cast<uint32_t>(std::stoi(sensor_desc->film->properties.at("width")));
    if (sensor_desc->film->properties.find("height") != sensor_desc->film->properties.end())
        height = static_cast<uint32_t>(std::stoi(sensor_desc->film->properties.at("height")));
    if (sensor_desc->sampler->properties.find("sample_count") != sensor_desc->sampler->properties.end())
        spp = static_cast<uint32_t>(std::stoi(sensor_desc->sampler->properties.at("sample_count")));
    sensor = new Sensor{sensor_desc->to_world, fov, 0, width, height, spp, near_clip, far_clip};
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

std::string Scene::get_bvh_statistics() {
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
                std::cerr << "Non-leaf node with geometries found." << std::endl;
                exit(EXIT_FAILURE);
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

bool Scene::intersect_brute_force(const Ray& ray, Intersection& isc) {
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
