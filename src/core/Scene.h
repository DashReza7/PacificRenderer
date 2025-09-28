#pragma once

#include <memory>
#include <vector>
#include <string>

#include "core/Integrator.h"
#include "core/Sensor.h"
#include "core/Shape.h"
#include "utils/SceneParser.h"

class Scene {
private:
    std::vector<Shape *> shapes{};
    BVHNode *bvh_root = nullptr;
    Sensor *sensor = nullptr;
    Integrator *integrator = nullptr;

    void load_shapes(const std::vector<ShapeDesc *> shapes_desc);

    void build_bvh(BVHNode *node, const std::vector<Geometry *> &contained_geoms);

    // get all geometries in a single vector (independent of Shape)
    std::vector<Geometry *> get_all_geoms();
    
public:
    std::filesystem::path scene_file_directory;

    Scene() = default;
    ~Scene() = default;

    void load_scene(const SceneDesc &scene_desc);

    std::string get_bvh_str(BVHNode *node = nullptr, int idt = 0);

    void print_bvh_statistics();

    bool intersect_brute_force(const Ray &ray, Intersection &isc);
};
