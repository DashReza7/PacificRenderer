#pragma once

#include <memory>
#include <string>
#include <vector>

#include "core/Emitter.h"
#include "core/Integrator.h"
#include "core/Sensor.h"
#include "core/Shape.h"
#include "utils/SceneParser.h"


class Scene {
private:
    std::vector<Shape *> shapes{};
    BVHNode *bvh_root = nullptr;
    Sensor *sensor = nullptr;
    std::vector<Emitter *> emitters{};

    std::unordered_map<BSDFDesc *, BSDF *> load_bsdfs(const std::vector<BSDFDesc *> &bsdfs_desc);
    std::unordered_map<EmitterDesc *, Emitter *> load_emitters(const std::vector<EmitterDesc *> &emitters_desc);
    void load_shapes(const std::vector<ShapeDesc *> shapes_desc, const std::unordered_map<BSDFDesc *, BSDF *> &bsdfs_dict, const std::unordered_map<EmitterDesc *, Emitter *> &emitters_dict);
    // TODO: right now, only perspective pinhole camera
    void load_sensor(const SensorDesc *sensor_desc);
    void build_bvh(BVHNode *node, const std::vector<Geometry *> &contained_geoms);
    // get all geometries in a single vector (independent of Shape)
    std::vector<Geometry *> get_all_geoms();
    Integrator *load_integrator(const IntegratorDesc *integrator_desc);

public:
    std::filesystem::path scene_file_path;

    /// Load the scene from a description. Returns an integrator.
    Integrator *load_scene(const SceneDesc &scene_desc);
    /// Get a string representation of the BVH tree.
    std::string get_bvh_str(BVHNode *node = nullptr, int idt = 0);
    /// Get statistics about the BVH. Number of nodes, leaf nodes, max depth, average number of geometries per leaf, max number of geometries in a leaf.
    std::string get_bvh_statistics();
    /// Ray-scene intersection using brute-force method (no BVH)
    bool intersect_brute_force(const Ray &ray, Intersection &isc);
    std::string to_string() const;
};
