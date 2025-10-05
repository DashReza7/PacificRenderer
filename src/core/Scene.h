#pragma once

#include <memory>
#include <string>
#include <vector>

#include "core/Emitter.h"
#include "core/Sensor.h"
#include "core/Shape.h"
#include "utils/SceneParser.h"


class Scene {
private:
    std::vector<Shape *> shapes{};
    BVHNode *bvh_root = nullptr;
    AccelerationType accel_type = AccelerationType::BVH;
    std::vector<Emitter *> emitters{};


    std::unordered_map<BSDFDesc *, BSDF *> load_bsdfs(const std::vector<BSDFDesc *> &bsdfs_desc);
    std::unordered_map<EmitterDesc *, Emitter *> load_emitters(const std::vector<EmitterDesc *> &emitters_desc);
    void load_shapes(const std::vector<ShapeDesc *> shapes_desc, const std::unordered_map<BSDFDesc *, BSDF *> &bsdfs_dict, const std::unordered_map<EmitterDesc *, Emitter *> &emitters_dict);
    // TODO: right now, only perspective pinhole camera
    void load_sensor(const SensorDesc *sensor_desc);
    void build_bvh(BVHNode *node, const std::vector<Geometry *> &contained_geoms);
    /// get all geometries in a single vector (independent of Shape)
    std::vector<Geometry *> get_all_geoms() const;
    /// Ray-scene intersection using brute-force method (no BVH)
    bool ray_intersect_bruteforce(const Ray &ray, Intersection &isc) const;
    bool ray_intersect_bvh(const Ray &ray, Intersection &isc) const;

    
public:
    Sensor *sensor = nullptr;
    std::filesystem::path scene_file_path;


    /// Load the scene from a description. Returns an integrator.
    void load_scene(const SceneDesc &scene_desc);
    /// Get a string representation of the BVH tree.
    std::string get_bvh_str(BVHNode *node = nullptr, int idt = 0) const;
    /// Get statistics about the BVH. Number of nodes, leaf nodes, max depth, average number of geometries per leaf, max number of geometries in a leaf.
    std::string get_bvh_statistics() const;
    bool ray_intersect(const Ray &ray, Intersection &isc) const;
    /// @brief Sample an emitter in the scene, given a surface intersection point
    /// @param isc The surface intersection point
    /// @param sample1 Uniform sample on [0,1) to sample the emitter index
    /// @param sample2 Uniform sample on [0,1)^3
    /// @return An EmitterSample struct containing the sampled position, radiance, is_valid, and pdf(in solid angel, not area measuere!)
    EmitterSample sample_emitter(const Intersection &isc, Float sample1, const Vec3f &sample2) const;
    /// @brief Get the pdf of a direction, if sampled by the emitter sampling technique. Used for finding MIS weights
    /// @param isc the intersection point we want to find the direction pdf from
    /// @param w the direction we want to compute its pdf
    /// @return The direction's pdf in solid angle measure
    Float pdf_nee(const Intersection &isc, const Vec3f &w) const;
    std::string to_string() const;
};
