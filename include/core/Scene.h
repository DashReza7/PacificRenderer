#pragma once

#include <memory>
#include <string>
#include <vector>
#include <filesystem>

#include "core/Emitter.h"
#include "core/Sensor.h"
#include "core/Shape.h"
#include "utils/SceneParser.h"

extern std::filesystem::path scene_file_path;

class Scene {
private:
    std::vector<Shape*> shapes{};
    BVHNode *bvh_root = nullptr;
    std::vector<Emitter*> emitters{};

    std::vector<Geometry*> get_all_geoms() const;
    bool ray_intersect_bruteforce(const Ray &ray, Intersection &isc) const;
    bool ray_intersect_bvh(const Ray &ray, Intersection &isc) const;

public:
    AccelerationType accel_type = AccelerationType::BVH;
    Sensor *sensor = nullptr;
    Emitter *env_map = nullptr;

    void load_scene(const SceneDesc &scene_desc);
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
    /// @brief Get the pdf of a direction, as if sampled by the emitter sampling technique. Used for finding MIS weights
    /// @param isc the intersection point we want to find the direction pdf from
    /// @param w the direction we want to compute its pdf
    /// @return The direction's pdf in solid angle measure
    Float pdf_nee(const Intersection &isc, const Vec3f &w) const;
    /// @brief sample a posn & dirn on a light source, Used for particle tracing
    /// @return RGB contribution value
    Vec3f sample_emitter_ptrace(Vec2f sample1, Vec3f sample2, Float sample3, 
                                Vec3f &posn, Vec3f &normal, Vec3f &dirn, Float &pdf_posn, Float &pdf_dirn) const;
    std::string to_string() const;
};
