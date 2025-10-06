#pragma once
#include "core/Film.h"
#include "core/Geometry.h"
#include "core/MathUtils.h"
#include "core/Sampler.h"


class Sensor {
public:
    Mat4f to_world;
    /// field of view in degrees. between 0 and 180
    Float fov;
    Float near_clip, far_clip;
    Film film;
    Sampler sampler;

    Sensor(const Mat4f &to_world, Float fov, uint32_t sampler_seed, uint32_t film_width, uint32_t film_height, uint32_t spp, Float near_clip, Float far_clip, const RFilter *rfilter) : to_world(to_world), fov(fov), film(film_width, film_height, rfilter), sampler(sampler_seed, spp), near_clip(near_clip), far_clip(far_clip) {}

    /// @brief Sample a ray from the sensor through the pixel (row, col) with the given 2D sample in [0,1)^2
    Ray sample_ray(uint32_t row, uint32_t col, const Vec2f &sample2, Float &px, Float &py) {
        // Compute sensor space coordinates
        Float aspect_ratio = static_cast<Float>(film.width) / static_cast<Float>(film.height);
        px = (static_cast<Float>(col) + sample2.x) / static_cast<Float>(film.width);
        py = (static_cast<Float>(row) + sample2.y) / static_cast<Float>(film.height);

        // Map to [-1, 1] range, with correct orientation
        Float x = -(2.0 * px - 1.0) * aspect_ratio * std::tan(glm::radians(fov) * 0.5);  // Negate for +X = left
        Float y = (2.0 * py - 1.0) * std::tan(glm::radians(fov) * 0.5);                  // +Y = up

        // Ray in camera space (camera looks toward +Z)
        Vec3f origin_cam{0, 0, 0};
        Vec3f dir_cam{x, y, 1.0};  // +Z forward direction is correct
        dir_cam = glm::normalize(dir_cam);

        // Transform to world space
        Vec4f origin_world_h = to_world * Vec4f{origin_cam, 1.0f};
        Vec4f dir_world_h = to_world * Vec4f{dir_cam, 0.0f};
        Vec3f origin_world = Vec3f{origin_world_h} / origin_world_h.w;
        Vec3f dir_world = glm::normalize(Vec3f{dir_world_h});

        return Ray{origin_world, dir_world, near_clip, far_clip};
    }

    std::string to_string() {
        std::ostringstream oss;
        oss << "Sensor: [ " << film.to_string() << ", " << sampler.to_string() << ", " << "fov=" << fov << ", near_clip=" << near_clip << ", far_clip=" << far_clip << ", to_world=(NotImplemented) ]";
        return oss.str();
    }
};
