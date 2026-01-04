#pragma once
#include "core/Film.h"
#include "core/Geometry.h"
#include "core/MathUtils.h"
#include "core/Sampler.h"

class Sensor {
private:
    Float film_area;

    Float get_film_area() const {
        Float aspect_ratio = static_cast<Float>(film.width) / static_cast<Float>(film.height);
        Float tan_half_fov = std::tan(glm::radians(fov) * 0.5f);
        Float width = 2.0f * tan_half_fov;
        Float height = width / aspect_ratio;
        return width * height;
    }

public:
    Mat4f to_world;
    /// field of view in degrees. between 0 and 180
    Float fov;
    Float near_clip, far_clip;
    Film film;
    Sampler sampler;

    Sensor(const Mat4f &to_world, Float fov, uint32_t sampler_seed, uint32_t film_width, uint32_t film_height, uint32_t spp, Float near_clip, Float far_clip, const RFilter *rfilter) : to_world(to_world), fov(fov), film(film_width, film_height, rfilter), sampler(sampler_seed, spp), near_clip(near_clip), far_clip(far_clip) {
        film_area = get_film_area();
    }

    /// @brief Sample a ray from the sensor through the pixel (row, col) with the given 2D sample in [0,1)^2
    Ray sample_ray(uint32_t row, uint32_t col, const Vec2f &sample2, Float &px, Float &py) {
        // Compute sensor space coordinates
        Float aspect_ratio = static_cast<Float>(film.width) / static_cast<Float>(film.height);
        px = (static_cast<Float>(col) + sample2.x) / static_cast<Float>(film.width);
        py = (static_cast<Float>(row) + sample2.y) / static_cast<Float>(film.height);

        // Map to [-1, 1] range, with correct orientation
        // Horizontal FoV
        Float x = -(2.0 * px - 1.0) * std::tan(glm::radians(fov) * 0.5);
        Float y = (2.0 * py - 1.0) / aspect_ratio * std::tan(glm::radians(fov) * 0.5);

        // Ray in camera space (camera looks toward +Z)
        Vec3f dir_cam{x, y, 1.0};  // +Z forward direction is correct
        dir_cam = glm::normalize(dir_cam);

        // Transform to world space
        Vec3f origin_world = get_origin_world();
        Vec4f dir_world_h = to_world * Vec4f{dir_cam, 0.0f};
        Vec3f dir_world = glm::normalize(Vec3f{dir_world_h});

        return Ray{origin_world, dir_world, near_clip, far_clip};
    }

    /// @brief Used for BDPT. Add the contribution of an incoming light to the correct pixel
    /// @param dirn The direction from camera to the incoming vertex
    /// @param L The throughput radiance
    void add_contrib(Vec3f dirn, Vec3f L) {
        // ignore if not in the camera spectrum.
        // find the corresponding (row, col) pixel. Also find the (px, py) in [0, 1].

        // Transform direction to camera space
        Mat4f to_camera = glm::inverse(to_world);
        Vec4f dirn_cam_h = to_camera * Vec4f{dirn, 0.0f};
        Vec3f dirn_cam = glm::normalize(Vec3f{dirn_cam_h});

        // Check if direction is in front of camera (positive Z in camera space)
        if (dirn_cam.z <= 0.0f)
            return;

        // Project onto image plane (at z = 1)
        Float x = -dirn_cam.x / dirn_cam.z;
        Float y = dirn_cam.y / dirn_cam.z;

        // Convert from camera space to normalized [0, 1] coordinates
        Float aspect_ratio = static_cast<Float>(film.width) / static_cast<Float>(film.height);
        Float tan_half_fov = std::tan(glm::radians(fov) * 0.5f);

        Float px = (x / tan_half_fov + 1.0f) * 0.5f;
        Float py = (y * aspect_ratio / tan_half_fov + 1.0f) * 0.5f;

        if (px < 0.0f || px >= 1.0f || py < 0.0f || py >= 1.0f)
            return;

        Float col = px * static_cast<Float>(film.width);
        Float row = py * static_cast<Float>(film.height);

        // multiply by the importance function
        L *= std::abs(dirn_cam.z);
        L /= film_area;

        film.commit_sample(L, row, col, px, py);
    }

    Vec3f get_origin_world() {
        Vec4f origin_world_h = to_world * Vec4f{0.0, 0.0, 0.0, 1.0};
        return Vec3f{origin_world_h} / origin_world_h.w;
    }

    std::string to_string() {
        std::ostringstream oss;
        oss << "Sensor: [ " << film.to_string() << ", " << sampler.to_string() << ", " << "fov=" << fov << ", near_clip=" << near_clip << ", far_clip=" << far_clip << ", to_world=(NotImplemented) ]";
        return oss.str();
    }
};
