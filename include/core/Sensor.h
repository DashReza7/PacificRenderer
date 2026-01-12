#pragma once
#include "core/Film.h"
#include "core/Geometry.h"
#include "core/MathUtils.h"
#include "core/Sampler.h"

class Sensor {
private:
    Float aspect_ratio;
    Float fov;  // field of view in degrees. between 0 and 180
    Float near_clip, far_clip;

public:
    Mat4f to_world;
    Vec3f origin_world;
    Vec3f forward_world;
    Float film_area;  // the film area at z=1
    Film film;
    Sampler sampler;

    Sensor(const Mat4f &to_world, Float fov, uint32_t sampler_seed, uint32_t film_width, uint32_t film_height,
           uint32_t spp, Float near_clip, Float far_clip, const RFilter *rfilter)
        : to_world(to_world), fov(fov), film(film_width, film_height, rfilter), sampler(sampler_seed, spp), near_clip(near_clip), far_clip(far_clip) {
        // compute film area
        aspect_ratio = static_cast<Float>(film.width) / static_cast<Float>(film.height);
        Float tan_half_fov = std::tan(glm::radians(fov) * 0.5f);
        Float width = 2.0f * tan_half_fov;
        Float height = width / aspect_ratio;
        film_area = width * height;

        Vec4f origin_world_h = to_world * Vec4f{0, 0, 0, 1};
        origin_world = Vec3f{origin_world_h} / origin_world_h.w;

        forward_world = glm::normalize(Vec3f{to_world * Vec4f{0, 0, 1, 0}});
    }

    // get the importance value given a dirn in the world from camera position
    Vec3f We(const Vec3f &dirn) {
        Vec2f p;
        Vec3f dirn_cam;
        bool is_valid = worldToIplane(dirn, p, dirn_cam);

        if (!is_valid)
            return Vec3f{0};

        Float cos2Theta = dirn_cam.z * dirn_cam.z;
        return Vec3f{1.0} / (film_area * Sqr(cos2Theta));
    }

    // return the directional(solid angle measure) and positional (area measure. equals 1 for pinhole camera) pdf
    // dirn is normalized from camera to world
    void pdf_We(const Vec3f &dirn, Float &pdf_pos, Float &pdf_dir) {
        Vec2f p_film;
        Vec3f dirn_cam;
        bool is_valid = worldToIplane(dirn, p_film, dirn_cam);
        if (!is_valid) {
            pdf_pos = pdf_dir = 0;
        } else {
            // TODO: Is abs really needed?
            Float cosTheta = std::abs(dirn_cam.z);
            // NOTE: this is only for pinhole camera
            pdf_pos = 1.0;
            pdf_dir = 1.0 / (film_area * cosTheta * cosTheta * cosTheta);
        }
    }

    /// @brief Sample a ray from the sensor through the pixel (row, col) with the given 2D sample in [0,1)^2
    Ray sample_ray(uint32_t row, uint32_t col, const Vec2f &sample2, Float &px, Float &py) {
        px = (static_cast<Float>(col) + sample2.x) / static_cast<Float>(film.width);   // in [0, 1)
        py = (static_cast<Float>(row) + sample2.y) / static_cast<Float>(film.height);  // in [0, 1)
        Vec3f dir_world = iplaneToWorld(px, py);

        return Ray{origin_world, dir_world, near_clip, far_clip};
    }

    /// @brief analogous to light sample_li
    /// @param isc the isc point we want to connect to camera
    /// @param w the direction from isc to camera
    /// @param pdf the pdf of selecting camera in solid angle measure
    /// @param p_film the position of the passing ray on the film plane ( in [0, 1) )
    /// @return importance (the result of We function)
    Vec3f sample_Wi(const Intersection &isc, Vec3f &w, Float &pdf, Vec2f &p_film) {
        w = glm::normalize(origin_world - isc.position);
        Float dist = glm::length(origin_world - isc.position);

        Vec3f cam_dirn;
        bool is_valid = worldToIplane(-w, p_film, cam_dirn);
        if (!is_valid) {
            w = Vec3f{0};
            pdf = 0;
            p_film = Vec2f{-1};
            return Vec3f{0};
        }
        pdf = Sqr(dist) / (std::abs(cam_dirn.z));

        return We(-w);
    }

    // return the dirn from camera to a point on the image plane in world coordinates
    // px & py are in normalized image space ( in [0, 1) )
    Vec3f iplaneToWorld(Float px, Float py) {
        // Map to [-1, 1] range, with correct orientation
        // Horizontal FoV
        Float x = -(2.0 * px - 1.0) * std::tan(glm::radians(fov) * 0.5);
        Float y = (2.0 * py - 1.0) / aspect_ratio * std::tan(glm::radians(fov) * 0.5);

        // dirn in camera space (camera looks toward +Z)
        Vec3f dir_cam = glm::normalize(Vec3f{x, y, 1.0});

        // Transform to world space
        Vec4f dir_world_h = to_world * Vec4f{dir_cam, 0.0f};
        Vec3f dir_world = glm::normalize(Vec3f{dir_world_h});

        return dir_world;
    }

    /// return the [px, py] ( in [0, 1) ) from the world direction (pointing from camera to the next vertex)
    /// dirn_cam is the dirn in the camera space (pointing to +Z)
    bool worldToIplane(const Vec3f &dirn, Vec2f &p, Vec3f &dirn_cam) {
        // Transform direction to camera space
        Mat4f to_camera = glm::inverse(to_world);
        Vec4f dirn_cam_h = to_camera * Vec4f{dirn, 0.0f};
        dirn_cam = glm::normalize(Vec3f{dirn_cam_h});

        // Check if direction is in front of camera (positive Z in camera space)
        if (dirn_cam.z <= 0.0f)
            return false;

        // Project onto image plane (at z = +1)
        Float x = -dirn_cam.x / dirn_cam.z;
        Float y = dirn_cam.y / dirn_cam.z;
        // Convert from camera space to normalized [0, 1) coordinates
        Float tan_half_fov = std::tan(glm::radians(fov) * 0.5f);
        Float px = (x / tan_half_fov + 1.0f) * 0.5f;
        Float py = (y * aspect_ratio / tan_half_fov + 1.0f) * 0.5f;

        if (px < 0.0f || px >= 1.0f || py < 0.0f || py >= 1.0f)
            return false;

        p = Vec2f{px, py};
        return true;
    }

    std::string to_string() {
        std::ostringstream oss;
        oss << "Sensor: [ " << film.to_string() << ", " << sampler.to_string() << ", " << "fov=" << fov << ", near_clip=" << near_clip << ", far_clip=" << far_clip << ", to_world=(NotImplemented) ]";
        return oss.str();
    }
};
