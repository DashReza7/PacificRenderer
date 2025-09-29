#pragma once
#include "core/MathUtils.h"
#include "core/Sampler.h"
#include "core/Film.h"

class Sensor {
private:
    Mat4f to_world;
    /// field of view in degrees. between 0 and 180
    Float fov;
    Float near_clip, far_clip;
    Film film;
    Sampler sampler;

public:
    Sensor(const Mat4f &to_world, Float fov, uint32_t sampler_seed, uint32_t film_width, uint32_t film_height, uint32_t spp, Float near_clip, Float far_clip) : to_world(to_world), fov(fov), film(film_width, film_height), sampler(sampler_seed, spp), near_clip(near_clip), far_clip(far_clip) {}

    std::string to_string() {
        std::ostringstream oss;
        oss << "Sensor: [ " << film.to_string() << ", " << sampler.to_string() << ", " << "fov=" << fov << ", near_clip=" << near_clip << ", far_clip=" << far_clip << ", to_world=(NotImplemented) ]";
        return oss.str();
    }
};
