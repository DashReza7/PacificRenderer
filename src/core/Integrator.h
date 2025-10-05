#pragma once
#include <string>

#include "core/Scene.h"

class Scene;

class Integrator {
public:
    virtual void render(const Scene *scene, Sensor *sensor, uint32_t n_threads = 1) = 0;
    virtual std::string to_string() const = 0;
};

class SamplingIntegrator : public Integrator {
public:
    virtual void render(const Scene *scene, Sensor *sensor, uint32_t n_threads = 1) override;
    /// @brief Sample the Radiance along the given ray
    virtual Vec3f sample_radiance(const Scene *scene, Sampler *sampler, const Ray &ray) const = 0;

    /// @brief finds the MIS weight for the NEE
    virtual Float get_mis_weight_nee(const Intersection &isc, const EmitterSample &emitter_sample) const;

    /// @brief finds the MIS weight for the BSDF sampling
    virtual Float get_mis_weight_bsdf(const Scene *scene, const Intersection &isc, const BSDFSample &bsdf_sample) const;
};

class MonteCarloIntegrator : public SamplingIntegrator {
protected:
    uint32_t max_depth;
    // depth to start Russian roulette. refer to mitsuba3 documentation for more details.
    uint32_t rr_depth;

public:
    MonteCarloIntegrator(uint32_t max_depth, uint32_t rr_depth) : max_depth(max_depth), rr_depth(rr_depth) {};
};
