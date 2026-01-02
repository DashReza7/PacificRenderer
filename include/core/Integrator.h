#pragma once
#include <string>

#include "core/Scene.h"

class Scene;

class Integrator {
public:
    virtual void render(const Scene *scene, Sensor *sensor, uint32_t n_threads, bool show_progress) = 0;
    virtual std::string to_string() const = 0;
};

class SamplingIntegrator : public Integrator {
public:
    virtual void render(const Scene *scene, Sensor *sensor, uint32_t n_threads, bool show_progress) override;
    /// @brief Sample the Radiance along the given ray
    virtual Vec3f sample_radiance(const Scene *scene, Sampler *sampler, const Ray &ray, int row, int col) const = 0;

    /// @brief finds the MIS weight for the NEE
    virtual Float get_mis_weight_nee(const Intersection &isc, const EmitterSample &emitter_sample, uint32_t n_bsdf_samples) const;

    /// @brief finds the MIS weight for the BSDF sampling
    virtual Float get_mis_weight_bsdf(const Scene *scene, const Intersection &isc, const BSDFSample &bsdf_sample, uint32_t n_emitter_samples) const;
};

class MonteCarloIntegrator : public SamplingIntegrator {
protected:
    int max_depth;
    // depth to start Russian roulette. refer to mitsuba3 documentation for more details.
    int rr_depth;

public:
    MonteCarloIntegrator(int max_depth, int rr_depth) : max_depth(max_depth), rr_depth(rr_depth) {};
};

class AdjointIntegrator : public Integrator {
public:
    virtual void render(const Scene *scene, Sensor *sensor, uint32_t n_threads, bool show_progress) override;
    virtual void sample(const Scene *scene, const Sensor *sensor);
};