#pragma once
#include <string>

#include "core/Scene.h"


class Scene;

class Integrator {
public:
    Integrator() = default;
    virtual ~Integrator() = default;

    virtual void render(const Scene *scene, Sensor *sensor, uint32_t n_threads = 1) = 0;
    virtual std::string to_string() const = 0;
};

class SamplingIntegrator : public Integrator {
public:
    SamplingIntegrator() = default;
    virtual ~SamplingIntegrator() override = default;

    virtual void render(const Scene *scene, Sensor *sensor, uint32_t n_threads=1) override;
    /// @brief Sample the Radiance along the given ray
    virtual Vec3f sample_radiance(const Scene *scene, Sampler *sampler, const Ray &ray) const = 0;
};

class MonteCarloIntegrator : public SamplingIntegrator {
protected:
    uint32_t max_depth;
    uint32_t rr_depth;

public:
    MonteCarloIntegrator() = default;
    virtual ~MonteCarloIntegrator() override = default;
};
