#pragma once

class Scene;

class Integrator {
protected:
    const Scene* scene;

public:
    Integrator(const Scene* scene) : scene(scene) {}
    virtual ~Integrator() = default;

    virtual void render() = 0;
    virtual std::string to_string() const = 0;
};

class DirectLightingIntegrator : public Integrator {
private:
    int emitter_samples;
    int bsdf_samples;
    bool hide_emitters;
    
public:
    DirectLightingIntegrator(const Scene* scene, int emitter_samples=1, int bsdf_samples=1, bool hide_emitters=false)
        : Integrator(scene), emitter_samples(emitter_samples), bsdf_samples(bsdf_samples), hide_emitters(hide_emitters) {}
    ~DirectLightingIntegrator() override = default;

    void render() override {
    }

    std::string to_string() const override {
        std::ostringstream oss;
        oss << "Integrator(DirectLighting): [ emitter_samples=" << emitter_samples << ", bsdf_samples=" << bsdf_samples << ", hide_emitters=" << (hide_emitters ? "true" : "false") << " ]";
        return oss.str();
    }
};

class PathTracer : public Integrator {
private:
    int max_depth;
    // depth to start Russian roulette. refer to mitsuba3 documentation for more details.
    int rr_depth = 5; 
    bool hide_emitters;

public:
    PathTracer(const Scene* scene, int max_depth=-1, int rr_depth=5, bool hide_emitters=false) : Integrator(scene), max_depth(max_depth), rr_depth(rr_depth), hide_emitters(hide_emitters) {}

    void render() override {
    }

    std::string to_string() const override {
        std::ostringstream oss;
        oss << "Integrator(PathTracer): [ max_depth=" << max_depth << ", rr_depth=" << rr_depth << ", hide_emitters=" << (hide_emitters ? "true" : "false") << " ]";
        return oss.str();
    }
};

class DepthIntegrator : public Integrator {
public:
    DepthIntegrator(const Scene* scene) : Integrator(scene) {}

    void render() override {
    }

    std::string to_string() const override {
        return "Integrator(Depth): [ ]";
    }
};

class GeometricNormalIntegrator : public Integrator {
public:
    GeometricNormalIntegrator(const Scene* scene) : Integrator(scene) {}

    void render() override {
    }

    std::string to_string() const override {
        return "Integrator(GeometricNormal): [ ]";
    }
};

class AlbedoIntegrator : public Integrator {
public:
    AlbedoIntegrator(const Scene* scene) : Integrator(scene) {}

    void render() override {
    }

    std::string to_string() const override {
        return "Integrator(Albedo): [ ]";
    }
};

class ParticleTracer : public Integrator {
private:
    int max_depth;
    // depth to start Russian roulette. refer to mitsuba3 documentation for more details.
    int rr_depth = 5; 
    bool hide_emitters;

public:
    ParticleTracer(const Scene* scene, int max_depth=-1, int rr_depth=5, bool hide_emitters=false)
        : Integrator(scene), max_depth(max_depth), rr_depth(rr_depth), hide_emitters(hide_emitters) {}
        
    void render() override {
    }

    std::string to_string() const override {
        std::ostringstream oss;
        oss << "Integrator(ParticleTracer): [ max_depth=" << max_depth << ", rr_depth=" << rr_depth << ", hide_emitters=" << (hide_emitters ? "true" : "false") << " ]";
        return oss.str();
    }
};
