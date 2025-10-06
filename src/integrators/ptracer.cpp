#include "core/Integrator.h"
#include "core/Scene.h"
#include "core/MathUtils.h"
#include "core/Registry.h"
#include <sstream>


class ParticleTracerIntegrator : public Integrator {
private:
    int max_depth;
    // depth to start Russian roulette. refer to mitsuba3 documentation for more details.
    int rr_depth; 
    bool hide_emitters;

public:
    ParticleTracerIntegrator(int max_depth, int rr_depth, bool hide_emitters) : max_depth(max_depth), rr_depth(rr_depth), hide_emitters(hide_emitters) {}

    void render(const Scene *scene, Sensor *sensor, uint32_t n_threads, bool show_progress) override {
    }

    std::string to_string() const override {
        std::ostringstream oss;
        oss << "Integrator(ParticleTracer): [ max_depth=" << max_depth << ", rr_depth=" << rr_depth << ", hide_emitters=" << (hide_emitters ? "true" : "false") << " ]";
        return oss.str();
    }
};


// ------------------- Registry functions -------------------
Integrator* createParticleTracerIntegrator(const std::unordered_map<std::string, std::string>& properties) {
    int max_depth = -1;
    int rr_depth = 5;
    bool hide_emitters = false;

    if (properties.find("max_depth") != properties.end()) {
        max_depth = std::stoi(properties.at("max_depth"));
    }
    if (properties.find("rr_depth") != properties.end()) {
        rr_depth = std::stoi(properties.at("rr_depth"));
    }
    if (properties.find("hide_emitters") != properties.end()) {
        hide_emitters = properties.at("hide_emitters") == "true";
    }

    return new ParticleTracerIntegrator(max_depth, rr_depth, hide_emitters);
}

namespace {
    struct ParticleTracerIntegratorRegistrar {
        ParticleTracerIntegratorRegistrar() {
            IntegratorRegistry::registerIntegrator("ptracer", createParticleTracerIntegrator);
        }
    };
    
    static ParticleTracerIntegratorRegistrar registrar;
}
