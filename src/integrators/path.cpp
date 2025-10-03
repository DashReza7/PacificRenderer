#include "core/Integrator.h"
#include "core/Scene.h"
#include "core/MathUtils.h"
#include "core/Registry.h"
#include <sstream>

class PathTracerIntegrator : public MonteCarloIntegrator {
private:
    int max_depth;
    // depth to start Russian roulette. refer to mitsuba3 documentation for more details.
    int rr_depth; 
    bool hide_emitters;

public:
    PathTracerIntegrator(int max_depth, int rr_depth, bool hide_emitters) : max_depth(max_depth), rr_depth(rr_depth), hide_emitters(hide_emitters) {}

    Vec3f sample_radiance(const Scene *scene, Sampler *sampler, const Ray &ray) const override {

    }

    std::string to_string() const override {
        std::ostringstream oss;
        oss << "Integrator(PathTracer): [ max_depth=" << max_depth << ", rr_depth=" << rr_depth << ", hide_emitters=" << (hide_emitters ? "true" : "false") << " ]";
        return oss.str();
    }
};

// ------------------- Registry functions -------------------
Integrator* createPathTracerIntegrator(const std::unordered_map<std::string, std::string>& properties) {
    int max_depth = -1;
    int rr_depth = 5;
    bool hide_emitters = false;

    auto it = properties.find("max_depth");
    if (it != properties.end())
        max_depth = std::stoi(it->second);

    it = properties.find("rr_depth");
    if (it != properties.end())
        rr_depth = std::stoi(it->second);

    it = properties.find("hide_emitters");
    if (it != properties.end())
        hide_emitters = (it->second == "true");

    return new PathTracerIntegrator(max_depth, rr_depth, hide_emitters);
}

namespace {
    struct PathTracerIntegratorRegistrar {
        PathTracerIntegratorRegistrar() {
            IntegratorRegistry::registerIntegrator("path", createPathTracerIntegrator);
        }
    };

    static PathTracerIntegratorRegistrar registrar;
}

