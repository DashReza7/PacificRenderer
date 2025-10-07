#include "core/Registry.h"
#include "core/RFilter.h"

class GaussianFilter : public RFilter {
private:
    Float stddev;
    
public:
    GaussianFilter(Float radius, Float stddev) : RFilter(radius), stddev(stddev) {}

    Float eval(Float x, Float y) const override {
        Float r = glm::length(Vec2f{x, y});
        if (r > radius)
            return 0.0f;
            
        return std::exp(-0.5f * Sqr(r / stddev));
    }
};


// --------------------------- Registry functions ---------------------------
RFilter *createGaussianFilter(const std::unordered_map<std::string, std::string> &properties) {
    Float radius = 2.0f;
    Float stddev = 0.5f;

    for (const auto &[key, value] : properties) {
        if (key == "radius") {
            radius = std::stod(value);
        } else if (key == "stddev") {
            stddev = std::stod(value);
        } else {
            throw std::runtime_error("Unknown property '" + key + "' for Gaussian RFilter");
        }
    }
    return new GaussianFilter{radius, stddev};
}

namespace {
struct GaussianFilterRegistrar {
    GaussianFilterRegistrar() {
        RFilterRegistry::registerRFilter("gaussian", createGaussianFilter);
    }
};

static GaussianFilterRegistrar registrar;
}  // namespace

