#include "core/Microfacet.h"
#include "core/Registry.h"

class BeckmannDistribution : public Microfacet {
private:
    Float alpha;

    Float G1(const Vec3f &w) const {
        if (std::abs(w.z) <= Epsilon)
            return 0.0;
        if (std::abs(w.z) >= 1.0 - Epsilon)
            return 1.0;
            
        Float a = 1.0 / (alpha * tan_theta(w));
        return 2.0 / (1.0 + std::erf(a) + std::exp(-Sqr(a)) / (std::sqrt(Pi) * a));
    }

public:
    BeckmannDistribution(Float alpha) : alpha(alpha) {}

    Vec3f sample_wm(const Vec3f &w, const Vec2f &sample) const override {
        Float theta;
        if (sample.x <= Epsilon)
            theta = PiOver2;
        else
            theta = std::atan(std::sqrt(-Sqr(alpha) * std::log(sample.x)));
        Float phi = 2.0 * Pi * sample.y;
        return sphericalToCartesian(theta, phi);
    }

    Float pdf(const Vec3f &w, const Vec3f &wm) const override {
        return D(wm);
    }

    Float D(const Vec3f &wm) const override {
        if (std::abs(wm.z) <= Epsilon)
            return 0.0;
        Float cos2Theta = wm.z * wm.z;
        Float tan2Theta = tan2_theta(wm);

        return std::exp(-tan2Theta / Sqr(alpha)) / (Pi * Sqr(alpha) * Sqr(cos2Theta));
    }

    Float G(const Vec3f &wi, const Vec3f &wo) const override {
        return G1(wi) * G1(wo);
    }
};

// --------------------------- Registry functions ---------------------------
Microfacet *createBeckmannMicrofacet(const std::unordered_map<std::string, std::string> &properties) {
    Float alpha = 0.1;

    if (properties.contains("alpha_u") && properties.contains("alpha_v") && properties.at("alpha_u") != properties.at("alpha_v"))
        throw std::runtime_error("BeckmannMicrofacet: Anisotropic Beckmann is not supported yet");
    for (const auto &[key, value] : properties) {
        if (key == "alpha_u") {
            alpha = std::stod(value);
            if (alpha <= 0)
                throw std::runtime_error("BeckmannMicrofacet: alpha must be positive");
        } else if (key == "alpha_v") {
            alpha = std::stod(value);
            if (alpha <= 0)
                throw std::runtime_error("BeckmannMicrofacet: alpha must be positive");
        } else if (key == "alpha") {
            alpha = std::stod(value);
            if (alpha <= 0)
                throw std::runtime_error("BeckmannMicrofacet: alpha must be positive");
        } else {
            throw std::runtime_error("BeckmannMicrofacet: Unknown property " + key);
        }
    }

    return new BeckmannDistribution{std::sqrt(Float(2.0)) * alpha};
}

namespace {
struct BeckmannMicrofacetRegistrar {
    BeckmannMicrofacetRegistrar() {
        MicrofacetRegistry::registerMicrofacet("beckmann", createBeckmannMicrofacet);
    }
};

static BeckmannMicrofacetRegistrar registrar;
}  // namespace
