#include "core/Microfacet.h"
#include "core/Registry.h"

class BeckmannDistribution : public Microfacet {
private:
    Float alpha;

    Float D(const Vec3f &w, const Vec3f &wm) const {
        return G1(w) * D(wm) * std::abs(glm::dot(w, wm)) / std::abs(w.z);
    }
    Float G1(const Vec3f &w) const {
        return 1.0 / (1.0 + Lambda(w));
    }
    Float Lambda(const Vec3f &w) const {
        if (std::abs(w.z) <= Epsilon)
            return 0.0;
        Float a = 1.0 / (alpha * std::abs(tan_theta(w)));
        if (a >= 1.6)
            return 0.0;
        return (1.0 - 1.259 * a + 0.396 * Sqr(a)) / (3.535 * a + 2.181 * Sqr(a));
    }

public:
    BeckmannDistribution(Float alpha) : alpha(alpha) {}

    // Remark: the returned wm is in the upper hemisphere (wm.z >= 0)
    // w will be inverted if it is in the lower hemisphere
    Vec3f sample_wm(const Vec3f &w, const Vec2f &sample) const override {
        throw std::runtime_error("BeckmannDistribution is not implemented yet");
        Vec3f omega{w};
        if (w.z < 0.0)
            omega = -omega;
        Float phi = 2 * Pi * sample.y;
        Float theta = 0.0;
        if (sample.x == 0.0)
            theta = PiOver2;
        else if (sample.x == 1.0)
            theta = 0.0;
        else
            theta = std::atan(std::sqrt(std::abs(Sqr(alpha) * std::log(sample.x))));
        Vec3f wm_local{std::sin(theta) * std::cos(phi), std::sin(theta) * std::sin(phi), std::cos(theta)};
        return localToWorld(wm_local, omega);
    }

    Float pdf(const Vec3f &w, const Vec3f &wm) const override {
        return D(w, wm);
    }

    Float D(const Vec3f &wm) const override {
        if (std::abs(wm.z) <= Epsilon)
            return 0.0;
        Float cos2Theta = wm.z * wm.z;
        Float tan2Theta = tan2_theta(wm);
        return std::exp(-tan2Theta / Sqr(alpha)) / (Pi * Sqr(alpha) * Sqr(cos2Theta));
    }

    Float G(const Vec3f &wi, const Vec3f &wo) const override {
        return 1.0 / (1.0 + Lambda(wi) + Lambda(wo));
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
