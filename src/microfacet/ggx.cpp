#include "core/Microfacet.h"
#include "core/Registry.h"

class GGXDistribution : public Microfacet {
private:
    Float alpha_u, alpha_v;

    Float D(const Vec3f &w, const Vec3f &wm) const {
        return G1(w) * D(wm) * std::abs(glm::dot(w, wm)) / std::abs(w.z);
    }
    Float G1(const Vec3f &w) const {
        return 1.0 / (1.0 + Lambda(w));
    }
    Float Lambda(const Vec3f &w) const {
        if (std::abs(w.z) <= Epsilon)
            return 0.0;
        Float tan2Theta = tan2_theta(w);
        Float alpha2 = Sqr(cos_phi(w) * alpha_u) + Sqr(sin_phi(w) * alpha_v);
        return (std::sqrt(1 + alpha2 * tan2Theta) - 1.0) / 2.0;
    }

public:
    GGXDistribution(Float alpha_u, Float alpha_v) : alpha_u(alpha_u), alpha_v(alpha_v) {}

    // Remark: the returned wm is in the upper hemisphere (wm.z >= 0)
    // w will be inverted if it is in the lower hemisphere
    Vec3f sample_wm(const Vec3f &w, const Vec2f &sample) const override {
        Vec3f wh = glm::normalize(Vec3f(alpha_u * w.x, alpha_v * w.y, w.z));
        if (wh.z < 0)
            wh = -wh;

        Vec3f T1 = (wh.z < 0.99999) ? glm::normalize(glm::cross(Vec3f{0, 0, 1}, wh)) : Vec3f{1, 0, 0};
        Vec3f T2 = glm::cross(wh, T1);

        Vec2f p = uniformDiskSample(sample);

        Float h = std::sqrt(1 - Sqr(p.x));
        p.y = lerp((1.0 + wh.z) / 2.0, h, p.y);

        Float pz = std::sqrt(std::max<Float>(0, 1 - glm::dot(p, p)));
        Vec3f nh = p.x * T1 + p.y * T2 + pz * wh;

        return glm::normalize(Vec3f(alpha_u * nh.x, alpha_v * nh.y, std::max<Float>(Float(1e-6), nh.z)));
    }

    Float pdf(const Vec3f &w, const Vec3f &wm) const override {
        return D(w, wm);
    }

    Float D(const Vec3f &wm) const override {
        if (std::abs(wm.z) <= Epsilon)
            return 0.0;
        Float tan2Theta = tan2_theta(wm);
        Float cos4_theta = Sqr(cos2_theta(wm));
        Float e = tan2Theta * (Sqr(cos_phi(wm) / alpha_u) +
                               Sqr(sin_phi(wm) / alpha_v));
        return 1.0 / (Pi * alpha_u * alpha_v * cos4_theta * Sqr(1 + e));
    }

    Float G(const Vec3f &wi, const Vec3f &wo) const override {
        return 1.0 / (1.0 + Lambda(wi) + Lambda(wo));
    }
};

// --------------------------- Registry functions ---------------------------
Microfacet *createGGXMicrofacet(const std::unordered_map<std::string, std::string> &properties) {
    Float alpha_u = 0.1, alpha_v = 0.1;

    for (const auto &[key, value] : properties) {
        if (key == "alpha_u") {
            alpha_u = std::stod(value);
            if (alpha_u <= 0)
                throw std::runtime_error("GGXMicrofacet: alpha_u must be positive");
            if (!properties.contains("alpha_v"))
                throw std::runtime_error("GGXMicrofacet: Must specify both alpha_u and alpha_v");
        } else if (key == "alpha_v") {
            alpha_v = std::stod(value);
            if (alpha_v <= 0)
                throw std::runtime_error("GGXMicrofacet: alpha_v must be positive");
            if (!properties.contains("alpha_u"))
                throw std::runtime_error("GGXMicrofacet: Must specify both alpha_u and alpha_v");
        } else if (key == "alpha") {
            alpha_u = alpha_v = std::stod(value);
            if (alpha_u <= 0)
                throw std::runtime_error("GGXMicrofacet: alpha must be positive");
            if (properties.contains("alpha_u") || properties.contains("alpha_v"))
                throw std::runtime_error("GGXMicrofacet: Cannot specify both alpha and alpha_u/alpha_v");
        } else {
            throw std::runtime_error("GGXMicrofacet: Unknown property " + key);
        }
    }

    return new GGXDistribution{alpha_u, alpha_v};
}

namespace {
struct GGXMicrofacetRegistrar {
    GGXMicrofacetRegistrar() {
        MicrofacetRegistry::registerMicrofacet("ggx", createGGXMicrofacet);
    }
};

static GGXMicrofacetRegistrar registrar;
}  // namespace
