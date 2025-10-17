#include "core/BSDF.h"
#include "core/Microfacet.h"
#include "core/Registry.h"
#include "utils/Misc.h"
#include "core/Texture.h"


// Texture reflectance not supported
class RoughConductorBSDF final : public BSDF {
public:
    Vec3f eta, k;              // real and imaginary parts of IOR
    std::string distribution;  // ggx or beckmann
    Float alpha_u, alpha_v;
    Microfacet *mf_dist;
    const Texture *specular_reflectance;

    RoughConductorBSDF(BSDFFlags flags, const Vec3f &eta, const Vec3f &k, std::string distribution, Float alpha_u, Float alpha_v, const Texture *specular_reflectance) : BSDF(flags), eta(eta), k(k), distribution(distribution), alpha_u(alpha_u), alpha_v(alpha_v), specular_reflectance(specular_reflectance) {
        mf_dist = MicrofacetRegistry::createMicrofacet(distribution, {{"alpha_u", std::to_string(alpha_u)}, {"alpha_v", std::to_string(alpha_v)}});
    }
    ~RoughConductorBSDF() {
        delete mf_dist;
    }

    Vec3f eval(const Intersection &isc, const Vec3f &wo) const override {
        Vec3f wi = worldToLocal(isc.dirn, isc.normal);
        if (wi.z <= 0.0 && !has_flag(BSDFFlags::TwoSided))
            return Vec3f{0.0};
        if (wi.z * wo.z <= 0.0)
            return Vec3f{0.0};

        Vec3f wm = wi + wo;
        Float wm_len = glm::length(wm);
        if (wm_len <= Epsilon)
            return Vec3f{0.0};
        wm /= wm_len;

        return mf_dist->D(wm) * fresnelComplex(std::abs(glm::dot(wi, wm)), eta, k) 
            * mf_dist->G(wi, wo) 
            / (Float(4.0) * std::abs(cos_theta(wi)) * std::abs(cos_theta(wo))) 
            * std::abs(cos_theta(wo))
            * specular_reflectance->eval(isc);
    }

    Float pdf(const Intersection &isc, const Vec3f &wo) const override {
        Vec3f wi = worldToLocal(isc.dirn, isc.normal);
        if (wi.z <= 0.0 && !has_flag(BSDFFlags::TwoSided))
            return 0.0;
        if (wi.z * wo.z <= 0.0)
            return 0.0;

        Vec3f wm = wi + wo;
        Float wm_len = glm::length(wm);
        if (wm_len <= Epsilon)
            return 0.0;
        wm /= wm_len;

        return mf_dist->pdf(wi, wm) / (4.0 * glm::dot(wo, wm));
    }

    std::pair<BSDFSample, Vec3f> sample(const Intersection &isc, Float sample1, const Vec2f &sample2) const override {
        Vec3f wi = worldToLocal(isc.dirn, isc.normal);
        if (wi.z <= 0.0 && !has_flag(BSDFFlags::TwoSided))
            return {BSDFSample{Vec3f{0.0}, 0.0, 1.0, BSDFSampleFlags::None},
                    Vec3f{0.0}};

        Vec3f wm = mf_dist->sample_wm(wi, sample2);
        wm = wm * glm::sign(wi.z);
        Vec3f wo = reflect(wi, wm);

        Float pdf_val = pdf(isc, wo);
        return {BSDFSample{wo, pdf_val, 1.0, BSDFSampleFlags::GlossyReflection},
                eval(isc, wo)};
    }

    std::string to_string() const override {
        std::ostringstream oss;
        oss << "BSDF(RoughConductor): [ eta=" << eta << ", k=" << k << ", distribution=" << distribution << ", alpha_u=" << alpha_u << ", alpha_v=" << alpha_v << " ]";
        return oss.str();
    }
};

// --------------------------- Registry functions ---------------------------
BSDF *createRoughConductorBSDF(const std::unordered_map<std::string, std::string> &properties, const std::unordered_map<std::string, const Texture *> &textures) {
    Vec3f eta{0.0, 0.0, 0.0};
    Vec3f k{1.0, 1.0, 1.0};
    BSDFFlags flags = BSDFFlags::None;
    std::string distribution = "beckmann";
    Float alpha_u = 0.1, alpha_v = 0.1;
    const Texture *specular_reflectance = TextureRegistry::createTexture("constant", {{"albedo", "1, 1, 1"}});

    for (const auto &[key, value] : properties) {
        if (key == "material") {
            throw std::runtime_error("SmoothConductorBSDF: named material not supported");
        } else if (key == "eta") {
            eta = strToVec3f(value);
        } else if (key == "k") {
            k = strToVec3f(value);
        } else if (key == "twosided" && (value == "true" || value == "1")) {
            flags |= BSDFFlags::TwoSided;
        } else if (key == "distribution") {
            distribution = value;
            if (distribution != "beckmann" && distribution != "ggx")
                throw std::runtime_error("SmoothConductorBSDF: Unsupported distribution " + distribution);
        } else if (key == "alpha_u") {
            alpha_u = std::stof(value);
            if (alpha_u <= 0.0)
                throw std::runtime_error("SmoothConductorBSDF: alpha_u should be positive");
        } else if (key == "alpha_v") {
            alpha_v = std::stof(value);
            if (alpha_v <= 0.0)
                throw std::runtime_error("SmoothConductorBSDF: alpha_v should be positive");
        } else if (key == "alpha") {
            if (properties.contains("alpha_u") || properties.contains("alpha_v"))
                throw std::runtime_error("SmoothConductorBSDF: alpha cannot be used with alpha_u or alpha_v");
            alpha_u = alpha_v = std::stod(value);
        } else if (key == "specular_reflectance") {
            delete specular_reflectance;
            specular_reflectance = TextureRegistry::createTexture("constant", {{"albedo", value}});
        } else {
            throw std::runtime_error("SmoothConductorBSDF: Unknown property " + key);
        }
    }

    for (const auto &[key, tex_ptr] : textures) {
        if (key == "specular_reflectance") {
            delete specular_reflectance;
            specular_reflectance = tex_ptr;
        } else {
            throw std::runtime_error("Unknown texture slot '" + key + "' for RoughConductorBSDF");
        }
    }

    return new RoughConductorBSDF(flags, eta, k, distribution, alpha_u, alpha_v, specular_reflectance);
}

namespace {
struct RoughConductorBSDFRegistrar {
    RoughConductorBSDFRegistrar() {
        BSDFRegistry::registerBSDF("roughconductor", createRoughConductorBSDF);
    }
};

static RoughConductorBSDFRegistrar registrar;
}  // namespace
