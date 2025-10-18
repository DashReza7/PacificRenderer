#include "core/BSDF.h"
#include "core/Microfacet.h"
#include "core/Registry.h"
#include "core/Texture.h"
#include "utils/Misc.h"


// implementation based on Mitsuba3
class RoughPlasticBSDF final : public BSDF {
public:
    const Texture *diffuse_reflectance;
    const Texture *specular_reflectance;
    Float eta;  // int_ior / ext_ior
    Float fdr_int, fdr_ext;
    Float specular_sampling_weight;
    bool nonlinear;
    Float alpha_u, alpha_v;
    const Microfacet *mf_dist;

    RoughPlasticBSDF(BSDFFlags flags, Float eta, const Texture *diffuse_reflectance, const Texture *specular_reflectance, bool nonlinear, Float alpha_u, Float alpha_v, const std::string &distribution)
        : BSDF(flags), eta(eta), diffuse_reflectance(diffuse_reflectance), specular_reflectance(specular_reflectance), nonlinear(nonlinear), alpha_u(alpha_u), alpha_v(alpha_v) {
        mf_dist = MicrofacetRegistry::createMicrofacet(distribution, {{"alpha_u", std::to_string(alpha_u)}, {"alpha_v", std::to_string(alpha_v)}});

        fdr_int = fresnel_diffuse_reflectance(1.0 / eta);
        fdr_ext = fresnel_diffuse_reflectance(eta);

        Float diff_mean = diffuse_reflectance->mean();
        Float spec_mean = specular_reflectance->mean();
        specular_sampling_weight = spec_mean / (diff_mean + spec_mean);
    }

    Vec3f eval(const Intersection &isc, const Vec3f &wo) const override {
        Vec3f wi = worldToLocal(isc.dirn, isc.normal);
        if (wi.z <= 0.0 && !has_flag(BSDFFlags::TwoSided))
            return Vec3f{0.0};
        if (wi.z * wo.z <= 0.0)
            return Vec3f{0.0};

        Float fi = fresnelReflection(std::abs(wi.z), eta);
        Float fo = fresnelReflection(std::abs(wo.z), eta);
        // diffuse lobe
        Vec3f bsdf_value = diffuse_reflectance->eval(isc);
        bsdf_value /= Vec3f{1.0} - (nonlinear ? (bsdf_value * fdr_int) : Vec3f{fdr_int});
        bsdf_value *= cosineHemispherePDF(wi, wo) / Sqr(eta) * (1.0 - fi) * (1.0 - fo);

        // glossy lobe
        Vec3f wm = wi + wo;
        Float wm_len = glm::length(wm);
        if (wm_len <= Epsilon)
            return Vec3f{0.0};
        wm /= wm_len;
        bsdf_value += fresnelReflection(std::abs(glm::dot(wi, wm)), eta)
                                * mf_dist->D(wm) 
                                * mf_dist->G(wi, wo)
                                / (Float(4.0) * std::abs(wi.z))
                                * specular_reflectance->eval(isc);

        return bsdf_value;
    }

    Float pdf(const Intersection &isc, const Vec3f &wo) const override {
        Vec3f wi = worldToLocal(isc.dirn, isc.normal);
        if (wi.z <= 0.0 && !has_flag(BSDFFlags::TwoSided))
            return 0.0;
        if (wi.z * wo.z <= 0.0)
            return 0.0f;

        Float fi = fresnelReflection(std::abs(wi.z), eta);
        Float prob_glossy = fi * specular_sampling_weight;
        Float prob_diffuse = (1.0 - fi) * (1.0 - specular_sampling_weight);
        prob_glossy = prob_glossy / (prob_glossy + prob_diffuse);
        prob_diffuse = 1.0 - prob_glossy;

        Vec3f wm = wi + wo;
        Float wm_len = glm::length(wm);
        if (wm_len <= Epsilon)
            return 0.0;
        wm /= wm_len;
        Float pdf_mf = mf_dist->pdf(wi, wm) / (4.0 * glm::dot(wo, wm));

        return prob_glossy * pdf_mf + prob_diffuse * cosineHemispherePDF(wi, wo);
    }

    std::pair<BSDFSample, Vec3f> sample(const Intersection &isc, Float sample1, const Vec2f &sample2) const override {
        Vec3f wi = worldToLocal(isc.dirn, isc.normal);
        if (wi.z <= 0.0 && !this->has_flag(BSDFFlags::TwoSided))
            return {BSDFSample{Vec3f{0.0}, 0.0, 1, BSDFSampleFlags::None},
                    Vec3f{0.0}};

        Float fi = fresnelReflection(std::abs(wi.z), eta);
        Float prob_specular = fi * specular_sampling_weight;
        Float prob_diffuse = (1.0 - fi) * (1.0 - specular_sampling_weight);
        prob_specular = prob_specular / (prob_specular + prob_diffuse);
        prob_diffuse = 1.0 - prob_specular;

        if (sample1 < prob_specular) {
            // specular reflection
            Vec3f wm = mf_dist->sample_wm(wi, sample2);
            wm = wm * glm::sign(wi.z);
            Vec3f wo = reflect(wi, wm);
            if (std::abs(wi.z) <= Epsilon || std::abs(wo.z) <= Epsilon)
                return {BSDFSample{Vec3f{0.0}, 0.0, 1, BSDFSampleFlags::None},
                        Vec3f{0.0}};
            // Float pdf = prob_specular * mf_dist->pdf(wi, wm);
            Float pdf_val = pdf(isc, wo);

            Float g = mf_dist->G(wi, wo);
            Float d = mf_dist->D(wm);
            Float fr = fresnelReflection(std::abs(glm::dot(wi, wm)), eta);
            // Vec3f bsdf = fr * d * g
            //                     / (Float(4.0) * std::abs(wi.z))
            //                     * specular_reflectance->eval(isc);
            Vec3f bsdf = eval(isc, wo);
            return {BSDFSample{wo, pdf_val, 1.0, BSDFSampleFlags::GlossyReflection},
                    bsdf};
        } else {
            // diffuse reflection
            Vec3f wo = cosineHemisphereSample(sample2);
            if (wi.z < 0.0)
                wo.z = -wo.z;
            // Float pdf = cosineHemispherePDF(wi, wo) * prob_diffuse;
            Float pdf_val = pdf(isc, wo);
            Float fo = fresnelReflection(std::abs(wo.z), eta);
            // Vec3f bsdf_value = diffuse_reflectance->eval(isc);
            // bsdf_value /= Vec3f{1.0} - (nonlinear ? (bsdf_value * fdr_int) : Vec3f{fdr_int});
            // bsdf_value /= Sqr(eta);
            // bsdf_value *= (1.0 - fi) * (1.0 - fo);
            // bsdf_value *= cosineHemispherePDF(wi, wo);
            Vec3f bsdf = eval(isc, wo);

            return {BSDFSample{wo, pdf_val, 1.0, BSDFSampleFlags::DiffuseReflection},
                    bsdf};
        }
    }

    std::string to_string() const override {
        std::ostringstream oss;
        oss << "BSDF(RoughPlastic): [ reflectance= ]";
        return oss.str();
    }
};

// --------------------------- Registry functions ---------------------------
BSDF *createRoughPlasticBSDF(const std::unordered_map<std::string, std::string> &properties, const std::unordered_map<std::string, const Texture *> &textures) {
    throw std::runtime_error("RoughPlasticBSDF has some bugs and is disabled in this build.");
    
    const Texture *diffuse_reflectance = TextureRegistry::createTexture("constant", {{"albedo", "0.5, 0.5, 0.5"}});
    const Texture *specular_reflectance = TextureRegistry::createTexture("constant", {{"albedo", "1, 1, 1"}});
    bool nonlinear = false;
    Float int_ior = 1.49;      // Polyperopylene
    Float ext_ior = 1.000277;  // air
    BSDFFlags flags = BSDFFlags::None;
    Float alpha_u = 0.1;
    Float alpha_v = 0.1;
    std::string distribution = "beckmann";

    for (const auto &[key, value] : properties) {
        if (key == "diffuse_reflectance") {
            delete diffuse_reflectance;
            diffuse_reflectance = TextureRegistry::createTexture("constant", {{"albedo", value}});
        } else if (key == "specular_reflectance") {
            delete specular_reflectance;
            specular_reflectance = TextureRegistry::createTexture("constant", {{"albedo", value}});
        } else if (key == "twosided") {
            flags = BSDFFlags::TwoSided;
        } else if (key == "nonlinear") {
            nonlinear = (value == "true" || value == "1");
        } else if (key == "int_ior") {
            int_ior = std::stod(value);
        } else if (key == "ext_ior") {
            ext_ior = std::stod(value);
        } else if (key == "alpha_u") {
            alpha_u = std::stod(value);
        } else if (key == "alpha_v") {
            alpha_v = std::stod(value);
        } else if (key == "alpha") {
            alpha_u = alpha_v = std::stod(value);
        } else if (key == "distribution") {
            distribution = value;
        } else {
            throw std::runtime_error("Unknown property '" + key + "' for RoughPlastic BSDF");
        }
    }

    for (const auto &[key, tex_ptr] : textures) {
        if (key == "diffuse_reflectance") {
            delete diffuse_reflectance;
            diffuse_reflectance = tex_ptr;
        } else if (key == "specular_reflectance") {
            delete specular_reflectance;
            specular_reflectance = tex_ptr;
        } else {
            throw std::runtime_error("Unknown texture slot '" + key + "' for RoughPlastic BSDF");
        }
    }

    return new RoughPlasticBSDF(flags, int_ior / ext_ior, diffuse_reflectance, specular_reflectance, nonlinear, alpha_u, alpha_v, distribution);
}

namespace {
struct RoughPlasticBSDFRegistrar {
    RoughPlasticBSDFRegistrar() {
        BSDFRegistry::registerBSDF("roughplastic", createRoughPlasticBSDF);
    }
};

static RoughPlasticBSDFRegistrar registrar;
}  // namespace
