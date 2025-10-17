#include "core/BSDF.h"
#include "core/Registry.h"
#include "core/Texture.h"
#include "utils/Misc.h"


// implementation based on Mitsuba3
class SmoothPlasticBSDF final : public BSDF {
public:
    const Texture *diffuse_reflectance;
    const Texture *specular_reflectance;
    Float eta;  // int_ior / ext_ior
    Float fdr_int, fdr_ext;
    Float specular_sampling_weight;
    bool nonlinear;

    SmoothPlasticBSDF(BSDFFlags flags, Float eta, const Texture *diffuse_reflectance, const Texture *specular_reflectance, bool nonlinear)
        : BSDF(flags), eta(eta), diffuse_reflectance(diffuse_reflectance), specular_reflectance(specular_reflectance), nonlinear(nonlinear) {
        // Compute Fresnel reflectance values
        fdr_int = fresnel_diffuse_reflectance(1.0 / eta);
        fdr_ext = fresnel_diffuse_reflectance(eta);

        Float diff_mean = diffuse_reflectance->mean();
        Float spec_mean = specular_reflectance->mean();
        specular_sampling_weight = spec_mean / (diff_mean + spec_mean);
    }

    Vec3f eval(const Intersection &isc, const Vec3f &wo) const override {
        Vec3f wi = worldToLocal(isc.dirn, isc.normal);
        if (wi.z * wo.z <= 0.0)
            return Vec3f{0.0};

        Float fi = fresnelReflection(std::abs(wi.z), eta);
        Float fo = fresnelReflection(std::abs(wo.z), eta);
        // only the diffuse lobe (specular lobe has delta distribution)
        Vec3f bsdf_value = diffuse_reflectance->eval(isc);
        bsdf_value /= Vec3f{1.0} - (nonlinear ? (bsdf_value * fdr_int) : Vec3f{fdr_int});
        bsdf_value *= cosineHemispherePDF(wi, wo) / Sqr(eta) * (1.0 - fi) * (1.0 - fo);

        return bsdf_value;
    }

    Float pdf(const Intersection &isc, const Vec3f &wo) const override {
        Vec3f wi = worldToLocal(isc.dirn, isc.normal);
        if (wi.z * wo.z <= 0.0)
            return 0.0f;
        // only the diffuse lobe (specular lobe has delta distribution)
        Float fi = fresnelReflection(std::abs(wi.z), eta);
        Float prob_specular = fi * specular_sampling_weight;
        Float prob_diffuse = (1.0 - fi) * (1.0 - specular_sampling_weight);
        prob_diffuse = prob_diffuse / (prob_specular + prob_diffuse);
        
        return cosineHemispherePDF(wi, wo) * prob_diffuse;
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
            Vec3f wo{-wi.x, -wi.y, wi.z};
            Float pdf = prob_specular;
            Vec3f bsdf_value = Vec3f{fi} * specular_reflectance->eval(isc);

            return {BSDFSample{wo, pdf, 1.0, BSDFSampleFlags::DeltaReflection},
                    bsdf_value};
        } else {
            // diffuse reflection
            Vec3f wo = cosineHemisphereSample(sample2);
            if (wi.z < 0.0)
                wo.z = -wo.z;
            Float pdf = cosineHemispherePDF(wi, wo) * prob_diffuse;
            Float fo = fresnelReflection(std::abs(wo.z), eta);
            Vec3f bsdf_value = diffuse_reflectance->eval(isc);
            bsdf_value /= Vec3f{1.0} - (nonlinear ? (bsdf_value * fdr_int) : Vec3f{fdr_int});
            bsdf_value /= Sqr(eta);
            bsdf_value *= (1.0 - fi) * (1.0 - fo);
            bsdf_value *= cosineHemispherePDF(wi, wo);

            return {BSDFSample{wo, pdf, 1.0, BSDFSampleFlags::DiffuseReflection},
                    bsdf_value};
        }
    }

    std::string to_string() const override {
        std::ostringstream oss;
        oss << "BSDF(Diffuse): [ reflectance= ]";
        return oss.str();
    }
};

// --------------------------- Registry functions ---------------------------
BSDF *createSmoothPlasticBSDF(const std::unordered_map<std::string, std::string> &properties, const std::unordered_map<std::string, const Texture *> &textures) {
    const Texture *diffuse_reflectance = TextureRegistry::createTexture("constant", {{"albedo", "0.5, 0.5, 0.5"}});
    const Texture *specular_reflectance = TextureRegistry::createTexture("constant", {{"albedo", "1, 1, 1"}});
    bool nonlinear = false;
    Float int_ior = 1.49;  // Polyperopylene
    Float ext_ior = 1.000277;  // air
    BSDFFlags flags = BSDFFlags::None;

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
        } else {
            throw std::runtime_error("Unknown property '" + key + "' for SmoothPlastic BSDF");
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
            throw std::runtime_error("Unknown texture slot '" + key + "' for SmoothPlastic BSDF");
        }
    }

    return new SmoothPlasticBSDF(flags, int_ior / ext_ior, diffuse_reflectance, specular_reflectance, nonlinear);
}

namespace {
struct SmoothPlasticBSDFRegistrar {
    SmoothPlasticBSDFRegistrar() {
        BSDFRegistry::registerBSDF("plastic", createSmoothPlasticBSDF);
    }
};

static SmoothPlasticBSDFRegistrar registrar;
}  // namespace
