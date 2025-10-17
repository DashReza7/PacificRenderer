#include "core/BSDF.h"
#include "core/Constants.h"
#include "core/MathUtils.h"
#include "core/Microfacet.h"
#include "core/Registry.h"
#include "core/Texture.h"


class RoughDielectricBSDF final : public BSDF {
public:
    Float eta;  // int_ior / ext_ior
    Float alpha_u, alpha_v;
    std::string distribution;
    const Microfacet *mf_dist;
    const Texture *specular_reflectance;
    const Texture *specular_transmission;

    RoughDielectricBSDF(BSDFFlags flags, Float eta, Float alpha_u, Float alpha_v, const std::string &distribution, const Texture *specular_reflectance, const Texture *specular_transmission) : BSDF(flags), eta(eta), alpha_u(alpha_u), alpha_v(alpha_v), distribution(distribution), specular_reflectance(specular_reflectance), specular_transmission(specular_transmission) {
        mf_dist = MicrofacetRegistry::createMicrofacet(distribution, {{"alpha_u", std::to_string(alpha_u)}, {"alpha_v", std::to_string(alpha_v)}});
    }
    ~RoughDielectricBSDF() {
        delete mf_dist;
    }

    Vec3f eval(const Intersection &isc, const Vec3f &wo) const override {
        Vec3f wi = worldToLocal(isc.dirn, isc.normal);
        bool is_reflect = wi.z * wo.z >= 0.0;
        Float etap = 1.0;  // eta_o / eta_i
        if (!is_reflect)
            etap = wi.z > 0.0 ? eta : 1.0 / eta;
        Vec3f wm = etap * wo + wi;
        if (std::abs(wi.z) <= Epsilon || std::abs(wo.z) <= Epsilon || glm::length(wm) <= Epsilon)
            return Vec3f{0.0};
        wm = wm * sign(wm.z);
        wm /= glm::length(wm);

        Float fr = fresnelReflection(glm::dot(wi, wm), eta);
        if (is_reflect) {
            // reflection
            Vec3f tmp;
            bool is_refracted = refract(wi, face_forward(wm, wi), eta, tmp);
            if (!is_refracted) {
                // TIR
                Float g = mf_dist->G(wi, wo);
                Float d = mf_dist->D(wm);
                Float bsdf_val = g * d / std::abs(4.0 * wi.z * wo.z) * std::abs(wo.z);
                return Vec3f{bsdf_val} * specular_reflectance->eval(isc);
            } else {
                Float g = mf_dist->G(wi, wo);
                Float d = mf_dist->D(wm);
                Float bsdf_val = g * d / std::abs(4.0 * wi.z * wo.z) * fr * std::abs(wo.z);
                return Vec3f{bsdf_val} * specular_reflectance->eval(isc);
            }
        } else {
            // refraction
            Float g = mf_dist->G(wi, wo);
            Float d = mf_dist->D(wm);
            Float bsdf_val = g * d * Sqr(etap) / Sqr(glm::dot(wi, wm) + etap * glm::dot(wo, wm)) * std::abs(glm::dot(wi, wm) * glm::dot(wo, wm) / wi.z / wo.z) * std::abs(wo.z) * (1.0 - fr);
            // XXX: account for non-symmetry. must be remove when transporting importance
            bsdf_val /= Sqr(etap);
            return Vec3f{bsdf_val} * specular_transmission->eval(isc);
        }
    }

    Float pdf(const Intersection &isc, const Vec3f &wo) const override {
        Vec3f wi = worldToLocal(isc.dirn, isc.normal);
        bool is_reflect = wi.z * wo.z >= 0.0;
        Float etap = 1.0;  // eta_o / eta_i
        if (!is_reflect)
            etap = wi.z > 0.0 ? eta : 1.0 / eta;
        Vec3f wm = etap * wo + wi;
        if (std::abs(wi.z) <= Epsilon || std::abs(wo.z) <= Epsilon || glm::length(wm) <= Epsilon)
            return 0.0;
        wm = wm * sign(wm.z);
        wm /= glm::length(wm);

        Float fr = fresnelReflection(glm::dot(wi, wm), eta);
        if (is_reflect) {
            // reflection
            Vec3f tmp;
            bool is_refracted = refract(wi, face_forward(wm, wi), eta, tmp);
            if (!is_refracted) {
                // TIR
                Float g = mf_dist->G(wi, wo);
                Float d = mf_dist->D(wm);
                return mf_dist->pdf(wi, wm) / (4.0 * std::abs(glm::dot(wi, wm)));
            } else {
                Float g = mf_dist->G(wi, wo);
                Float d = mf_dist->D(wm);
                return mf_dist->pdf(wi, wm) / (4.0 * std::abs(glm::dot(wi, wm))) * fr;
            }
        } else {
            // refraction
            Float denom = Sqr(glm::dot(wo, wm) + glm::dot(wi, wm) / etap);
            Float dwm_dwo = std::abs(glm::dot(wo, wm)) / denom;
            return mf_dist->pdf(wi, wm) * dwm_dwo * (1.0 - fr);
        }
    }

    std::pair<BSDFSample, Vec3f> sample(const Intersection &isc, Float sample1, const Vec2f &sample2) const override {
        Vec3f wi = worldToLocal(isc.dirn, isc.normal);
        if (std::abs(wi.z) <= Epsilon)
            return {BSDFSample{Vec3f{0.0}, 0.0, 1.0, BSDFSampleFlags::None},
                    Vec3f{0.0}};

        Vec3f wm = mf_dist->sample_wm(wi, sample2);
        Float etap = wi.z >= 0.0 ? eta : 1.0 / eta;

        Vec3f wo;
        bool is_refracted = refract(wi, face_forward(wm, wi), etap, wo);
        if (!is_refracted) {
            // TIR
            wo = reflect(wi, face_forward(wm, wi));

            if (wi.z * wo.z <= Epsilon)
                return {BSDFSample{Vec3f{0.0}, 0.0, 1.0, BSDFSampleFlags::None},
                        Vec3f{0.0}};

            Float g = mf_dist->G(wi, wo);
            Float d = mf_dist->D(wm);
            Float bsdf_val = g * d / (4.0 * wi.z * wo.z) * std::abs(wo.z);
            Float pdf = mf_dist->pdf(wi, wm) / (4.0 * std::abs(glm::dot(wi, wm)));
            return {BSDFSample{wo, pdf, 1.0, BSDFSampleFlags::DeltaReflection},
                    Vec3f{bsdf_val} * specular_reflectance->eval(isc)};
        } else {
            Float fr = fresnelReflection(std::abs(glm::dot(wi, wm)), etap);
            if (sample1 <= fr) {
                // reflection
                wo = reflect(wi, face_forward(wm, wi));
                if (std::abs(wo.z) <= Epsilon || wi.z * wo.z <= Epsilon)
                    return {BSDFSample{Vec3f{0.0}, 0.0, 1.0, BSDFSampleFlags::None},
                            Vec3f{0.0}};

                Float g = mf_dist->G(wi, wo);
                Float d = mf_dist->D(wm);
                Float bsdf_val = g * d / (4.0 * wi.z * wo.z) * std::abs(wo.z) * fr;
                Float pdf = mf_dist->pdf(wi, wm) / (4.0 * std::abs(glm::dot(wi, wm))) * fr;
                return {BSDFSample{wo, pdf, 1.0, BSDFSampleFlags::DeltaReflection},
                        Vec3f{bsdf_val} * specular_reflectance->eval(isc)};
            } else {
                // refraction
                if (std::abs(wo.z) <= Epsilon || wi.z * wo.z >= -Epsilon)
                    return {BSDFSample{Vec3f{0.0}, 0.0, 1.0, BSDFSampleFlags::None},
                            Vec3f{0.0}};

                Float g = mf_dist->G(wi, wo);
                Float d = mf_dist->D(wm);
                Float bsdf_val = g * d * Sqr(etap) / Sqr(glm::dot(wi, wm) + etap * glm::dot(wo, wm)) * std::abs(glm::dot(wi, wm) * glm::dot(wo, wm) / wi.z / wo.z) * std::abs(wo.z) * (1.0 - fr);
                // XXX: account for non-symmetry. must be removed when transporting importance
                bsdf_val /= Sqr(etap);

                Float denom = Sqr(glm::dot(wo, wm) + glm::dot(wi, wm) / etap);
                Float dwm_dwo = std::abs(glm::dot(wo, wm)) / denom;
                Float pdf = mf_dist->pdf(wi, wm) * dwm_dwo * (1.0 - fr);
                return {BSDFSample{wo, pdf, Float(1.0) / etap, BSDFSampleFlags::DeltaTransmission},
                        Vec3f{bsdf_val} * specular_transmission->eval(isc)};
            }
        }
    }

    std::string to_string() const override {
        std::ostringstream oss;
        oss << "BSDF(RoughDielectric): [ eta=" << eta << ", distribution=" << distribution << ", alpha_u=" << alpha_u << ", alpha_v=" << alpha_v << " ]";
        return oss.str();
    }
};

// --------------------------- Registry functions ---------------------------
BSDF *createRoughDielectricBSDF(const std::unordered_map<std::string, std::string> &properties, const std::unordered_map<std::string, const Texture *> &textures) {
    Float int_ior = 1.5046;    // bk27
    Float ext_ior = 1.000277;  // air
    Float alpha_u = 0.1, alpha_v = 0.1;
    std::string distribution = "beckmann";
    const Texture *specular_reflectance = TextureRegistry::createTexture("constant", {{"albedo", "1, 1, 1"}});
    const Texture *specular_transmission = TextureRegistry::createTexture("constant", {{"albedo", "1, 1, 1"}});

    for (const auto &[key, value] : properties) {
        if (key == "int_ior") {
            try {
                int_ior = std::stod(value);
            } catch (const std::invalid_argument &) {
                if (!IOR_TABLE.contains(value))
                    throw std::runtime_error("Unknown material '" + value + "' for RoughDielectric BSDF");
                int_ior = IOR_TABLE.at(value);
            }
        } else if (key == "ext_ior") {
            try {
                ext_ior = std::stod(value);
            } catch (const std::invalid_argument &) {
                if (!IOR_TABLE.contains(value))
                    throw std::runtime_error("Unknown material '" + value + "' for RoughDielectric BSDF");
                ext_ior = IOR_TABLE.at(value);
            }
        } else if (key == "alpha") {
            alpha_u = std::stod(value);
            alpha_v = std::stod(value);
        } else if (key == "alpha_u") {
            alpha_u = std::stod(value);
        } else if (key == "alpha_v") {
            alpha_v = std::stod(value);
        } else if (key == "distribution") {
            distribution = value;
        } else if (key == "specular_reflectance") {
            delete specular_reflectance;
            specular_reflectance = TextureRegistry::createTexture("constant", {{"albedo", value}});
        } else if (key == "specular_transmission") {
            delete specular_transmission;
            specular_transmission = TextureRegistry::createTexture("constant", {{"albedo", value}});
        } else {
            throw std::runtime_error("Unknown property '" + key + "' for RoughDielectric BSDF");
        }
    }

    for (const auto &[key, tex_ptr] : textures) {
        if (key == "specular_reflectance") {
            delete specular_reflectance;
            specular_reflectance = tex_ptr;
        } else if (key == "specular_transmission") {
            delete specular_transmission;
            specular_transmission = tex_ptr;
        } else {
            throw std::runtime_error("Unknown texture slot '" + key + "' for RoughDielectric BSDF");
        }
    }

    return new RoughDielectricBSDF(BSDFFlags::PassThrough,
                                   int_ior / ext_ior,
                                   alpha_u, alpha_v,
                                   distribution, specular_reflectance, specular_transmission);
}

namespace {
struct RoughDielectricBSDFRegistrar {
    RoughDielectricBSDFRegistrar() {
        BSDFRegistry::registerBSDF("roughdielectric", createRoughDielectricBSDF);
    }
};

static RoughDielectricBSDFRegistrar registrar;
}  // namespace
