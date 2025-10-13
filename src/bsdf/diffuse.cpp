#include "core/BSDF.h"
#include "core/Registry.h"
#include "utils/Misc.h"
#include "core/Texture.h"

class DiffuseBSDF final : public BSDF {
public:
    const Texture *reflectance;
    bool cosine_sampling;

    DiffuseBSDF(BSDFFlags flags, const Texture *reflectance, bool cosine_sampling) : BSDF(flags), reflectance(reflectance), cosine_sampling(cosine_sampling) {}

    Vec3f eval(const Intersection &isc, const Vec3f &wo) const override {
        Vec3f wi = worldToLocal(isc.dirn, isc.normal);
        if (wi.z * wo.z <= 0.0)
            return Vec3f{0.0};
        return InvPi * std::abs(wo.z) * reflectance->eval(isc);
    }

    Float pdf(const Intersection &isc, const Vec3f &wo) const override {
        Vec3f wi = worldToLocal(isc.dirn, isc.normal);
        if (wi.z * wo.z <= 0.0)
            return 0.0f;
        if (cosine_sampling)
            return std::abs(wo.z) * InvPi;
        else  // uniform hemispher sammpling
            return Inv2Pi;
    }

    std::pair<BSDFSample, Vec3f> sample(const Intersection &isc, Float sample1, const Vec2f &sample2) const override {
        Vec3f wi = worldToLocal(isc.dirn, isc.normal);
        if (wi.z <= 0.0 && !this->has_flag(BSDFFlags::TwoSided))
            return {BSDFSample{Vec3f{0.0}, 0.0, 1}, Vec3f{0.0}};
        
        if (cosine_sampling) {
            Vec3f wo = cosineHemisphereSample(sample2);
            if (wi.z < 0.0 && this->has_flag(BSDFFlags::TwoSided))
                wo.z = -wo.z;

            return {BSDFSample{wo, pdf(isc, wo), 1},
                    eval(isc, wo)};
        } else {  // uniform hemisphere sampling
            Vec3f wo = uniformHemisphereSample(sample2);
            if (wi.z < 0.0 && this->has_flag(BSDFFlags::TwoSided))
                wo.z = -wo.z;

            return {BSDFSample{wo, pdf(isc, wo), 1},
                    eval(isc, wo)};
        }
    }

    std::string to_string() const override {
        std::ostringstream oss;
        oss << "BSDF(Diffuse): [ reflectance=" << reflectance << " ]";
        return oss.str();
    }
};

// --------------------------- Registry functions ---------------------------
BSDF *createDiffuseBSDF(const std::unordered_map<std::string, std::string> &properties, const std::unordered_map<std::string, const Texture*>& textures) {
    const Texture *reflectance = TextureRegistry::createTexture("constant", {});
    bool cosine_sampling = true;
    BSDFFlags flags = BSDFFlags::None;

    for (const auto &[key, value] : properties) {
        if (key == "reflectance") {
            delete reflectance;
            reflectance = TextureRegistry::createTexture("constant", {{"albedo", value}});
        } else if (key == "cosine_sampling") {
            cosine_sampling = (value == "true");
        } else if (key == "twosided") {
            flags = BSDFFlags::TwoSided;
        } else {
            throw std::runtime_error("Unknown property '" + key + "' for Diffuse BSDF");
        }
    }

    for (const auto &[key, tex_ptr] : textures) {
        if (key == "reflectance") {
            delete reflectance;
            reflectance = tex_ptr;
        } else {
            throw std::runtime_error("Unknown texture slot '" + key + "' for Diffuse BSDF");
        }
    }

    return new DiffuseBSDF(flags, reflectance, cosine_sampling);
}

namespace {
struct DiffuseBSDFRegistrar {
    DiffuseBSDFRegistrar() {
        BSDFRegistry::registerBSDF("diffuse", createDiffuseBSDF);
    }
};

static DiffuseBSDFRegistrar registrar;
}  // namespace
