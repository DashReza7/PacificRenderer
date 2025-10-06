#include "core/BSDF.h"
#include "core/Registry.h"
#include "utils/Misc.h"

// TODO: implement two-sided diffuse
// Texture reflectance not supported
class DiffuseBSDF final : public BSDF {
public:
    Vec3f reflectance;
    bool cosine_sampling;

    DiffuseBSDF(BSDFFlags flags, const Vec3f &reflectance, bool cosine_sampling) : BSDF(flags), reflectance(reflectance), cosine_sampling(cosine_sampling) {}

    Vec3f eval(const Vec3f &wi, const Vec3f &wo) const override {
        // TODO: what about two-sided BSDFs
        if (wi.z * wo.z <= 0.0)
            return Vec3f{0.0};
        return InvPi * std::abs(wo.z) * reflectance;
    }

    Float pdf(const Vec3f &wi, const Vec3f &wo) const override {
        if (wi.z * wo.z <= 0.0)
            return 0.0f;
        if (cosine_sampling)
            return std::abs(wo.z) * InvPi;
        else  // uniform hemispher sammpling
            return Inv2Pi;
    }

    std::pair<BSDFSample, Vec3f> sample(const Vec3f &wi, Float sample1, const Vec2f &sample2) const override {
        // TODO: what about the two-sided BSDF
        if (cosine_sampling) {
            Vec3f wo = cosineHemisphereSample(sample2);
            if (wi.z < 0.0 && this->has_flag(BSDFFlags::TwoSided))
                wo.z = -wo.z;
            
            return {BSDFSample{wo, pdf(wi, wo), 1},
                    eval(wi, wo)};
        } else {  // uniform hemisphere sampling
            Vec3f wo = uniformHemisphereSample(sample2);
            if (wi.z < 0.0 && this->has_flag(BSDFFlags::TwoSided))
                wo.z = -wo.z;
            
            return {BSDFSample{wo, pdf(wi, wo), 1},
                    eval(wi, wo)};
        }
    }

    std::string to_string() const override {
        std::ostringstream oss;
        oss << "BSDF(Diffuse): [ reflectance=" << reflectance << " ]";
        return oss.str();
    }
};

// --------------------------- Registry functions ---------------------------
BSDF *createDiffuseBSDF(const std::unordered_map<std::string, std::string> &properties) {
    Vec3f reflectance{0.5, 0.5, 0.5};
    bool cosine_sampling = true;

    auto it = properties.find("reflectance");
    if (it != properties.end())
        reflectance = strToVec3f(it->second);

    it = properties.find("cosine_sampling");
    if (it != properties.end())
        cosine_sampling = (it->second == "true");

    BSDFFlags flags = BSDFFlags::None;
    it = properties.find("two_sided");
    if (it != properties.end() && it->second == "true")
        flags = BSDFFlags::TwoSided;

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
