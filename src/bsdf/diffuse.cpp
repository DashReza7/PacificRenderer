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
        if (wi.z <= 0 || wo.z <= 0)
            return Vec3f{0.0};
        return reflectance * InvPi * wo.z;
    }

    Float pdf(const Vec3f &wi, const Vec3f &wo) const override {
        if (wi.z <= 0 || wo.z <= 0)
            return 0.0f;
        if (cosine_sampling)
            return wo.z * InvPi;
        else  // uniform hemispher sammpling
            return Inv2Pi;
    }

    std::pair<BSDFSample, Vec3f> sample(const Vec3f &wi, Float sample1, const Vec2f &sample2) const override {
        if (cosine_sampling) {
            Vec3f wo = cosineHemisphereSample(sample2);
            return {BSDFSample{wo, pdf(Vec3f{0, 0, 1}, wo), 1, 1.0},
                    eval(Vec3f{0, 0, 1}, wo)};
        } else {  // uniform hemisphere sampling
            Vec3f wo = uniformHemisphereSample(sample2);
            return {BSDFSample{wo, pdf(Vec3f{0, 0, 1}, wo), 1, 1.0},
                    eval(Vec3f{0, 0, 1}, wo)};
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

    return new DiffuseBSDF(BSDFFlags::Diffuse, reflectance, cosine_sampling);
}

namespace {
struct DiffuseBSDFRegistrar {
    DiffuseBSDFRegistrar() {
        BSDFRegistry::registerBSDF("diffuse", createDiffuseBSDF);
    }
};

static DiffuseBSDFRegistrar registrar;
}  // namespace
