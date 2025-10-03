#include "core/BSDF.h"
#include "core/Registry.h"
#include "utils/Misc.h"

// Texture reflectance not supported
class DiffuseBSDF final : public BSDF {
public:
    Vec3f reflectance;
    bool cosine_sampling;

    DiffuseBSDF(const Vec3f &reflectance, bool cosine_sampling) : reflectance(reflectance), cosine_sampling(cosine_sampling) {}

    std::pair<BSDFSample, Vec3f> sample(const Intersection &isc, Float sample1, const Vec2f &sample2) const override {
        if (cosine_sampling) {
            Vec3f wo_local = cosineHemisphereSample(sample2);
            // diffuse pdf&eval are independent of wi
            return {BSDFSample{localToWorld(wo_local, isc.normal),
                               pdf(Vec3f{0.0, 0.0, 1.0}, wo_local),
                               1.0},
                    eval(Vec3f{0.0, 0.0, 1.0}, wo_local)};
        } else {  // uniform hemisphere sampling
            Vec3f wo_local = uniformHemisphereSample(sample2);
            // diffuse pdf&eval are independent of wi
            return {BSDFSample{localToWorld(wo_local, isc.normal),
                               pdf(Vec3f{0.0, 0.0, 1.0}, wo_local),
                               1.0},
                    eval(Vec3f{0.0, 0.0, 1.0}, wo_local)};
        }
    }

    Vec3f eval(const Vec3f &wi, const Vec3f &wo) const override {
        if (wi.z <= 0 || wo.z <= 0)
            return Vec3f{0.0f};
        return reflectance * InvPi * wo.z;
    }

    Float pdf(const Vec3f &wi, const Vec3f &wo) const override {
        if (wi.z <= 0 || wo.z <= 0)
            return 0.0f;
        if (cosine_sampling)
            return wo.z * InvPi;
        else
            return Inv2Pi;
    }

    std::string to_string() const override {
        std::ostringstream oss;
        oss << "BSDF(Diffuse): [ reflectance=" << reflectance << " ]";
        return oss.str();
    }
};

// --------------------------- Registry functions ---------------------------
BSDF *createDiffuseBSDF(const std::unordered_map<std::string, std::string> &properties) {
    Vec3f reflectance{0.5f, 0.5f, 0.5f};
    bool cosine_sampling = true;

    auto it = properties.find("reflectance");
    if (it != properties.end())
        reflectance = strToVec3f(it->second);

    it = properties.find("cosine_sampling");
    if (it != properties.end())
        cosine_sampling = (it->second == "true");

    return new DiffuseBSDF(reflectance, cosine_sampling);
}

namespace {
struct DiffuseBSDFRegistrar {
    DiffuseBSDFRegistrar() {
        BSDFRegistry::registerBSDF("diffuse", createDiffuseBSDF);
    }
};

static DiffuseBSDFRegistrar registrar;
}  // namespace
