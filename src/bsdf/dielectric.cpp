#include "core/BSDF.h"
#include "core/MathUtils.h"
#include "core/Registry.h"

class SmoothDielectricBSDF final : public BSDF {
public:
    Float eta;  // ext_ior / int_ior

    SmoothDielectricBSDF(BSDFFlags flags, Float eta) : BSDF(flags), eta(eta) {}

    Vec3f eval(const Vec3f &wi, const Vec3f &wo) const override {
        return Vec3f{0.0};
    }

    Float pdf(const Vec3f &wi, const Vec3f &wo) const override {
        return 0.0;
    }

    std::pair<BSDFSample, Vec3f> sample(const Vec3f &wi, Float sample1, const Vec2f &sample2) const override {
        // FIXME: total internal reflection value&pdf wrong
        // FIXME: bug in bsdf value return (and maybe pdf). Especially when mixed with total internal reflection
        Float cos_theta_i = wi.z;
        Float effective_normal_sign = cos_theta_i >= 0 ? 1.0 : -1.0;
        Vec3f effective_normal = cos_theta_i >= 0 ? Vec3f{0, 0, 1} : Vec3f{0, 0, -1};
        Float effective_eta = cos_theta_i >= 0 ? eta : 1.0 / eta;

        Vec3f refracted_dirn;
        bool is_refracted = refract(wi, effective_normal, effective_eta, refracted_dirn);
        if (!is_refracted) {
            // total internal reflection
            return {BSDFSample{reflect(wi, effective_normal), 1.0, 1.0, effective_normal_sign},
                    Vec3f{1.0}};
        }

        Float fr = BSDF::fresnelReflection(std::abs(cos_theta_i), effective_eta);
        if (sample1 <= fr) {  // reflection
            return {BSDFSample{reflect(wi, effective_normal), fr, 1.0, effective_normal_sign},
                    Vec3f{fr}};
        } else {  // refraction
            // TODO: should be changed when transferring importance instead of radiance
            // TODO: is the returned BSDF value correct???
            return {BSDFSample{refracted_dirn, Float(1.0) - fr, effective_eta, -effective_normal_sign},
                    Vec3f{Sqr(effective_eta) * (Float(1.0) - fr)}};
        }
    }

    std::string to_string() const override {
        std::ostringstream oss;
        oss << "BSDF(SmoothDielectric): [ eta=" << eta << " ]";
        return oss.str();
    }
};

// --------------------------- Registry functions ---------------------------
BSDF *createDielectricBSDF(const std::unordered_map<std::string, std::string> &properties) {
    Float int_ior = 1.5046;    // bk27
    Float ext_ior = 1.000277;  // air

    auto it = properties.find("int_ior");
    if (it != properties.end())
        int_ior = Float(std::stod(it->second));

    it = properties.find("ext_ior");
    if (it != properties.end())
        ext_ior = Float(std::stod(it->second));

    return new SmoothDielectricBSDF(BSDFFlags::Delta, ext_ior / int_ior);
}

namespace {
struct DielectricBSDFRegistrar {
    DielectricBSDFRegistrar() {
        BSDFRegistry::registerBSDF("dielectric", createDielectricBSDF);
    }
};

static DielectricBSDFRegistrar registrar;
}  // namespace
