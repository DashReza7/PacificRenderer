#include "core/BSDF.h"
#include "core/MathUtils.h"
#include "core/Registry.h"
#include "core/Constants.h"

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
        Float cos_theta_i = wi.z;
        Vec3f effective_normal = cos_theta_i >= 0 ? Vec3f{0, 0, 1} : Vec3f{0, 0, -1};
        // eta_t / eta_i
        Float effective_eta = cos_theta_i >= 0 ? 1.0 / eta : eta;

        Vec3f refracted_dirn;
        bool is_refracted = refract(wi, effective_normal, effective_eta, refracted_dirn);
        if (!is_refracted) {
            // total internal reflection
            return {BSDFSample{reflect(wi, effective_normal), 1.0, 1.0},
                    Vec3f{1.0}};
        }

        Float fr = BSDF::fresnelReflection(std::abs(cos_theta_i), effective_eta);
        if (sample1 <= fr) {  // reflection
            return {BSDFSample{reflect(wi, effective_normal), fr, 1.0},
                    Vec3f{fr}};
        } else {  // refraction
            // FIXME: should be changed when transferring importance instead of radiance
            return {BSDFSample{refracted_dirn, Float(1.0) - fr, Float(1.0) / effective_eta},
                    Vec3f{(Float(1.0) - fr) / effective_eta}};
        }
    }

    std::string to_string() const override {
        std::ostringstream oss;
        oss << "BSDF(SmoothDielectric): [ eta=" << eta << " ]";
        return oss.str();
    }
};

// --------------------------- Registry functions ---------------------------
BSDF *createSmoothDielectricBSDF(const std::unordered_map<std::string, std::string> &properties) {
    Float int_ior = 1.5046;    // bk27
    Float ext_ior = 1.000277;  // air

    for (const auto &[key, value] : properties) {
        if (key == "int_ior") {
            try {
                int_ior = std::stod(value);
            } catch (const std::invalid_argument &) {
                if (!IOR_TABLE.contains(value))
                    throw std::runtime_error("Unknown material '" + value + "' for SmoothDielectric BSDF");
                int_ior = IOR_TABLE.at(value);
            }
        } else if (key == "ext_ior") {
            try {
                ext_ior = std::stod(value);
            } catch (const std::invalid_argument &) {
                if (!IOR_TABLE.contains(value))
                    throw std::runtime_error("Unknown material '" + value + "' for SmoothDielectric BSDF");
                ext_ior = IOR_TABLE.at(value);
            }

        } else {
            throw std::runtime_error("Unknown property '" + key + "' for SmoothDielectric BSDF");
        }
    }

    return new SmoothDielectricBSDF(BSDFFlags::Delta, ext_ior / int_ior);
}

namespace {
struct SmoothDielectricBSDFRegistrar {
    SmoothDielectricBSDFRegistrar() {
        BSDFRegistry::registerBSDF("dielectric", createSmoothDielectricBSDF);
    }
};

static SmoothDielectricBSDFRegistrar registrar;
}  // namespace
