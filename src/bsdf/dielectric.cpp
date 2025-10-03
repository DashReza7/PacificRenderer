#include "core/BSDF.h"
#include "core/MathUtils.h"
#include "core/Registry.h"


class SmoothDielectricBSDF final : public BSDF {
public:
    Float eta;  // ext_ior/int_ior

    SmoothDielectricBSDF(Float int_ior = 1.5046 /* bk27 */, Float ext_ior = 1.000277 /* air */) : eta(ext_ior / int_ior) {}

    /// @brief Compute the Fresnel reflection coefficient
    /// @param cos_theta_i Cosine of the angle between the incident direction and the normal. Should be positive.
    /// @param eta Relative ior (ext_ior/int_ior)
    static Float fresnelReflection(Float cos_theta_i, Float eta) {
        // Snell's law
        Float sin2_theta_t = (1 - Sqr(cos_theta_i)) / Sqr(eta);
        if (sin2_theta_t >= 1)  // total internal reflection
            return 1.0;
        Float cos_theta_t = glm::clamp(Float(std::sqrt(std::abs(1.0 - sin2_theta_t))), Float(0.0), Float(1.0));

        Float r_parl = (eta * cos_theta_i - cos_theta_t) / (eta * cos_theta_i + cos_theta_t);
        Float r_perp = (cos_theta_i - eta * cos_theta_t) / (cos_theta_i + eta * cos_theta_t);
        return (Sqr(r_parl) + Sqr(r_perp)) * 0.5;
    }

    std::pair<BSDFSample, Vec3f> sample(const Intersection &isc, Float sample1, const Vec2f &sample2) const override {
        Float cos_theta_i = dot(isc.dirn, isc.normal);
        Vec3f effective_normal = cos_theta_i >= 0 ? isc.normal : -isc.normal;
        Float effective_eta = cos_theta_i >= 0 ? eta : 1.0 / eta;
        Float fr = SmoothDielectricBSDF::fresnelReflection(std::abs(cos_theta_i), effective_eta);
        if (sample1 <= fr)  // reflection
            return {BSDFSample{reflect(isc.dirn, effective_normal), fr, 1.0},
                    Vec3f{1.0}};
        else {  // refraction
            // return {BSDFSample{refract(isc.dirn, effective_normal, effective_eta), Float(1.0) - fr, effective_eta},
            //         Vec3f{1.0}};
            Vec3f refracted_dirn = refract(isc.dirn, effective_normal, effective_eta);
            if (glm::length(refracted_dirn) <= Epsilon) {
                // total internal reflection
                return {BSDFSample{reflect(isc.dirn, effective_normal), 1.0, 1.0},
                        Vec3f{1.0}};
            }

            // Scale by eta^2 for proper radiance transport
            Float eta_scale = (cos_theta_i >= 0) ? Sqr(eta) : 1.0 / Sqr(eta);
            return {BSDFSample{refracted_dirn, Float(1.0) - fr, effective_eta},
                    Vec3f{eta_scale}};
        }
    }

    Vec3f eval(const Vec3f &wi, const Vec3f &wo) const override {
        return Vec3f{0.0};
    }

    Float pdf(const Vec3f &wi, const Vec3f &wo) const override {
        return 0.0;
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

    return new SmoothDielectricBSDF(int_ior, ext_ior);
}

namespace {
struct DielectricBSDFRegistrar {
    DielectricBSDFRegistrar() {
        BSDFRegistry::registerBSDF("dielectric", createDielectricBSDF);
    }
};

static DielectricBSDFRegistrar registrar;
}  // namespace
