#include "core/BSDF.h"
#include "core/Registry.h"
#include "utils/Misc.h"

// TODO: implement two-sided diffuse
// Texture reflectance not supported
class SmoothConductorBSDF final : public BSDF {
public:
    Vec3f eta, k;  // real and imaginary parts of IOR

    SmoothConductorBSDF(BSDFFlags flags, const Vec3f &eta, const Vec3f &k) : BSDF(flags), eta(eta), k(k) {}

    Vec3f eval(const Vec3f &wi, const Vec3f &wo) const override {
        return Vec3f{0.0};
    }

    Float pdf(const Vec3f &wi, const Vec3f &wo) const override {
        return 0.0;
    }

    std::pair<BSDFSample, Vec3f> sample(const Vec3f &wi, Float sample1, const Vec2f &sample2) const override {
        // TODO: handle two-sided BSDFs
        if (wi.z <= 0.0 && !this->has_flag(BSDFFlags::TwoSided))
            return {BSDFSample{Vec3f{0.0}, 0.0, 1.0}, Vec3f{0.0}};
        Vec3f bsdf_value = fresnelComplex(std::abs(wi.z), eta, k);
        
        // TODO: is eta correct? should it be 1?
        return {BSDFSample{Vec3f{-wi.x, -wi.y, wi.z}, 1.0, 1.0}, bsdf_value};
    }

    std::string to_string() const override {
        std::ostringstream oss;
        oss << "BSDF(SmoothConductor): [ eta=" << eta << ", k=" << k << " ]";
        return oss.str();
    }
};

// --------------------------- Registry functions ---------------------------
BSDF *createSmoothConductorBSDF(const std::unordered_map<std::string, std::string> &properties) {
    // Cu(Copper)
    Vec3f eta{0.271, 0.676, 1.316};
    Vec3f k{3.609, 2.624, 2.292};
    BSDFFlags flags = BSDFFlags::Delta;
    
    for (const auto &[key, value] : properties) {
        if (key == "material") {
            throw std::runtime_error("SmoothConductorBSDF: named material not supported");
        } else if (key == "eta") {
            eta = strToVec3f(value);
        } else if (key == "k") {
            k = strToVec3f(value);
        } else if (key == "twosided" && (value == "true" || value == "1")) {
            flags |= BSDFFlags::TwoSided;
        } else {
            throw std::runtime_error("SmoothConductorBSDF: Unknown property " + key);
        }
    }

    return new SmoothConductorBSDF(flags, eta, k);
}

namespace {
struct SmoothConductorBSDFRegistrar {
    SmoothConductorBSDFRegistrar() {
        BSDFRegistry::registerBSDF("conductor", createSmoothConductorBSDF);
    }
};

static SmoothConductorBSDFRegistrar registrar;
}  // namespace
