#include "core/BSDF.h"
#include "core/Registry.h"
#include "core/Texture.h"
#include "utils/Misc.h"


// Texture reflectance not supported
class SmoothConductorBSDF final : public BSDF {
public:
    Vec3f eta, k;  // real and imaginary parts of IOR
    const Texture *specular_reflectance;

    SmoothConductorBSDF(BSDFFlags flags, const Vec3f &eta, const Vec3f &k, const Texture *specular_reflectance) : BSDF(flags), eta(eta), k(k), specular_reflectance(specular_reflectance) {}

    Vec3f eval(const Intersection &isc, const Vec3f &wo) const override {
        return Vec3f{0.0};
    }

    Float pdf(const Intersection &isc, const Vec3f &wo) const override {
        return 0.0;
    }

    std::pair<BSDFSample, Vec3f> sample(const Intersection &isc, Float sample1, const Vec2f &sample2) const override {
        Vec3f wi = worldToLocal(isc.dirn, isc.normal);
        if (wi.z <= 0.0 && !this->has_flag(BSDFFlags::TwoSided))
            return {BSDFSample{Vec3f{0.0}, 0.0, 1.0, BSDFSampleFlags::None}, 
                    Vec3f{0.0}};
        Vec3f bsdf_value = fresnelComplex(std::abs(wi.z), eta, k);

        return {BSDFSample{Vec3f{-wi.x, -wi.y, wi.z}, 1.0, 1.0, BSDFSampleFlags::DeltaReflection}, 
                bsdf_value * specular_reflectance->eval(isc)};
    }

    std::string to_string() const override {
        std::ostringstream oss;
        oss << "BSDF(SmoothConductor): [ eta=" << eta << ", k=" << k << " ]";
        return oss.str();
    }
};

// --------------------------- Registry functions ---------------------------
BSDF *createSmoothConductorBSDF(const std::unordered_map<std::string, std::string> &properties, const std::unordered_map<std::string, const Texture *> &textures) {
    Vec3f eta{0.0, 0.0, 0.0};
    Vec3f k{1.0, 1.0, 1.0};
    BSDFFlags flags = BSDFFlags::Delta;
    const Texture *specular_reflectance = TextureRegistry::createTexture("constant", {{"albedo", "1, 1, 1"}});

    for (const auto &[key, value] : properties) {
        if (key == "material") {
            throw std::runtime_error("SmoothConductorBSDF: named material not supported");
        } else if (key == "eta") {
            eta = strToVec3f(value);
        } else if (key == "k") {
            k = strToVec3f(value);
        } else if (key == "twosided" && (value == "true" || value == "1")) {
            flags |= BSDFFlags::TwoSided;
        } else if (key == "specular_reflectance") {
            delete specular_reflectance;
            specular_reflectance = TextureRegistry::createTexture("constant", {{"albedo", value}});
        } else {
            throw std::runtime_error("SmoothConductorBSDF: Unknown property " + key);
        }
    }

    for (const auto &[key, tex_ptr] : textures) {
        if (key == "specular_reflectance") {
            delete specular_reflectance;
            specular_reflectance = tex_ptr;
        } else {
            throw std::runtime_error("Unknown texture slot '" + key + "' for SmoothConductorBSDF");
        }
    }

    return new SmoothConductorBSDF(flags, eta, k, specular_reflectance);
}

namespace {
struct SmoothConductorBSDFRegistrar {
    SmoothConductorBSDFRegistrar() {
        BSDFRegistry::registerBSDF("conductor", createSmoothConductorBSDF);
    }
};

static SmoothConductorBSDFRegistrar registrar;
}  // namespace
