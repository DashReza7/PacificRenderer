#include "core/BSDF.h"
#include "core/MathUtils.h"
#include "core/Registry.h"
#include "core/Texture.h"

class ThinDielectricBSDF final : public BSDF {
public:
    Float eta;  // ext_ior / int_ior
    const Texture *specular_reflectance;
    const Texture *specular_transmission;

    ThinDielectricBSDF(BSDFFlags flags, Float eta, const Texture *specular_reflectance, const Texture *specular_transmission) : BSDF(flags), eta(eta), specular_reflectance(specular_reflectance), specular_transmission(specular_transmission) {}

    Vec3f eval(const Intersection &isc, const Vec3f &wo) const override {
        return Vec3f{0.0};
    }

    Float pdf(const Intersection &isc, const Vec3f &wo) const override {
        return 0.0;
    }

    std::pair<BSDFSample, Vec3f> sample(const Intersection &isc, Float sample1, const Vec2f &sample2) const override {
        Vec3f wi = worldToLocal(isc.dirn, isc.normal);
        Float cos_theta_i = wi.z;
        Vec3f effective_normal = cos_theta_i >= 0 ? Vec3f{0, 0, 1} : Vec3f{0, 0, -1};


        Float fr = fresnelReflection(std::abs(cos_theta_i), 1.0 / eta);
        if (fr >= 1.0) {
            // total internal reflection
            return {BSDFSample{reflect(wi, effective_normal), 1.0, 1.0},
                    specular_reflectance->eval(isc)};
        }
        
        fr += Sqr(1.0 - fr) * fr / (1.0 - Sqr(fr));
        if (sample1 <= fr) {  // reflection
            return {BSDFSample{reflect(wi, effective_normal), fr, 1.0},
                    Vec3f{fr} * specular_reflectance->eval(isc)};
        } else {  // transmission
            return {BSDFSample{-wi, Float(1.0) - fr, 1.0},
                    Vec3f{(Float(1.0) - fr)} * specular_transmission->eval(isc)};
        }
    }

    std::string to_string() const override {
        std::ostringstream oss;
        oss << "BSDF(ThinDielectric): [ eta(ext_ior/int_ior)=" << eta << " ]";
        return oss.str();
    }
};

// --------------------------- Registry functions ---------------------------
BSDF *createThinDielectricBSDF(const std::unordered_map<std::string, std::string> &properties, const std::unordered_map<std::string, const Texture*>& textures) {
    Float int_ior = 1.5046;    // bk27
    Float ext_ior = 1.000277;  // air
    const Texture *specular_reflectance = TextureRegistry::createTexture("constant", {{"albedo", "1, 1, 1"}});
    const Texture *specular_transmission = TextureRegistry::createTexture("constant", {{"albedo", "1, 1, 1"}});

    for (const auto &[key, value] : properties) {
        if (key == "int_ior") {
            int_ior = std::stod(value);
        } else if (key == "ext_ior") {
            ext_ior = std::stod(value);
        } else if (key == "specular_reflectance") {
            delete specular_reflectance;
            specular_reflectance = TextureRegistry::createTexture("constant", {{"albedo", value}});
        } else if (key == "specular_transmission") {
            delete specular_transmission;
            specular_transmission = TextureRegistry::createTexture("constant", {{"albedo", value}});
        } else {
            throw std::runtime_error("Unknown property '" + key + "' for ThinDielectric BSDF");
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
            throw std::runtime_error("Unknown texture slot '" + key + "' for ThinDielectric BSDF");
        }
    }

    return new ThinDielectricBSDF(BSDFFlags::Delta | BSDFFlags::PassThrough,
                ext_ior / int_ior, 
                specular_reflectance, 
                specular_transmission);
}

namespace {
struct ThinDielectricBSDFRegistrar {
    ThinDielectricBSDFRegistrar() {
        BSDFRegistry::registerBSDF("thindielectric", createThinDielectricBSDF);
    }
};

static ThinDielectricBSDFRegistrar registrar;
}  // namespace
