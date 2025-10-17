#include "core/Texture.h"
#include "core/Registry.h"
#include "utils/Misc.h"

class ConstantTexture : public Texture {
private:
    Vec3f albedo;

public:
    ConstantTexture(const Vec3f &albedo) : albedo(albedo) {}

    Vec3f eval(const Intersection &isc) const override {
        return albedo;
    }

    Float mean() const override {
        return luminance(albedo);
    }
};

// --------------------------- Registry functions ---------------------------
Texture *createConstantTexture(const std::unordered_map<std::string, std::string> &properties) {
    Vec3f albedo = Vec3f{0.5};

    for (const auto &[key, value] : properties) {
        if (key == "albedo") {
            albedo = strToVec3f(properties.at("albedo"));
        } else {
            throw std::runtime_error("Unknown property '" + key + "' for Constant Texture");
        }
    }

    return new ConstantTexture(albedo);
}

namespace {
struct ConstantTextureRegistrar {
    ConstantTextureRegistrar() {
        TextureRegistry::registerTexture("constant", createConstantTexture);
    }
};

static ConstantTextureRegistrar registrar;
}  // namespace
