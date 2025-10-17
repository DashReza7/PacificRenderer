#include "core/Registry.h"
#include "core/Texture.h"
#include "utils/Misc.h"

class CheckerboardTexture final : public Texture {
private:
    const Vec3f color0;
    const Vec3f color1;
    const Mat4f to_uv;

public:
    CheckerboardTexture(const Vec3f &c0, const Vec3f &c1, const Mat4f &to_uv) : color0(c0), color1(c1), to_uv(to_uv) {}

    Vec3f eval(const Intersection &isc) const override {
        Vec2f uv = isc.geom->get_uv(isc.position);
        uv = Vec2f{to_uv * Vec4f{uv, 0.0, 1.0}};
        bool u_mask = uv.x - std::floor(uv.x) > 0.5;
        bool v_mask = uv.y - std::floor(uv.y) > 0.5;

        if (u_mask == v_mask)
            return color0;
        else
            return color1;
    }

    Float mean() const override {
        return 0.5 * (luminance(color0) + luminance(color1));
    }
};

// --------------------------- Registry functions ---------------------------
Texture *createCheckerboardTexture(const std::unordered_map<std::string, std::string> &properties) {
    Vec3f color0{0.4};
    Vec3f color1{0.2};
    Mat4f to_uv{1.0};
    
    for (const auto &[key, value] : properties) {
        if (key == "color0") {
            color0 = strToVec3f(value);
        } else if(key == "color1") {
            color1 = strToVec3f(value);
        } else if(key == "to_uv") {
            to_uv = strToMat4f(value);
        } else if (key == "inv_to_uv") {
            // ignore
        } else {
            throw std::runtime_error("Unknown property '" + key + "' for Bitmap Texture");
        }
    }

    return new CheckerboardTexture{color0, color1, to_uv};
}

namespace {
struct CheckerboardTextureRegistrar {
    CheckerboardTextureRegistrar() {
        TextureRegistry::registerTexture("checkerboard", createCheckerboardTexture);
    }
};

static CheckerboardTextureRegistrar registrar;
}  // namespace
