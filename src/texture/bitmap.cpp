#include "core/Registry.h"
#include "core/Texture.h"
#include "utils/Misc.h"

class BitmapTexture final : public Texture {
public:

    Vec3f eval(const Intersection &isc) const override {
        throw std::runtime_error("Bitmap texture not implemented yet.");
    }
};

// --------------------------- Registry functions ---------------------------
Texture *createBitmapTexture(const std::unordered_map<std::string, std::string> &properties) {
    throw std::runtime_error("Bitmap texture not implemented yet.");
    
    std::string filename;
    std::string filter_type = "bilinear";
    std::string wrap_mode = "repeat";
    Mat3f to_uv = Mat3f{1.0};
    bool raw = false;

    for (const auto &[key, value] : properties) {
        if (key == "filename") {
        } else if (key == "filter_type") {
        } else if (key == "") {
        } else {
            throw std::runtime_error("Unknown property '" + key + "' for Bitmap Texture");
        }
    }
    if (filename.empty())
        throw std::runtime_error("Bitmap Texture requires a 'filename' property");
}

namespace {
struct BitmapTextureRegistrar {
    BitmapTextureRegistrar() {
        TextureRegistry::registerTexture("bitmap", createBitmapTexture);
    }
};

static BitmapTextureRegistrar registrar;
}  // namespace
