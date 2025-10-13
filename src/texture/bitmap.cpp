#include "core/Registry.h"
#include "core/Texture.h"
#include "utils/Misc.h"

class BitmapTexture final : public Texture {
public:

};

// --------------------------- Registry functions ---------------------------
Texture *createBitmapTexture(const std::unordered_map<std::string, std::string> &properties) {
    std::string filename;

    throw std::runtime_error("Bitmap Texture not implemented yet");

    // for (const auto &[key, value] : properties) {
    //     if () {
    //     } else {
    //         throw std::runtime_error("Unknown property '" + key + "' for Constant Texture");
    //     }
    // }

}

namespace {
struct BitmapTextureRegistrar {
    BitmapTextureRegistrar() {
        TextureRegistry::registerTexture("bitmap", createBitmapTexture);
    }
};

static BitmapTextureRegistrar registrar;
}  // namespace
