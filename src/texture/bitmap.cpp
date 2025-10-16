#include <variant>
#include <algorithm>

#include "ImfArray.h"
#include "ImfRgbaFile.h"
#include "core/Registry.h"
#include "core/Texture.h"
#include "utils/Misc.h"
#include "stb_image.h"
#include "core/Scene.h"
#include "core/Bitmap.h"

class BitmapTexture final : public Texture {
private:
    Bitmap bitmap{};
    const std::string filter_type;
    const std::string wrap_mode;
    const Mat4f to_uv;

public:
    BitmapTexture(const std::string &filename, const std::string &filter_type, const std::string &wrap_mode, bool raw, const Mat4f to_uv) : filter_type(filter_type), wrap_mode(wrap_mode), to_uv(to_uv) {
        if (filter_type != "nearest")
            throw std::runtime_error("Unsupported filter_type '" + filter_type + "' for Bitmap Texture");
                
        loadBitmap(filename, raw, bitmap);
    }

    Vec3f eval(const Intersection &isc) const override {
        Vec2f uv = isc.geom->get_uv(isc.position);
        uv = Vec2f{to_uv * Vec4f{uv, 0.0, 1.0}};
        
        if (wrap_mode == "repeat") {
            uv.x = uv.x - std::floor(uv.x);
            uv.y = uv.y - std::floor(uv.y);
        } else if (wrap_mode == "clamp") {
            uv.x = std::clamp(uv.x, Float(0.0), Float(1.0));
            uv.y = std::clamp(uv.y, Float(0.0), Float(1.0));
        } else if (wrap_mode == "mirror") {
            bool mirror_x = (static_cast<int>(std::floor(uv.x)) % 2 == 1);
            bool mirror_y = (static_cast<int>(std::floor(uv.y)) % 2 == 1);
            uv.x = uv.x - std::floor(uv.x);
            uv.y = uv.y - std::floor(uv.y);
            if (mirror_x)
                uv.x = 1.0 - uv.x;
            if (mirror_y)
                uv.y = 1.0 - uv.y;
        } else {
            throw std::runtime_error("Unsupported wrap_mode '" + wrap_mode + "' for Bitmap Texture");
        }
        
        // TODO: implement bilinear filtering
        
        int u = static_cast<int>(uv.x * bitmap.width) % bitmap.width;
        int v = static_cast<int>(uv.y * bitmap.height) % bitmap.height;

        return bitmap(u, v);
    }
};

// --------------------------- Registry functions ---------------------------
Texture *createBitmapTexture(const std::unordered_map<std::string, std::string> &properties) {
    std::string filename;
    std::string filter_type = "bilinear";
    std::string wrap_mode = "repeat";
    Mat3f to_uv = Mat4f{1.0};
    bool raw = false;

    for (const auto &[key, value] : properties) {
        if (key == "filename") {
            filename = (scene_file_path.parent_path() / value).string();
        } else if (key == "filter_type") {
            filter_type = value;
        } else if (key == "wrap_mode") {
            wrap_mode = value;
        } else if (key == "to_uv") {
            to_uv = strToMat4f(value);
        } else if (key == "raw") {
            raw = (value == "true" || value == "1");
        } else {
            throw std::runtime_error("Unknown property '" + key + "' for Bitmap Texture");
        }
    }
    if (filename.empty())
        throw std::runtime_error("Bitmap Texture requires a 'filename' property");

    return new BitmapTexture{filename, filter_type, wrap_mode, raw, to_uv};
}

namespace {
struct BitmapTextureRegistrar {
    BitmapTextureRegistrar() {
        TextureRegistry::registerTexture("bitmap", createBitmapTexture);
    }
};

static BitmapTextureRegistrar registrar;
}  // namespace
