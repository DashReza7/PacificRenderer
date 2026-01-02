#include "core/Emitter.h"
#include "core/Registry.h"
#include "utils/Misc.h"
#include "core/Scene.h"
#include "core/Bitmap.h"


class EnvmapLight final : public Emitter {
public:
    Float scale;
    Mat4f to_world, inv_to_world;
    Bitmap bitmap;

    EnvmapLight(const std::string &filename, Float scale, const Mat4f &to_world, const Mat4f &inv_to_world) : scale(scale), to_world(to_world), inv_to_world(inv_to_world) {
        loadBitmap(filename, false, bitmap);
    }

    
    virtual Vec3f eval(const Intersection &isc) const override {
        Vec3f w = Vec3f{inv_to_world * Vec4f{isc.dirn, 0.0}};
        Vec2f uv{std::atan2(w.x, -w.z) * Inv2Pi, std::acos(std::clamp(w.y, Float(-1.0), Float(1.0))) * InvPi};

        uv.x = uv.x - std::floor(uv.x);
        uv.y = uv.y - std::floor(uv.y);
        int u = static_cast<int>(uv.x * bitmap.width) % bitmap.width;
        int v = static_cast<int>(uv.y * bitmap.height) % bitmap.height;
        return bitmap(u, v) * scale;        
    }

    EmitterSample sampleLi(const Scene *scene, const Intersection &isc, const Vec3f &sample) const override {
        Vec3f w = uniformSphereSample(Vec2f{sample.y, sample.z});
        // check for occlusion
        Ray shadow_ray{isc.position + sign(glm::dot(isc.normal, w)) * isc.normal * Epsilon, w, Epsilon, 1e4, true};
        Intersection tmp_isc{};
        bool is_valid = !scene->ray_intersect(shadow_ray, tmp_isc);
        tmp_isc.dirn = w;

        return EmitterSample{Inv4Pi, -w, is_valid, eval(tmp_isc), EmitterFlags::NONE};
    }

    Vec3f sampleLe(const Vec2f &sample1, const Vec3f &sample2, Vec3f &posn, Vec3f &dirn, Float &pdf) const override {
        throw std::runtime_error("Envmap light does not support sampleLe() yet.");
    }
    
    std::string to_string() const override {
        std::ostringstream oss;
        oss << "Emitter(EnvmapLight): []";
        return oss.str();
    }
};

// --------------------------- Registry functions ---------------------------
Emitter *createEnvmapLight(const std::unordered_map<std::string, std::string> &properties, const std::unordered_map<std::string, const Texture*>& textures) {
    std::string filename;
    Float scale = 1.0;
    Mat4f to_world = Mat4f{1.0}, inv_to_world = Mat4f{1.0};

    for (const auto &[key, value] : properties) {
        if (key == "filename") {
            filename = (scene_file_path.parent_path() / value).string();
        } else if (key == "scale") {
            scale = std::stod(value);
        } else if (key == "to_world") {
            if (!properties.contains("inv_to_world"))
                throw std::runtime_error("Envmap Light emitter requires 'inv_to_world' property");
            to_world = strToMat4f(value);
        } else if (key == "inv_to_world") {
            if (!properties.contains("to_world"))
                throw std::runtime_error("Envmap Light emitter requires 'to_world' property");
            inv_to_world = strToMat4f(value);
        } else {
            throw std::runtime_error("Unknown property '" + key + "' for Envmap Light emitter");
        }
    }
    if (filename.empty())
        throw std::runtime_error("Envmap Light emitter requires 'filename' property");

    return new EnvmapLight{filename, scale, to_world, inv_to_world};
}

namespace {
struct EnvmapLightRegistrar {
    EnvmapLightRegistrar() {
        EmitterRegistry::registerEmitter("envmap", createEnvmapLight);
    }
};

static EnvmapLightRegistrar registrar;
}  // namespace
