#include <stdexcept>

#include "core/Registry.h"
#include "core/Scene.h"
#include "core/Texture.h"

// --------------------------- BSDFRegistry Implementation -----------------------
void BSDFRegistry::registerBSDF(const std::string& type, BSDFCreator creator) {
    BSDFRegistry::getCreators()[type] = creator;
}
BSDF* BSDFRegistry::createBSDF(const std::string& type, const std::unordered_map<std::string, std::string>& properties, const std::unordered_map<std::string, const Texture*>& textures) {
    auto it = BSDFRegistry::getCreators().find(type);
    if (it == BSDFRegistry::getCreators().end()) {
        throw std::runtime_error("Unknown BSDF type: " + type);
    }
    return it->second(properties, textures);
}
std::vector<std::string> BSDFRegistry::getRegisteredTypes() {
    std::vector<std::string> types;
    for (const auto& pair : BSDFRegistry::getCreators()) {
        types.push_back(pair.first);
    }
    return types;
}

// ------------------------ IntegratorRegistry Implementation ---------------------
void IntegratorRegistry::registerIntegrator(const std::string& type, IntegratorRegistry::IntegratorCreator creator) {
    IntegratorRegistry::getCreators()[type] = creator;
}
Integrator* IntegratorRegistry::createIntegrator(const std::string& type, const std::unordered_map<std::string, std::string>& properties) {
    auto it = IntegratorRegistry::getCreators().find(type);
    if (it == IntegratorRegistry::getCreators().end()) {
        throw std::runtime_error("Unknown Integrator type: " + type);
    }
    return it->second(properties);
}
std::vector<std::string> IntegratorRegistry::getRegisteredTypes() {
    std::vector<std::string> types;
    for (const auto& pair : IntegratorRegistry::getCreators()) {
        types.push_back(pair.first);
    }
    return types;
}

// -------------------------- EmitterRegistry Implementation -----------------------
void EmitterRegistry::registerEmitter(const std::string& type, EmitterRegistry::EmitterCreator creator) {
    EmitterRegistry::getCreators()[type] = creator;
}
Emitter* EmitterRegistry::createEmitter(const std::string& type, const std::unordered_map<std::string, std::string>& properties, const std::unordered_map<std::string, const Texture*>& textures) {
    auto it = EmitterRegistry::getCreators().find(type);
    if (it == EmitterRegistry::getCreators().end()) {
        throw std::runtime_error("Unknown Emitter type: " + type);
    }
    return it->second(properties, textures);
}
std::vector<std::string> EmitterRegistry::getRegisteredTypes() {
    std::vector<std::string> types;
    for (const auto& pair : EmitterRegistry::getCreators()) {
        types.push_back(pair.first);
    }
    return types;
}

// -------------------------- RFilterRegistry Implementation -----------------------
void RFilterRegistry::registerRFilter(const std::string& type, RFilterRegistry::RFilterCreator creator) {
    RFilterRegistry::getCreators()[type] = creator;
}
RFilter* RFilterRegistry::createRFilter(const std::string& type, const std::unordered_map<std::string, std::string>& properties) {
    auto it = RFilterRegistry::getCreators().find(type);
    if (it == RFilterRegistry::getCreators().end()) {
        throw std::runtime_error("Unknown RFilter type: " + type);
    }
    return it->second(properties);
}
std::vector<std::string> RFilterRegistry::getRegisteredTypes() {
    std::vector<std::string> types;
    for (const auto& pair : RFilterRegistry::getCreators()) {
        types.push_back(pair.first);
    }
    return types;
}

// -------------------------- GeometryRegistry Implementation -----------------------
void GeometryRegistry::registerGeometry(const std::string& type, GeometryRegistry::GeometryCreator creator) {
    GeometryRegistry::getCreators()[type] = creator;
}
Geometry* GeometryRegistry::createGeometry(const std::string& type, const std::unordered_map<std::string, std::string>& properties, const Shape *parent_shape, const GeometryCreationContext* ctx) {
    auto it = GeometryRegistry::getCreators().find(type);
    if (it == GeometryRegistry::getCreators().end()) {
        throw std::runtime_error("Unknown Geometry type: " + type);
    }
    return it->second(properties, parent_shape, ctx);
}
std::vector<std::string> GeometryRegistry::getRegisteredTypes() {
    std::vector<std::string> types;
    for (const auto& pair : GeometryRegistry::getCreators()) {
        types.push_back(pair.first);
    }
    return types;
}

// --------------------------- MicrofacetRegistry Implementation -----------------------
void MicrofacetRegistry::registerMicrofacet(const std::string& type, MicrofacetCreator creator) {
    MicrofacetRegistry::getCreators()[type] = creator;
}
Microfacet* MicrofacetRegistry::createMicrofacet(const std::string& type, const std::unordered_map<std::string, std::string>& properties) {
    auto it = MicrofacetRegistry::getCreators().find(type);
    if (it == MicrofacetRegistry::getCreators().end()) {
        throw std::runtime_error("Unknown Microfacet type: " + type);
    }
    return it->second(properties);
}
std::vector<std::string> MicrofacetRegistry::getRegisteredTypes() {
    std::vector<std::string> types;
    for (const auto& pair : MicrofacetRegistry::getCreators()) {
        types.push_back(pair.first);
    }
    return types;
}

// --------------------------- TextureRegistry Implementation -----------------------
void TextureRegistry::registerTexture(const std::string& type, TextureCreator creator) {
    TextureRegistry::getCreators()[type] = creator;
}
Texture* TextureRegistry::createTexture(const std::string& type, const std::unordered_map<std::string, std::string>& properties) {
    auto it = TextureRegistry::getCreators().find(type);
    if (it == TextureRegistry::getCreators().end()) {
        throw std::runtime_error("Unknown Texture type: " + type);
    }
    return it->second(properties);
}
std::vector<std::string> TextureRegistry::getRegisteredTypes() {
    std::vector<std::string> types;
    for (const auto& pair : TextureRegistry::getCreators()) {
        types.push_back(pair.first);
    }
    return types;
}
