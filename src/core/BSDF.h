#pragma once

#include <array>
#include <stdexcept>

#include "core/Pacific.h"
#include "core/MathUtils.h"

class BSDF {
public:
    virtual Vec2f sample(Float theta_i) = 0;
    virtual Float eval(Float theta_i, Float theta_o) = 0;
    virtual Float pdf(Float theta_i, Float theta_o) = 0;
    virtual std::string to_string() const = 0;

};

// Texture reflectance not supported
class DiffuseBSDF final : public BSDF {
public:
    Vec3f reflectance = Vec3f{0.5};

    DiffuseBSDF(Vec3f reflectance) : reflectance(reflectance) {}

    Vec2f sample(Float theta_i) override {}
    Float eval(Float theta_i, Float theta_o) override {}
    Float pdf(Float theta_i, Float theta_o) override {}

    std::string to_string() const override {
        std::ostringstream oss;
        oss << "BSDF(Diffuse): [ reflectance=" << reflectance << " ]";
        return oss.str();
     }
};

class SmoothDielectricBSDF final : public BSDF {
public:
    Float int_ior = 1.5046;    // bk27
    Float ext_ior = 1.000277;  // air

    SmoothDielectricBSDF(Float int_ior, Float ext_ior) : int_ior(int_ior), ext_ior(ext_ior) {}

    Vec2f sample(Float theta_i) override {}
    Float eval(Float theta_i, Float theta_o) override {}
    Float pdf(Float theta_i, Float theta_o) override {}
    std::string to_string() const override {
        std::ostringstream oss;
        oss << "BSDF(SmoothDielectric): [ int_ior=" << int_ior << ", ext_ior=" << ext_ior << " ]";
        return oss.str();
    }
};

class RoughDielectricBSDF final : public BSDF {
public:
    Float int_ior = 1.5046;    // bk27
    Float ext_ior = 1.000277;  // air
    Float alpha_u = 0.1;
    Float alpha_v = 0.1;
    std::string distribution = "beckmann";

    RoughDielectricBSDF(Float int_ior, Float ext_ior, Float alpha_u, Float alpha_v) : int_ior(int_ior), ext_ior(ext_ior), alpha_u(alpha_u), alpha_v(alpha_v) {}

    Vec2f sample(Float theta_i) override {}
    Float eval(Float theta_i, Float theta_o) override {}
    Float pdf(Float theta_i, Float theta_o) override {}
    std::string to_string() const override { throw std::runtime_error("to_string not implemented"); }
};

class SmoothConductorBSDF final : public BSDF {
public:
    Vec3f eta = Vec3f{0.200, 0.924, 1.102};  // silver
    Vec3f k = Vec3f{3.912, 2.452, 2.142};    // silver

    SmoothConductorBSDF(Vec3f eta, Vec3f k) : eta(eta), k(k) {}

    Vec2f sample(Float theta_i) override {}
    Float eval(Float theta_i, Float theta_o) override {}
    Float pdf(Float theta_i, Float theta_o) override {}
    std::string to_string() const override { throw std::runtime_error("to_string not implemented"); }
};

class RoughConductorBSDF final : public BSDF {
public:
    Vec3f eta = Vec3f{0.200, 0.924, 1.102};  // silver
    Vec3f k = Vec3f{3.912, 2.452, 2.142};    // silver
    Float alpha_u = 0.1;
    Float alpha_v = 0.1;
    std::string distribution = "beckmann";

    RoughConductorBSDF(Vec3f eta, Vec3f k, Float alpha_u, Float alpha_v) : eta(eta), k(k), alpha_u(alpha_u), alpha_v(alpha_v) {}

    Vec2f sample(Float theta_i) override {}
    Float eval(Float theta_i, Float theta_o) override {}
    Float pdf(Float theta_i, Float theta_o) override {}
    std::string to_string() const override { throw std::runtime_error("to_string not implemented"); }
};

class SmoothPlasticBSDF final : public BSDF {
public:
    Float int_ior = 1.5046;    // bk27
    Float ext_ior = 1.000277;  // air
    Vec3f reflectance = Vec3f{0.04};

    SmoothPlasticBSDF(Float int_ior, Float ext_ior, Vec3f reflectance) : int_ior(int_ior), ext_ior(ext_ior), reflectance(reflectance) {}

    Vec2f sample(Float theta_i) override {}
    Float eval(Float theta_i, Float theta_o) override {}
    Float pdf(Float theta_i, Float theta_o) override {}
    std::string to_string() const override { throw std::runtime_error("to_string not implemented"); }
};

class RoughPlasticBSDF final : public BSDF {
public:
    Float int_ior = 1.5046;    // bk27
    Float ext_ior = 1.000277;  // air
    Vec3f reflectance = Vec3f{0.04};
    Float alpha_u = 0.1;
    Float alpha_v = 0.1;
    std::string distribution = "beckmann";

    RoughPlasticBSDF(Float int_ior, Float ext_ior, Vec3f reflectance, Float alpha_u, Float alpha_v) : int_ior(int_ior), ext_ior(ext_ior), reflectance(reflectance), alpha_u(alpha_u), alpha_v(alpha_v) {}

    Vec2f sample(Float theta_i) override {}
    Float eval(Float theta_i, Float theta_o) override {}
    Float pdf(Float theta_i, Float theta_o) override {}
    std::string to_string() const override { throw std::runtime_error("to_string not implemented"); }
};

// Not implemented yet
class DisneyBSDF final : public BSDF {
public:
    Vec3f base_color = Vec3f{0.5};
    Float subsurface = 0.0;
    Float metallic = 0.0;
    Float specular = 0.5;
    Float specular_tint = 0.0;
    Float roughness = 0.5;
    Float anisotropic = 0.0;
    Float sheen = 0.0;
    Float sheen_tint = 0.5;
    Float clearcoat = 0.0;
    Float clearcoat_gloss = 1.0;

    DisneyBSDF(Vec3f base_color, Float subsurface, Float metallic, Float specular, Float specular_tint, Float roughness, Float anisotropic, Float sheen, Float sheen_tint, Float clearcoat, Float clearcoat_gloss)
        : base_color(base_color), subsurface(subsurface), metallic(metallic), specular(specular), specular_tint(specular_tint), roughness(roughness), anisotropic(anisotropic), sheen(sheen), sheen_tint(sheen_tint), clearcoat(clearcoat), clearcoat_gloss(clearcoat_gloss) {}

    Vec2f sample(Float theta_i) override {}
    Float eval(Float theta_i, Float theta_o) override {}
    Float pdf(Float theta_i, Float theta_o) override {}
    std::string to_string() const override { throw std::runtime_error("to_string not implemented"); }
};
