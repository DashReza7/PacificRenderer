#pragma once

#include <array>
#include <cmath>
#include <stdexcept>

#include "core/Geometry.h"
#include "core/MathUtils.h"
#include "core/Pacific.h"

// TODO: add an `is_valid` to BSDFSample, so that when the sample is invalid,
// like when the pdf is too small, we can just ignore it in the integrator.

/// @brief BSDFSample object.
struct BSDFSample {
    /// Outgoing direction in world space
    Vec3f wo;
    /// Probability density at the sample
    Float pdf;
    /// Relative ior (from incoming medium to outgoing medium)
    Float eta;

    BSDFSample(const Vec3f &wo, Float pdf, Float eta) : wo(wo), pdf(pdf), eta(eta) {}
};

enum class BSDFFlags {
    None = 0,
    Delta = 1 << 0,  // delta distribution, i.e. perfect specular reflection/refraction
    TwoSided = 1 << 1
};

class BSDF {
private:
    BSDFFlags flags;


public:
    BSDF(BSDFFlags flags) : flags(flags) {}

    /// @brief Compute the Fresnel reflection coefficient
    /// @param cos_theta_i Cosine of the angle between the incident direction and the (effective)normal. Should be positive.
    /// @param eta Relative ior (eta_i over eta_t)
    static Float fresnelReflection(Float cos_theta_i, Float eta) {
        // Snell's law
        Float sin2_theta_t = Sqr(eta) * (1 - Sqr(cos_theta_i));
        if (sin2_theta_t >= 1)  // total internal reflection
            return 1.0;
        Float cos_theta_t = std::sqrt(std::abs(1.0 - sin2_theta_t));

        Float inv_eta = 1.0 / eta;
        Float r_parl = (inv_eta * cos_theta_i - cos_theta_t) / (inv_eta * cos_theta_i + cos_theta_t);
        Float r_perp = (cos_theta_i - inv_eta * cos_theta_t) / (cos_theta_i + inv_eta * cos_theta_t);
        return (Sqr(r_parl) + Sqr(r_perp)) * 0.5;
    }
    
    /// @brief Sample the BSDF
    /// @param wi Incoming direction in local space (z is the normal direction). Facing the normal direction
    /// @param sample1 1D random sample in [0,1). Used for selecting multiple lobes in mixture BSDFs
    /// @param sample2 2D random sample in [0,1)^2
    /// @return BSDFSample and the BSDF value(RGB). the sample_eval accounts for the cosine foreshortening term if needed. (i.e. not in delta BSDFs)
    virtual std::pair<BSDFSample, Vec3f> sample(const Vec3f &wi, Float sample1, const Vec2f &sample2) const = 0;

    /// @brief Evaluate the BSDF
    /// @param wi Incoming direction in local space (z is the normal direction)
    /// @param wo Outgoing direction in local space (z is the normal direction)
    /// @return BSDF value. the return value accounts for the cosine foreshortening term if needed. (i.e. not in delta BSDFs)
    virtual Vec3f eval(const Vec3f &wi, const Vec3f &wo) const = 0;

    /// @brief Evaluate the PDF of the BSDF sample
    /// @param wi Incoming direction in local space (z is the normal direction)
    /// @param wo Outgoing direction in local space (z is the normal direction)
    virtual Float pdf(const Vec3f &wi, const Vec3f &wo) const = 0;

    bool has_flag(BSDFFlags flag) const {
        return (static_cast<uint32_t>(flags) & static_cast<uint32_t>(flag)) != 0;
    }
    
    virtual std::string to_string() const = 0;
};
