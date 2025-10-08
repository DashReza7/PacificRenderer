#pragma once

#include <array>
#include <cmath>
#include <complex>
#include <stdexcept>

#include "core/Geometry.h"
#include "core/MathUtils.h"
#include "core/Pacific.h"


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
BSDFFlags operator|(BSDFFlags a, BSDFFlags b);
BSDFFlags operator&(BSDFFlags a, BSDFFlags b);
BSDFFlags& operator|=(BSDFFlags& a, BSDFFlags b);
BSDFFlags& operator&=(BSDFFlags& a, BSDFFlags b);

class BSDF {
private:
    BSDFFlags flags;

public:
    BSDF(BSDFFlags flags) : flags(flags) {}

    /// @brief Compute the Fresnel reflection coefficient
    /// @param cos_theta_i Incoming angle cosine. Should be positive.
    /// @param eta Relative ior (eta_i over eta_t)
    static Float fresnelReflection(Float cos_theta_i, Float eta);

    /// @brief Compute the Fresnel reflection coefficient
    /// @param cos_theta_i Incoming angle cosine, should be positive
    /// @param eta Relative ior (eta_t over eta_i)
    static Float fresnelComplex(Float cos_theta_i, std::complex<Float> eta);

    static Vec3f fresnelComplex(Float cos_theta_i, const Vec3f &eta, const Vec3f &k);
    
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
        return (flags & flag) != BSDFFlags::None;
    }

    virtual std::string to_string() const = 0;
};
