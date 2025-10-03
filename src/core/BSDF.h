#pragma once

#include <array>
#include <cmath>
#include <stdexcept>

#include "core/Geometry.h"
#include "core/MathUtils.h"
#include "core/Pacific.h"

/// @brief BSDFSample object.
/// wo is in world space
struct BSDFSample {
    /// Outgoing direction in world space
    Vec3f wo;
    /// Probability density at the sample
    Float pdf;
    /// Relative ior in the sample direction
    Float eta;

    BSDFSample(const Vec3f &wo, Float pdf, Float eta) : wo(wo), pdf(pdf), eta(eta) {}
};

class BSDF {
public:
    BSDF() = default;

    /// @brief Sample the BSDF
    /// @param isc Intersection info, including the incoming direction in world space
    /// @param sample1 1D random sample in [0,1). Used for selecting multiple lobes in mixture BSDFs
    /// @param sample2 2D random sample in [0,1)^2
    /// @return BSDFSample and the BSDF value(RGB). the sample_eval accounts for the cosine foreshortening term if needed. (i.e. not in delta BSDFs)
    virtual std::pair<BSDFSample, Vec3f> sample(const Intersection &isc, Float sample1, const Vec2f &sample2) const = 0;
    
    /// @brief Evaluate the BSDF
    /// @param wi Incoming direction in local space (z is the normal direction)
    /// @param wo Outgoing direction in local space (z is the normal direction)
    /// @return BSDF value. the return value accounts for the cosine foreshortening term if needed. (i.e. not in delta BSDFs)
    virtual Vec3f eval(const Vec3f &wi, const Vec3f &wo) const = 0;
    
    /// @brief Evaluate the PDF of the BSDF sample
    /// @param wi Incoming direction in local space (z is the normal direction)
    /// @param wo Outgoing direction in local space (z is the normal direction)
    virtual Float pdf(const Vec3f &wi, const Vec3f &wo) const = 0;

    virtual std::string to_string() const = 0;
};


// TODO: to be implemented:
// RoughDielectricBSDF
// SmoothConductorBSDF
// RoughConductorBSDF
// SmoothPlasticBSDF
// RoughPlasticBSDF
// DisneyBSDF
