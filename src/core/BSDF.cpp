#include "core/BSDF.h"
#include "core/BSDF.h"

// Bitwise operators for BSDFFlags
BSDFFlags operator|(BSDFFlags a, BSDFFlags b) {
    return static_cast<BSDFFlags>(static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
}
BSDFFlags operator&(BSDFFlags a, BSDFFlags b) {
    return static_cast<BSDFFlags>(static_cast<uint32_t>(a) & static_cast<uint32_t>(b));
}
BSDFFlags& operator|=(BSDFFlags& a, BSDFFlags b) {
    return a = a | b;
}
BSDFFlags& operator&=(BSDFFlags& a, BSDFFlags b) {
    return a = a & b;
}

// Bitwise operators for BSDFSampleFlags
BSDFSampleFlags operator|(BSDFSampleFlags a, BSDFSampleFlags b) {
    return static_cast<BSDFSampleFlags>(static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
}
BSDFSampleFlags operator&(BSDFSampleFlags a, BSDFSampleFlags b) {
    return static_cast<BSDFSampleFlags>(static_cast<uint32_t>(a) & static_cast<uint32_t>(b));
}
BSDFSampleFlags& operator|=(BSDFSampleFlags& a, BSDFSampleFlags b) {
    return a = a | b;
}
BSDFSampleFlags& operator&=(BSDFSampleFlags& a, BSDFSampleFlags b) {
    return a = a & b;
}

Float fresnelReflection(Float cos_theta_i, Float eta) {
    if (cos_theta_i < 0.0) {
        eta = 1.0 / eta;
        cos_theta_i = -cos_theta_i;
    }
    
    // Snell's law
    Float sin2_theta_t = (1 - Sqr(cos_theta_i)) / Sqr(eta);
    if (sin2_theta_t >= 1)  // total internal reflection
        return 1.0;
    Float cos_theta_t = std::sqrt(std::abs(1.0 - sin2_theta_t));

    Float r_parl = (eta * cos_theta_i - cos_theta_t) / (eta * cos_theta_i + cos_theta_t);
    Float r_perp = (cos_theta_i - eta * cos_theta_t) / (cos_theta_i + eta * cos_theta_t);
    return (Sqr(r_parl) + Sqr(r_perp)) * 0.5;
}

Float fresnelComplex(Float cos_theta_i, std::complex<Float> eta) {
    Float sin2_theta_i = 1.0 - Sqr(cos_theta_i);
    std::complex<Float> sin2_theta_t = sin2_theta_i / Sqr(eta);
    std::complex<Float> cos_theta_t = std::sqrt(Float(1.0) - sin2_theta_t);

    std::complex<Float> r_parl = (eta * cos_theta_i - cos_theta_t) / (eta * cos_theta_i + cos_theta_t);
    std::complex<Float> r_perp = (cos_theta_i - eta * cos_theta_t) / (cos_theta_i + eta * cos_theta_t);
    return (std::norm(r_parl) + std::norm(r_perp)) / 2;
}

Vec3f fresnelComplex(Float cos_theta_i, const Vec3f& eta, const Vec3f& k) {
    return Vec3f(fresnelComplex(cos_theta_i, std::complex<Float>{eta.x, k.x}), fresnelComplex(cos_theta_i, std::complex<Float>{eta.y, k.y}), fresnelComplex(cos_theta_i, std::complex<Float>{eta.z, k.z}));
}

Float fresnel_diffuse_reflectance(Float eta) {
    // implementation based on Mitsuba3
    Float inv_eta = 1.0 / eta;
    if (eta < 1.0)
        return -1.4399f * Sqr(eta) + 0.7099 * eta + 0.6681f + 0.0636f * inv_eta;
    else
        return -1.36881 * std::pow(inv_eta, 5) + 4.98554 * std::pow(inv_eta, 4) - 7.80989 * std::pow(inv_eta, 3) + 6.75335 * std::pow(inv_eta, 2) - 3.4793 * inv_eta + 0.919317;
}
