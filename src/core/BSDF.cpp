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

// GGX implementations are based on PBRT
Float GGXDistribution::D(const Vec3f& wm, Float alpha_u, Float alpha_v) {
    if (std::abs(wm.z) <= Epsilon)
        return 0.0;
    Float tan2Theta = tan2_theta(wm);
    Float cos4_theta = Sqr(cos2_theta(wm));
    Float e = tan2Theta * (Sqr(cos_phi(wm) / alpha_u) +
                           Sqr(sin_phi(wm) / alpha_v));
    return 1.0 / (Pi * alpha_u * alpha_v * cos4_theta * Sqr(1 + e));
}
Float GGXDistribution::D(const Vec3f& w, const Vec3f& wm, Float alpha_u, Float alpha_v) {
    return G1(w, alpha_u, alpha_v) * D(wm, alpha_u, alpha_v) * std::abs(glm::dot(w, wm)) / std::abs(w.z);
}
Float GGXDistribution::G1(const Vec3f& w, Float alpha_u, Float alpha_v) {
    return 1.0 / (1.0 + Lambda(w, alpha_u, alpha_v));
}
Float GGXDistribution::Lambda(const Vec3f& w, Float alpha_u, Float alpha_v) {
    if (std::abs(w.z) <= Epsilon)
        return 0.0;
    Float tan2Theta = tan2_theta(w);
    Float alpha2 = Sqr(cos_phi(w) * alpha_u) + Sqr(sin_phi(w) * alpha_v);
    return (std::sqrt(1 + alpha2 * tan2Theta) - 1.0) / 2.0;
}
Float GGXDistribution::G(const Vec3f& wi, const Vec3f& wo, Float alpha_u, Float alpha_v) {
    return 1.0 / (1.0 + Lambda(wi, alpha_u, alpha_v) + Lambda(wo, alpha_u, alpha_v));
}
Vec3f GGXDistribution::sample_wm(const Vec3f& w, Float alpha_u, Float alpha_v, const Vec2f& sample) {
    Vec3f wh = glm::normalize(Vec3f(alpha_u * w.x, alpha_v * w.y, w.z));
    if (wh.z < 0)
        wh = -wh;

    Vec3f T1 = (wh.z < 0.99999) ? glm::normalize(glm::cross(Vec3f{0, 0, 1}, wh)) : Vec3f{1, 0, 0};
    Vec3f T2 = glm::cross(wh, T1);

    Vec2f p = uniformDiskSample(sample);

    Float h = std::sqrt(1 - Sqr(p.x));
    p.y = lerp((1.0 + wh.z) / 2.0, h, p.y);

    Float pz = std::sqrt(std::max<Float>(0, 1 - glm::dot(p, p)));
    Vec3f nh = p.x * T1 + p.y * T2 + pz * wh;

    return glm::normalize(Vec3f(alpha_u * nh.x, alpha_v * nh.y, std::max<Float>(Float(1e-6), nh.z)));
}
Float GGXDistribution::pdf(const Vec3f& w, const Vec3f& wm, Float alpha_u, Float alpha_v) {
    return D(w, wm, alpha_u, alpha_v);
}
