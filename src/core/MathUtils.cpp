#include "core/MathUtils.h"
#include "core/MathUtils.h"
#include <iostream>


Mat3f localToWorldMat(const Vec3f &world_z) {
    Float sign = world_z.z >= 0 ? 1.0 : -1.0;
    Float a = -1.0 / (sign + world_z.z);
    Float b = world_z.x * world_z.y * a;
    Vec3f world_x{1.0 + sign * world_z.x * world_z.x * a, sign * b, -sign * world_z.x};
    Vec3f world_y{b, sign + world_z.y * world_z.y * a, -world_z.y};
    Mat3f mat;
    mat[0] = world_x;
    mat[1] = world_y;
    mat[2] = world_z;
    return mat;
}

Vec3f localToWorld(const Vec3f &local, const Vec3f &world_z) {
    if (std::abs(glm::length(world_z) - 1.0) > 1e-6)
        throw std::runtime_error("world_z is not normalized in localToWorld");
    
    Mat3f trafo = localToWorldMat(world_z);
    return trafo * local;
}

Vec3f worldToLocal(const Vec3f &world, const Vec3f &world_z) {
    Mat3f trafo = localToWorldMat(world_z);
    return glm::transpose(trafo) * world;
}

Vec3f reflect(const Vec3f &wi, const Vec3f &n) {
    return -wi + Float(2.0 * dot(wi, n)) * n;
}

bool refract(const Vec3f &wi, const Vec3f &n, Float eta, Vec3f &wo) {
    Float cos_theta_i = dot(n, wi);
    Float sin2_theta_i = std::max(Float(0), Float(1) - Sqr(cos_theta_i));
    Float sin2_theta_t = sin2_theta_i / Sqr(eta);
    // total internal reflection
    if (sin2_theta_t >= 1.0)
        return false;
    Float cos_theta_t = glm::max(std::sqrt(1.0 - sin2_theta_t), 0.0);
    wo = -wi / eta + (cos_theta_i / eta - cos_theta_t) * n;
    return true;
}

Float triangle_area(const Vec3f &a, const Vec3f &b, const Vec3f &c) {
    return Float(0.5) * length(cross(b - a, c - a));
}

Vec2f uniformDiskSample(const Vec2f &sample) {
    Float r = std::sqrt(sample.x);
    Float theta = 2.0 * Pi * sample.y;
    return Vec2f{r * std::cos(theta), r * std::sin(theta)};
}

Vec3f uniformHemisphereSample(const Vec2f &sample) {
    Float cos_theta = sample.x;
    Float sin_theta = std::sqrt(std::abs(1.0 - Sqr(cos_theta)));
    Float phi = 2.0 * Pi * sample.y;
    return Vec3f{sin_theta * std::cos(phi), sin_theta * std::sin(phi), cos_theta};
}

Vec3f uniformSphereSample(const Vec2f &sample) {
    Float cos_theta = 1.0 - 2.0 * sample.x;
    Float sin_theta = std::sqrt(std::abs(1.0 - Sqr(cos_theta)));
    Float phi = 2.0 * Pi * sample.y;
    return Vec3f{sin_theta * std::cos(phi), sin_theta * std::sin(phi), cos_theta};
}

Vec3f cosineHemisphereSample(const Vec2f &sample) {
    Float theta = std::acos(std::sqrt(sample.x));
    Float phi = 2.0 * Pi * sample.y;
    return Vec3f{std::sin(theta) * std::cos(phi), std::sin(theta) * std::sin(phi), std::cos(theta)};
}
Float cosineHemispherePDF(const Vec3f &wi, const Vec3f &wo) {
    if (wi.z * wo.z <= 0.0)
        return 0.0;
    return std::abs(wo.z) * InvPi;
}

Vec3f sphericalToCartesian(Float theta, Float phi) {
    Float sin_theta = std::sin(theta);
    return Vec3f{
        sin_theta * std::cos(phi),
        sin_theta * std::sin(phi),
        std::cos(theta)};
}

Vec3f barycentric(const Vec3f &v0, const Vec3f &v1, const Vec3f &v2, const Vec3f &p) {
    Vec3f v0v1 = v1 - v0;
    Vec3f v0v2 = v2 - v0;
    Vec3f v0p = p - v0;

    Float d00 = glm::dot(v0v1, v0v1);
    Float d01 = glm::dot(v0v1, v0v2);
    Float d11 = glm::dot(v0v2, v0v2);
    Float d20 = glm::dot(v0p, v0v1);
    Float d21 = glm::dot(v0p, v0v2);

    Float denom = d00 * d11 - d01 * d01;
    if (denom <= 1e-11) {
        std::cerr << "Warning: barycentric() -> Degenerate triangle exists: " << denom << std::endl;
        return Vec3f{0.33, 0.33, 0.34}; // degenerate triangle, return arbitrary barycentric coords
    }

    Float w1 = (d11 * d20 - d01 * d21) / denom;
    Float w2 = (d00 * d21 - d01 * d20) / denom;
    Float w0 = 1.0 - w1 - w2;

    return Vec3f{w0, w1, w2};
}

Float lerp(Float t, Float a, Float b) {
    return (1.0 - t) * a + t * b;
}

Mat4f get_rotation_matrix(const Vec3f &axis, Float angle) {
    Float cos_phi = std::cos(glm::radians(angle));
    Float one_minus_cos_phi = 1.0 - cos_phi;
    Float sin_phi = std::sin(glm::radians(angle));
    Mat4f rot_mat = {
        {cos_phi + one_minus_cos_phi * Sqr(axis.x), one_minus_cos_phi * axis.x * axis.y + axis.z * sin_phi, one_minus_cos_phi * axis.x * axis.z - axis.y * sin_phi, 0.0},
        {one_minus_cos_phi * axis.x * axis.y - axis.z * sin_phi, cos_phi + one_minus_cos_phi * Sqr(axis.y), one_minus_cos_phi * axis.y * axis.z + axis.x * sin_phi, 0.0},
        {one_minus_cos_phi * axis.x * axis.z + axis.y * sin_phi, one_minus_cos_phi * axis.y * axis.z - axis.x * sin_phi, cos_phi + one_minus_cos_phi * Sqr(axis.z), 0.0},
        {0.0, 0.0, 0.0, 1.0}};

    return rot_mat;
}
