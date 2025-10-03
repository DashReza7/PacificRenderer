#include "core/MathUtils.h"

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
    Mat3f trafo = localToWorldMat(world_z);
    return trafo * local;
}

Vec3f worldToLocal(const Vec3f &world, const Vec3f &world_z) {
    Mat3f trafo = localToWorldMat(world_z);
    return glm::inverse(trafo) * world;
}

Vec3f reflect(const Vec3f &wi, const Vec3f &n) {
    return -wi + Float(2.0 * dot(wi, n)) * n;
}

Vec3f refract(const Vec3f &wi, const Vec3f &n, Float eta) {
    Float cos_theta_i = glm::clamp(dot(n, wi), Float(-1.0), Float(1.0));
    Float sin2_theta_i = std::max(Float(0), Float(1) - Sqr(cos_theta_i));
    Float sin2_theta_t = sin2_theta_i / Sqr(eta);
    // total internal reflection
    if (sin2_theta_t >= 1.0)
        return Vec3f{0.0};
    Float cos_theta_t = glm::clamp(std::sqrt(1.0 - sin2_theta_t), 0.0, 1.0);
    return eta * -wi + (eta * cos_theta_i - cos_theta_t) * n;
}

Float triangle_area(const Vec3f &a, const Vec3f &b, const Vec3f &c) {
    return Float(0.5) * length(cross(b - a, c - a));
}

Vec3f uniformHemisphereSample(const Vec2f &sample) {
    Float cos_theta = sample.x;
    Float sin_theta = std::sqrt(std::abs(1.0 - Sqr(cos_theta)));
    Float phi = 2.0 * Pi * sample.y;
    return Vec3f{sin_theta * std::cos(phi), sin_theta * std::sin(phi), cos_theta};
}

Vec3f cosineHemisphereSample(const Vec2f &sample) {
    Float theta = std::acos(std::sqrt(sample.x));
    Float phi = 2.0 * Pi * sample.y;
    return Vec3f{std::sin(theta) * std::cos(phi), std::sin(theta) * std::sin(phi), std::cos(theta)};
}
