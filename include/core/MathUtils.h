#pragma once

#include <sstream>

#include <glm/gtc/matrix_transform.hpp>

#include "core/Pacific.h"


#define Sqr(x) ((x)*(x))

inline const Float Epsilon = 5e-6;
inline const Float Pi = 3.14159265358979323846;
inline const Float InvPi = 0.31830988618379067154;
inline const Float Inv2Pi = 0.15915494309189533577;
inline const Float Inv4Pi = 0.07957747154594766788;
inline const Float PiOver2 = 1.57079632679489661923;
inline const Float PiOver4 = 0.78539816339744830961;
inline const Float Sqrt2 = 1.41421356237309504880;

using Vec2f = glm::vec<2, Float>;
using Vec3f = glm::vec<3, Float>;
using Vec4f = glm::vec<4, Float>;
using Mat3f = glm::mat<3, 3, Float>;
using Mat4f = glm::mat<4, 4, Float>;


inline std::ostream& operator<<(std::ostream& os, const Vec2f& v) {
    os << "[" << v.x << ", " << v.y << "]";
    return os;
}
inline std::ostream& operator<<(std::ostream& os, const Vec3f& v) {
    os << "[" << v.x << ", " << v.y << ", " << v.z << "]";
    return os;
}
inline std::ostream& operator<<(std::ostream& os, const Vec4f& v) {
    os << "[" << v.x << ", " << v.y << ", " << v.z << ", " << v.w << "]";
    return os;
}
inline std::ostream& operator<<(std::ostream& os, const Mat4f& m) {
    os << "[";
    for (int row = 0; row < 4; row++) {
        os << "[";
        for (int col = 0; col < 4; col++) {
            os << m[col][row];  // GLM: m[column][row]
            if (col < 3)
                os << ", ";
        }
        os << "]";
        if (row < 3)
            os << ",\n ";
    }
    os << "]";
    return os;
}


Mat3f localToWorldMat(const Vec3f &world_z);
/// @brief Transform a vector from local-space to world-space
Vec3f localToWorld(const Vec3f &local, const Vec3f &world_z);
Vec3f worldToLocal(const Vec3f &world, const Vec3f &world_z);

/// @brief Reflect the incident direction `wi` about the normal `n`
/// @param wi Incident direction (pointing outward from the surface)
/// @param n Normal direction (pointing outward from the surface)
/// @return Reflected direction (pointing outward from the surface)
Vec3f reflect(const Vec3f &wi, const Vec3f &n);
/// @brief Refract the incident direction `wi` through the surface with normal `n`. wi & n must be on the same hemisphere
/// @param wi Incident direction (pointing outward from the surface)
/// @param n Normal direction (pointing outward from the surface)
/// @param eta Relative index of refraction (eta_t / eta_i)
/// @return Refracted direction (pointing outward from the surface)
bool refract(const Vec3f &wi, const Vec3f &n, Float eta, Vec3f &wo);

Float triangle_area(const Vec3f &a, const Vec3f &b, const Vec3f &c);

/// @brief uniformly sample a discrete pmf from 0 (inclusive) to n (exclusive)
int uniformDiscrete(Float sample, int n);
/// @brief uniformly sample a discrete pmf from a to b (both inclusive)
int uniformDiscrete(Float sample, int a, int b);
Vec2f uniformDiskSample(const Vec2f &sample);
Vec3f uniformHemisphereSample(const Vec2f &sample);
Vec3f uniformSphereSample(const Vec2f &sample);
Vec3f cosineHemisphereSample(const Vec2f &sample);
Float cosineHemispherePDF(const Vec3f &wi, const Vec3f &wo);

Vec3f sphericalToCartesian(Float theta, Float phi);

inline Float sign(Float x) {
    return x > 0.0 ? 1.0 : (x < 0.0 ? -1.0 : 0.0);
}

inline bool check_valid(Float x, bool nan=true, bool inf=true, bool neg=true) {
    if (nan && std::isnan(x))
        return false;
    if (inf && std::isinf(x))
        return false;
    if (neg && x < 0)
        return false;
    return true;
}
inline bool check_valid(Vec3f x, bool nan=true, bool inf=true, bool neg=true) {
    if (nan && (std::isnan(x.x) || std::isnan(x.y) || std::isnan(x.z)))
        return false;
    if (inf && (std::isinf(x.x) || std::isinf(x.y) || std::isinf(x.z)))
        return false;
    if (neg && (x.x < 0 || x.y < 0 || x.z < 0))
        return false;
    return true;
}

Vec3f barycentric(const Vec3f& v0, const Vec3f& v1, const Vec3f& v2, const Vec3f& p);
Float lerp(Float t, Float a, Float b);

// angle is in degrees
Mat4f get_rotation_matrix(const Vec3f &axis, Float angle);

inline Float cos_theta(const Vec3f &w) {
    return w.z;
}
inline Float cos2_theta(const Vec3f &w) {
    return w.z * w.z;
}
inline Float sin2_theta(const Vec3f &w) {
    return glm::max(Float(0), Float(1) - cos2_theta(w));
}
inline Float sin_theta(const Vec3f &w) {
    return std::sqrt(sin2_theta(w)) * (w.z >= 0 ? 1.0 : -1.0);
}
inline Float tan2_theta(const Vec3f &w) {
    return 1.0 / cos2_theta(w) - 1.0;
}
inline Float tan_theta(const Vec3f &w) {
    return std::sqrt(tan2_theta(w)) * (w.z >= 0 ? 1.0 : -1.0);
}
inline Float cos_phi(const Vec3f &w) {
    Float sinTheta = sin_theta(w);
    return sinTheta <= Epsilon ? 1.0 : glm::clamp(w.x / sinTheta, Float(-1.0), Float(1.0));
}
inline Float sin_phi(const Vec3f &w) {
    Float sinTheta = sin_theta(w);
    return sinTheta <= Epsilon ? 0.0 : glm::clamp(w.y / sinTheta, Float(-1.0), Float(1.0));
}
inline Float absDot(const Vec3f &v, const Vec3f &w) {
    return std::abs(glm::dot(v, w));
}

inline Vec3f face_forward(const Vec3f &v, const Vec3f &n) {
    return glm::dot(v, n) >= 0.0 ? v : -v;
}
