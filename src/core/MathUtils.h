#pragma once

#include <sstream>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "core/Pacific.h"

inline const Float Epsilon = 1e-4f;
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
using Vec2i = glm::vec<2, int>;
using Vec3i = glm::vec<3, int>;
using Vec4i = glm::vec<4, int>;
using Mat4f = glm::mat<4, 4, Float>;
using Mat4i = glm::mat<4, 4, int>;


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
