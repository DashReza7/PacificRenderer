#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "core/Pacific.h"

const Float Epsilon = 1e-4f;
const Float Pi = 3.14159265358979323846;
const Float InvPi = 0.31830988618379067154;
const Float Inv2Pi = 0.15915494309189533577;
const Float Inv4Pi = 0.07957747154594766788;
const Float PiOver2 = 1.57079632679489661923;
const Float PiOver4 = 0.78539816339744830961;
const Float Sqrt2 = 1.41421356237309504880;

using Vec2f = glm::vec<2, Float>;
using Vec3f = glm::vec<3, Float>;
using Vec4f = glm::vec<4, Float>;
using Vec2i = glm::vec<2, int>;
using Vec3i = glm::vec<3, int>;
using Vec4i = glm::vec<4, int>;
using Mat4f = glm::mat<4, 4, Float>;
using Mat4i = glm::mat<4, 4, int>;
