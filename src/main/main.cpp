#pragma once

#include <iostream>
#include <memory>
#include "core/Pacific.h"
#include "utils/Misc.h"
#include "core/MathUtils.h"
#include "utils/SceneParser.h"

int main()
{
	Vec3f a{ 1, 2, 3 };
	Vec3f b{ 2, 3, 4 };
	Float dotp = dot(a, b);

	std::cout << dotp << std::endl;
}
