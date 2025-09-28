#pragma once

#include <memory>
#include <vector>

#include "core/BSDF.h"
#include "core/Geometry.h"
class Geometry;

class Shape {
public:
    std::vector<Geometry *> geometries{};
    BSDF *bsdf;

    Shape() = default;
    ~Shape() = default;

private:
};
