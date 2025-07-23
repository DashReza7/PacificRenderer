#pragma once

#include "core/BSDF.h"
#include "core/Geometry.h"

#include <memory>
#include <vector>

class Geometry;

class Shape {
   public:
    std::vector<std::unique_ptr<Geometry>> geometry;
    std::shared_ptr<BSDF> bsdf;

    Shape() = default;
    ~Shape() = default;

   private:
};
