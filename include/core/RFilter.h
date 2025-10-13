#pragma once
#include <core/Pacific.h>
#include <core/MathUtils.h>

class RFilter {
public:
    Float radius;
    int bound;

    RFilter(Float radius) : radius(radius) {
        bound = static_cast<int>(radius - 0.5) + 1;
    }

    /// @brief Evaluate the filter weight at a given position (x, y)
    /// @param x x coordinate relative to the pixel center
    /// @param y y coordinate relative to the pixel center
    virtual Float eval(Float x, Float y) const = 0;
};
