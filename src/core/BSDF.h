#pragma once

#include <array>

#include "core/Pacific.h"

class BSDF {
   public:
    BSDF() = default;
    ~BSDF() = default;

    virtual std::array<Float, 2> sample(Float theta_i) = 0;
    virtual Float eval(Float theta_i, Float theta_o) = 0;
    virtual Float pdf(Float theta_i, Float theta_o) = 0;

   private:
};
