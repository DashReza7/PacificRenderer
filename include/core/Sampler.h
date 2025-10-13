#pragma once
#include "core/MathUtils.h"

/// TODO: right now only independent sampler(PCG)
class Sampler {
private:
    uint64_t state;
    
    uint32_t next_uint32() {
        uint64_t oldstate = state;
        state = oldstate * 6364136223846793005ULL + 1442695040888963407ULL;
        uint32_t xorshifted = ((oldstate >> 18u) ^ oldstate) >> 27u;
        uint32_t rot = oldstate >> 59u;
        return (xorshifted >> rot) | (xorshifted << ((-rot) & 31));
    }

public:
    uint32_t spp;
    
    Sampler(uint64_t seed, uint32_t spp) : state(0), spp(spp) {
        next_uint32();
        state += seed;
        next_uint32();
    }

    
    Float get_1D() {
        return (next_uint32() >> 8) * 0x1.0p-24f; // 24-bit precision
    }

    Vec2f get_2D() {
        return Vec2f{get_1D(), get_1D()};
    }
    
    Vec3f get_3D() {
        return Vec3f{get_1D(), get_1D(), get_1D()};
    }

    std::string to_string() const {
        std::ostringstream ss;
        ss << "Sampler: [ samples_count=" << spp << " ]";
        return ss.str();
    }
};
