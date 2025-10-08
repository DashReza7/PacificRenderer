#include "core/Emitter.h"

// Bitwise operators for EmitterFlags
EmitterFlags operator|(EmitterFlags a, EmitterFlags b) {
    return static_cast<EmitterFlags>(static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
}
EmitterFlags operator&(EmitterFlags a, EmitterFlags b) {
    return static_cast<EmitterFlags>(static_cast<uint32_t>(a) & static_cast<uint32_t>(b));
}
EmitterFlags& operator|=(EmitterFlags& a, EmitterFlags b) {
    return a = a | b;
}
EmitterFlags& operator&=(EmitterFlags& a, EmitterFlags b) {
    return a = a & b;
}
