#include "core/MathUtils.h"
#include "core/Geometry.h"

inline Float luminance(const Vec3f &color) {
    return 0.212671 * color.x + 0.715160 * color.y + 0.072169 * color.z;
}

class Texture {
public:
    virtual ~Texture() = default;

    virtual Vec3f eval(const Intersection &isc) const = 0;
    virtual Float mean() const = 0;
};
