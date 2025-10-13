#include "core/MathUtils.h"
#include "core/Geometry.h"


class Texture {
public:
    virtual ~Texture() = default;

    virtual Vec3f eval(const Intersection &isc) const = 0;
};
