#include "core/MathUtils.h"

class Microfacet {
public:
    virtual ~Microfacet() = default;

    virtual Vec3f sample_wm(const Vec3f &w, const Vec2f &sample) const = 0;
    virtual Float pdf(const Vec3f &w, const Vec3f &wm) const = 0;
    virtual Float D(const Vec3f &wm) const = 0;
    virtual Float G(const Vec3f &wi, const Vec3f &wo) const = 0;
};
