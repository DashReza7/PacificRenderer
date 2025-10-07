#include "core/Shape.h"

std::tuple<Vec3f, Vec3f, Float> Shape::sample_point_on_surface(Float sample1, const Vec2f &sample2) const {
    if (geometries.size() == 0)
        throw std::runtime_error("Shape has no geometries to sample from");

    Geometry *geom = geometries[0];
    // randomly select a geometry and sample on it
    if (type == Type::OBJ)
        geom = geometries[std::min(int(sample1 * geometries.size()), int(geometries.size() - 1))];
    auto [p, n, pdf] = geom->sample_point_on_surface(sample2);
    pdf /= geometries.size();

    return {p, n, pdf};
}
