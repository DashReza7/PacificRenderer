#include "core/Shape.h"

std::tuple<Vec3f, Vec3f, Float> Shape::sample_point_on_surface(Float sample1, const Vec2f &sample2) const {
    if (geometries.size() == 0)
        throw std::runtime_error("Shape has no geometries to sample from");

    if (type == Type::SPHERE) {
        // TODO: what to do when the sphere's transform has non-uniform scaling?
        // FIXME: wrong for non-uniform scaled sphere
        auto sphere = dynamic_cast<Sphere *>(geometries[0]);
        // uniform sampling on sphere surface
        Float phi = 2.0 * Pi * sample2.x;
        Float theta = acos(1.0 - 2.0 * sample2.y);
        Vec3f normal{
            sin(theta) * cos(phi),
            sin(theta) * sin(phi),
            cos(theta)};
        Vec3f position = sphere->center + sphere->radius * normal;
        position = Vec3f{sphere->transform * Vec4f{position, 1.0}};
        // calculate the true radius
        Vec3f scaled_x = sphere->transform * Vec4f{1, 0, 0, 0};
        Float radius = sphere->radius * glm::length(scaled_x);
        Float pdf = 1.0 / (4.0 * Pi * Sqr(radius));
        return {position, sphere->get_normal(position), pdf};
    } else if (type == Type::OBJ) {
        // randomly select a geometry and sample on it
        int geom_index = std::min(int(sample1 * geometries.size()), int(geometries.size() - 1));
        // FIXME: right now assuming only Triangle type (Quad not supported)
        Triangle *triangle = dynamic_cast<Triangle *>(geometries[geom_index]);
        if (triangle == nullptr)
            throw std::runtime_error("Shape::sample_point_on_surface() only supports Triangle geometries for OBJ shapes");
        // uniform sampling on triangle surface
        Float r_sqrd = std::sqrt(sample2.x);
        Vec3f rnd_pt = *triangle->positions[0] * (Float(1.0) - sample2.y) * r_sqrd
                     + *triangle->positions[1] * (Float(1.0) - r_sqrd)
                     + *triangle->positions[2] * sample2.y * r_sqrd;
        Vec3f normal = triangle->get_normal(rnd_pt);
        Float pdf = 1.0 / (triangle_area(*triangle->positions[0], *triangle->positions[1], *triangle->positions[2]) * geometries.size());
        
        return {rnd_pt, normal, pdf};
    } else {
        throw std::runtime_error("Shape::sample_point_on_surface() not implemented for this shape type");
    }
}
