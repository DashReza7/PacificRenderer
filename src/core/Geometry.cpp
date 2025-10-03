#include "core/Geometry.h"

bool BVHNode::intersect(const Ray& ray, Intersection& isc) {
    // AABB-ray intersection test
    Float tmin = (bbox.min_corner.x - ray.o.x) / ray.d.x;
    Float tmax = (bbox.max_corner.x - ray.o.x) / ray.d.x;
    if (tmin > tmax)
        std::swap(tmin, tmax);
    Float tymin = (bbox.min_corner.y - ray.o.y) / ray.d.y;
    Float tymax = (bbox.max_corner.y - ray.o.y) / ray.d.y;
    if (tymin > tymax)
        std::swap(tymin, tymax);
    if ((tmin > tymax) || (tymin > tmax))
        return false;
    if (tymin > tmin)
        tmin = tymin;
    if (tymax < tmax)
        tmax = tymax;
    Float tzmin = (bbox.min_corner.z - ray.o.z) / ray.d.z;
    Float tzmax = (bbox.max_corner.z - ray.o.z) / ray.d.z;
    if (tzmin > tzmax)
        std::swap(tzmin, tzmax);
    if ((tmin > tzmax) || (tzmin > tmax))
        return false;
    if (tzmin > tmin)
        tmin = tzmin;
    if (tzmax < tmax)
        tmax = tzmax;
    // Check if AABB intersection is within ray bounds
    if (tmax < ray.tmin || tmin > ray.tmax)
        return false;

    // Leaf node
    if (left == nullptr && right == nullptr) {
        bool is_hit = false;
        Float best_dist = INFINITY;
        for (const auto& geom : geoms) {
            Intersection isc_tmp{};
            bool is_hit_tmp = geom->intersect(ray, isc_tmp);
            if (is_hit_tmp) {
                if (ray.shadow_ray)
                    return true;
                is_hit = true;
                if (isc_tmp.distance < best_dist) {
                    best_dist = isc_tmp.distance;
                    isc = isc_tmp;
                }
            }
        }
        return is_hit;
    }

    bool hit_left = false, hit_right = false;
    Intersection isc_left, isc_right;
    hit_left = left->intersect(ray, isc_left);
    hit_right = right->intersect(ray, isc_right);

    if (hit_left && hit_right) {
        if (isc_left.distance < isc_right.distance)
            isc = isc_left;
        else
            isc = isc_right;
        return true;
    } else if (hit_left) {
        isc = isc_left;
        return true;
    } else if (hit_right) {
        isc = isc_right;
        return true;
    } else {
        return false;
    }
}
