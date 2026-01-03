#include "core/Geometry.h"

bool BVHNode::intersect(const Ray &ray, Intersection &isc) {
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
        for (const auto &geom : geoms) {
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

// OPTIONAL: Optimized traversal that updates ray.tmax for better culling
bool BVHNode::intersect_optimized(Ray &ray, Intersection &isc) {
    const Float epsilon = 1e-7;
    
    // AABB-ray intersection test with division-by-zero handling
    Float tmin = ray.tmin;
    Float tmax = ray.tmax;
    
    // X axis
    if (std::abs(ray.d.x) < epsilon) {
        if (ray.o.x < bbox.min_corner.x || ray.o.x > bbox.max_corner.x)
            return false;
    } else {
        Float t1 = (bbox.min_corner.x - ray.o.x) / ray.d.x;
        Float t2 = (bbox.max_corner.x - ray.o.x) / ray.d.x;
        if (t1 > t2) std::swap(t1, t2);
        tmin = std::max(tmin, t1);
        tmax = std::min(tmax, t2);
        if (tmin > tmax) return false;
    }
    
    // Y axis
    if (std::abs(ray.d.y) < epsilon) {
        if (ray.o.y < bbox.min_corner.y || ray.o.y > bbox.max_corner.y)
            return false;
    } else {
        Float t1 = (bbox.min_corner.y - ray.o.y) / ray.d.y;
        Float t2 = (bbox.max_corner.y - ray.o.y) / ray.d.y;
        if (t1 > t2) std::swap(t1, t2);
        tmin = std::max(tmin, t1);
        tmax = std::min(tmax, t2);
        if (tmin > tmax) return false;
    }
    
    // Z axis
    if (std::abs(ray.d.z) < epsilon) {
        if (ray.o.z < bbox.min_corner.z || ray.o.z > bbox.max_corner.z)
            return false;
    } else {
        Float t1 = (bbox.min_corner.z - ray.o.z) / ray.d.z;
        Float t2 = (bbox.max_corner.z - ray.o.z) / ray.d.z;
        if (t1 > t2) std::swap(t1, t2);
        tmin = std::max(tmin, t1);
        tmax = std::min(tmax, t2);
        if (tmin > tmax) return false;
    }
    
    if (tmax < ray.tmin - epsilon || tmin > ray.tmax + epsilon)
        return false;
    
    // Leaf node
    if (left == nullptr && right == nullptr) {
        bool is_hit = false;
        
        for (const auto &geom : geoms) {
            Intersection isc_tmp{};
            bool is_hit_tmp = geom->intersect(ray, isc_tmp);
            
            if (is_hit_tmp && isc_tmp.distance >= ray.tmin && isc_tmp.distance <= ray.tmax) {
                if (ray.shadow_ray)
                    return true;
                
                if (isc_tmp.distance < ray.tmax) {
                    ray.tmax = isc_tmp.distance;  // Update ray for culling
                    isc = isc_tmp;
                    is_hit = true;
                }
            }
        }
        return is_hit;
    }
    
    // Internal node - optimized traversal
    if (ray.shadow_ray) {
        if (left && left->intersect_optimized(ray, isc))
            return true;
        if (right && right->intersect_optimized(ray, isc))
            return true;
        return false;
    } else {
        // Traverse nearer child first (simple heuristic)
        bool hit = false;
        Intersection isc_tmp;
        
        // Try left child
        if (left && left->intersect_optimized(ray, isc_tmp)) {
            isc = isc_tmp;
            hit = true;
            // ray.tmax has been updated, potentially culling right child
        }
        
        // Try right child (may be culled by updated ray.tmax)
        if (right && right->intersect_optimized(ray, isc_tmp)) {
            isc = isc_tmp;
            hit = true;
        }
        
        return hit;
    }
}
