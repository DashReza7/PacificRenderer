#include "core/Emitter.h"

#include "core/Geometry.h"
#include "core/Scene.h"


EmitterSample PointLight::sampleLi(const Scene *scene, const Intersection &isc, const Vec3f &sample) const {
    Vec3f dirn = isc.position - position;
    Float distance = glm::length(dirn);
    dirn = glm::normalize(dirn);
    bool is_valid = dot(dirn, isc.normal) > 0.0;
    if (is_valid) {
        // check for occlusion
        Ray shadow_ray{isc.position + isc.normal * Epsilon, dirn, Epsilon, distance - 2 * Epsilon, true};
        Intersection tmp_isc{};
        is_valid = !scene->ray_intersect(shadow_ray, tmp_isc);
    }

    return EmitterSample{1.0, position, is_valid, intensity / Sqr(distance)};
}

EmitterSample AreaLight::sampleLi(const Scene *scene, const Intersection &isc, const Vec3f &sample) const {
    auto [position, normal, pdf] = shape->sample_point_on_surface(sample.x, Vec2f{sample.y, sample.z});
    Vec3f dirn = position - isc.position;
    Float distance = glm::length(dirn);
    dirn = glm::normalize(dirn);
    bool is_valid = glm::dot(normal, dirn) < 0 && glm::dot(isc.normal, dirn) > 0;
    if (is_valid) {
        // check for occlusion
        Ray shadow_ray{isc.position + isc.normal * Epsilon, dirn, Epsilon, distance - 2 * Epsilon, false};
        Intersection tmp_isc;
        bool is_hit = scene->ray_intersect(shadow_ray, tmp_isc);
        if (is_hit) {
            // might be a false positive
            if (tmp_isc.shape != shape || glm::length(tmp_isc.position - position) >= Epsilon)
                is_valid = false;
        }
    }
    pdf *= Sqr(distance) / glm::dot(normal, -dirn);
    return EmitterSample{pdf, position, is_valid, radiance};
}
