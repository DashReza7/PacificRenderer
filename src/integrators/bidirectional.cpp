#include "core/Integrator.h"
#include "core/MathUtils.h"
#include "core/Scene.h"

class BidirectionalIntegrator : public SamplingIntegrator {
private:
    int max_depth;

    // prefix.size is the number of vertices. 1 means only the camera vertex.
    void createCameraPath(const Scene *scene, Sampler *sampler, const Ray &ray,
                         std::vector<Intersection> &prefix_iscs, std::vector<Vec3f> &prefix_betas,
                         int max_length = -1) const {
        prefix_iscs.push_back(Intersection{});
        prefix_betas.push_back(Vec3f{0});

        Vec3f beta = Vec3f{1.0};
        Ray cur_ray{ray};
        Intersection isc;

        bool is_hit = scene->ray_intersect(cur_ray, isc);
        if (!is_hit)
            return;
        prefix_iscs.push_back(isc);
        prefix_betas.push_back(beta);

        for (int depth = 1; depth < max_length || max_length < 0; depth++) {
            auto [bsdf_sample, bsdf_value] = isc.shape->bsdf->sample(isc, sampler->get_1D(), sampler->get_2D());
            cur_ray = Ray{isc.position + sign(glm::dot(isc.normal, bsdf_sample.wo)) * Epsilon * isc.normal,
                          bsdf_sample.wo, cur_ray.tmin, cur_ray.tmax};
            is_hit = scene->ray_intersect(cur_ray, isc);
            if (!is_hit || bsdf_sample.pdf == 0)
                return;
            beta *= bsdf_value / bsdf_sample.pdf;
            prefix_iscs.push_back(isc);
            prefix_betas.push_back(beta);
        }
    }

    // prefix.size is the number of vertices. 1 means only the camera vertex.
    void createLightPath(const Scene *scene, Sampler *sampler,
                        std::vector<Intersection> &prefix_iscs, std::vector<Vec3f> &prefix_betas,
                        int max_length = -1) const {
        Vec3f light_posn, light_normal, light_dirn;
        Float light_pdf;
        Vec3f lightLe = scene->sample_emitter_ptrace(sampler->get_2D(), sampler->get_3D(), sampler->get_1D(),
                                                     light_posn, light_normal, light_dirn, light_pdf);
        Intersection light_isc{};
        light_isc.position = light_posn;
        light_isc.normal = light_normal;
        prefix_iscs.push_back(light_isc);
        prefix_betas.push_back(lightLe / (light_pdf * 2.0f * Pi));

        // first intersection
        Ray ray{light_posn + Epsilon * light_normal, light_dirn, 1e-4, 1e6};
        Intersection isc;
        bool is_hit = scene->ray_intersect(ray, isc);
        if (!is_hit)
            return;

        Vec3f beta = lightLe;
        // TODO: Is this correct?
        beta *= std::abs(glm::dot(light_normal, light_dirn));
        beta /= light_pdf;
        prefix_iscs.push_back(isc);
        prefix_betas.push_back(beta);

        for (int depth = 1; depth < max_length || max_length < 0; depth++) {
            auto [bsdf_sample, bsdf_value] = isc.shape->bsdf->sample(isc, sampler->get_1D(), sampler->get_2D());
            ray = Ray{isc.position + sign(glm::dot(isc.normal, bsdf_sample.wo)) * Epsilon * isc.normal, bsdf_sample.wo, ray.tmin, ray.tmax};
            is_hit = scene->ray_intersect(ray, isc);
            if (!is_hit || bsdf_sample.pdf == 0)
                return;
            beta *= bsdf_value / bsdf_sample.pdf;
            prefix_iscs.push_back(isc);
            prefix_betas.push_back(beta);
        }
    }

    // whether there's an occlusion , delta bsdf, and surfaces being oriented
    bool is_connection_valid(const Scene *scene, const Intersection &cam_isc, const Intersection &light_isc) const {
        Vec3f dirn = glm::normalize(light_isc.position - cam_isc.position);
        Float distance = glm::length(light_isc.position - cam_isc.position);

        // Check for occlusion
        Intersection tmp_isc;
        bool is_occluded = scene->ray_intersect(Ray{cam_isc.position + sign(glm::dot(dirn, cam_isc.normal)) * Epsilon * cam_isc.normal, dirn, 1e-4, distance - 1e-3f, true}, tmp_isc);
        if (is_occluded)
            return false;

        // Check for delta bsdf
        if (cam_isc.shape->bsdf->has_flag(BSDFFlags::Delta) ||
            (light_isc.shape && light_isc.shape->bsdf->has_flag(BSDFFlags::Delta)))
            return false;

        // Check for surfaces facing each other
        if (!cam_isc.shape->bsdf->has_flag(BSDFFlags::TwoSided) && glm::dot(dirn, cam_isc.normal) <= 0.0)
            return false;
        if (light_isc.shape && !light_isc.shape->bsdf->has_flag(BSDFFlags::TwoSided) && glm::dot(-dirn, light_isc.normal) <= 0.0)
            return false;

        return true;
    }

public:
    BidirectionalIntegrator(int max_depth) : max_depth(max_depth) {}

    Vec3f sample_radiance(const Scene *scene, Sampler *sampler, const Ray &sensor_ray, int row, int col) const override {
        // ---------------- Sample a camera path ----------------
        int cam_max_length = 10;  // max number of segments
        std::vector<Intersection> cam_prefix_iscs{};
        std::vector<Vec3f> cam_prefix_betas{};
        createCameraPath(scene, sampler, sensor_ray, cam_prefix_iscs, cam_prefix_betas, cam_max_length);
        int n_cam_vertices = cam_prefix_betas.size();
        if (n_cam_vertices == 1)
            return Vec3f{0};

        // ---------------- Sample a  light path ----------------
        int light_max_length = 10;  // max number of segments
        std::vector<Intersection> light_prefix_iscs{};
        std::vector<Vec3f> light_prefix_betas{};
        createLightPath(scene, sampler, light_prefix_iscs, light_prefix_betas, light_max_length);
        int n_light_vertices = light_prefix_betas.size();

        // ------------------- Connect  paths -------------------
        // randomly select two vertieces from each subpath, and compute its contribution
        int cam_vertex_idx = uniformDiscrete(sampler->get_1D(), 1, n_cam_vertices - 1);
        int light_vertex_idx = uniformDiscrete(sampler->get_1D(), 0, n_light_vertices - 1);
        Intersection cam_isc = cam_prefix_iscs.at(cam_vertex_idx);
        Intersection light_isc = light_prefix_iscs.at(light_vertex_idx);

        Vec3f dirn = glm::normalize(light_isc.position - cam_isc.position);
        Float distance = glm::length(light_isc.position - cam_isc.position);
        if (!is_connection_valid(scene, cam_isc, light_isc))
            return Vec3f{0};

        Vec3f cam_bsdfval = cam_isc.shape->bsdf->eval(cam_isc, worldToLocal(dirn, cam_isc.normal));
        // manually handle the case where n_light_vertices == 1
        Vec3f light_bsdfval{0};
        if (light_vertex_idx == 0)
            light_bsdfval = light_prefix_betas.at(0) * std::abs(glm::dot(dirn, light_isc.normal));
        else
            light_bsdfval = light_isc.shape->bsdf->eval(light_isc, worldToLocal(-dirn, light_isc.normal));

        Vec3f cam_beta = cam_prefix_betas.at(cam_vertex_idx);
        Vec3f light_beta = light_prefix_betas.at(light_vertex_idx);
        Vec3f total_contrib = cam_beta * light_beta * cam_bsdfval * light_bsdfval / Sqr(distance);

        return total_contrib;
    }

    std::string to_string() const override {
        return "BD to_string not implemented yet!";
    }
};

// ------------------- Registry functions -------------------
Integrator *createBidirectionalIntegrator(const std::unordered_map<std::string, std::string> &properties) {
    int max_depth = 10;

    for (const auto &[key, value] : properties) {
        if (key == "max_depth") {
            max_depth = std::stoi(value);
        } else {
            throw std::runtime_error("Unknown property '" + key + "' for Path Tracer integrator");
        }
    }

    return new BidirectionalIntegrator(max_depth);
}

namespace {
struct BidirectionalIntegratorRegistrar {
    BidirectionalIntegratorRegistrar() {
        IntegratorRegistry::registerIntegrator("bidirectional", createBidirectionalIntegrator);
    }
};

static BidirectionalIntegratorRegistrar registrar;
}  // namespace
