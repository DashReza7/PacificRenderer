#include "core/Integrator.h"
#include "core/MathUtils.h"
#include "core/Scene.h"

class BidirectionalIntegrator : public SamplingIntegrator {
private:
    int max_depth;
    bool hide_emitters;  // TODO: not working yet!

    // prefix.size is the number of vertices. 1 means only the camera vertex.
    void createCameraPath(const Scene *scene, Sampler *sampler, const Ray &ray,
                          std::vector<Intersection> &path_iscs, std::vector<Vec3f> &path_betas,
                          std::vector<Float> &path_probs, int row, int col, int max_length = -1) const {
        
        Intersection cam_isc{};
        cam_isc.position = scene->sensor->origin_world;
        cam_isc.normal = glm::normalize(Vec3f{scene->sensor->to_world * Vec4f{0, 0, 1, 0}});
        path_iscs.push_back(cam_isc);
        // TODO: make sure this doesn't fail anything
        path_betas.push_back(Vec3f{1});
        path_probs.push_back(1.0);

        Ray cur_ray{ray};
        Intersection isc;

        bool is_hit = scene->ray_intersect(cur_ray, isc);
        if (!is_hit)
            return;
        Vec3f We = scene->sensor->We(ray.d);
        Float pdf_we, unused;
        scene->sensor->pdf_We(ray.d, unused, pdf_we);
        if (pdf_we == 0 || We == Vec3f{0})  // This happens for some border pixels (not important. ignore)
            return;
        Vec3f beta = Vec3f{We / pdf_we};
        
        path_iscs.push_back(isc);
        path_betas.push_back(beta);
        path_probs.push_back(pdf_we);            
        
        for (int depth = 1; depth < max_length || max_length < 0; depth++) {
            auto [bsdf_sample, bsdf_value] = isc.shape->bsdf->sample(isc, sampler->get_1D(), sampler->get_2D());
            Vec3f wo = localToWorld(bsdf_sample.wo, isc.normal);
            cur_ray = Ray{isc.position + sign(glm::dot(isc.normal, wo)) * Epsilon * isc.normal,
                          wo, 1e-2, cur_ray.tmax};
            is_hit = scene->ray_intersect(cur_ray, isc);
            if (!is_hit || bsdf_sample.pdf == 0)
                return;
            Float dist = glm::length(isc.position - path_iscs.at(path_iscs.size()-1).position);
            beta *= bsdf_value / bsdf_sample.pdf;
            path_iscs.push_back(isc);
            path_betas.push_back(beta);
            path_probs.push_back(path_probs.at(path_probs.size()-1)
                                 * bsdf_sample.pdf * std::abs(glm::dot(isc.normal, ray.d))
                                 / Sqr(dist));

            // DEBUG
            if (!check_valid(path_probs.at(path_probs.size()-1))) {
                std::cout << "pp in cam_path: " << path_probs.at(path_probs.size()-1) << std::endl;
                exit(EXIT_FAILURE);
            }
            if (!check_valid(path_betas.at(path_betas.size()-1))) {
                std::cout << "cam_beta in cam_path: " << path_betas.at(path_betas.size()-1) << std::endl;
                exit(EXIT_FAILURE);
            }
        }
    }

    // prefix.size is the number of vertices. 1 means only the camera vertex.
    void createLightPath(const Scene *scene, Sampler *sampler,
                         std::vector<Intersection> &path_iscs, std::vector<Vec3f> &path_betas,
                         std::vector<Float> &path_probs, int max_length = -1) const {

        Vec3f light_posn, light_normal, light_dirn;
        Float light_pdf_posn, light_pdf_dirn;
        const Shape *light_shape;
        Vec3f lightLe = scene->sampleEmitter(sampler->get_2D(), sampler->get_3D(), sampler->get_1D(),
                                                     light_posn, light_normal, light_dirn, light_shape,
                                                     light_pdf_posn, light_pdf_dirn);
        Intersection light_isc{};
        light_isc.position = light_posn;
        light_isc.normal = light_normal;
        light_isc.shape = light_shape;
        path_iscs.push_back(light_isc);
        path_betas.push_back(lightLe / light_pdf_posn);
        path_probs.push_back(light_pdf_posn);

        // first intersection
        Ray ray{light_posn + Epsilon * light_normal, light_dirn, 1e-2, 1e6};
        Intersection isc;
        bool is_hit = scene->ray_intersect(ray, isc);
        if (!is_hit)
            return;

        Vec3f beta = lightLe * std::abs(glm::dot(light_normal, light_dirn)) 
                             / (light_pdf_posn * light_pdf_dirn);
        path_iscs.push_back(isc);
        path_betas.push_back(beta);
        path_probs.push_back(path_probs.at(0) * light_pdf_dirn 
                                              * std::abs(glm::dot(light_dirn, isc.normal)) 
                                              / Sqr(glm::length(isc.position - light_posn)));
        
        // DEBUG
        if (!check_valid(path_probs.at(path_probs.size() - 1))) {
            std::cout << "pp in light path: " << path_probs.at(path_probs.size() - 1) << std::endl;
            exit(EXIT_FAILURE);
        }

        for (int depth = 1; depth < max_length || max_length < 0; depth++) {
            auto [bsdf_sample, bsdf_value] = isc.shape->bsdf->sample(isc, sampler->get_1D(), sampler->get_2D());
            Vec3f wo = localToWorld(bsdf_sample.wo, isc.normal);
            ray = Ray{isc.position + sign(glm::dot(isc.normal, wo)) * Epsilon * isc.normal,
                      wo, ray.tmin, ray.tmax};
            is_hit = scene->ray_intersect(ray, isc);
            if (!is_hit || bsdf_sample.pdf == 0)
                return;
            Float dist = glm::length(isc.position - path_iscs.at(path_iscs.size()-1).position);
            beta *= bsdf_value / bsdf_sample.pdf;
            path_iscs.push_back(isc);
            path_betas.push_back(beta);
            path_probs.push_back(path_probs.at(path_probs.size()-1) 
                                 * bsdf_sample.pdf * std::abs(glm::dot(isc.normal, ray.d))
                                 / Sqr(dist));

            // DEBUG
            if (!check_valid(path_probs.at(path_probs.size()-1))) {
                std::cout << "pp in light path: " << path_probs.at(path_probs.size()-1) << std::endl;
                exit(EXIT_FAILURE);
            }
            if (!check_valid(path_betas.at(path_betas.size() - 1))) {
                std::cout << "\ninvalid light beta: " << path_betas.at(path_betas.size() - 1) << std::endl;
                exit(EXIT_FAILURE);
            }
        }
    }

    // return the path contribution
    Vec3f connect_bdpt(const Scene *scene, 
                       const std::vector<Intersection> &cam_iscs, const std::vector<Intersection> &light_iscs,
                       const std::vector<Vec3f> &cam_betas, const std::vector<Vec3f> &light_betas,
                       const std::vector<Float> &cam_probs, const std::vector<Float> &light_probs,
                       int s, int t, Float &mis_weight, Vec2f &p_new) const {

        mis_weight = 0;
        p_new = Vec2f{-1};
        int cam_idx = t - 1;
        int light_idx = s - 1;
        
        Intersection cam_isc = cam_iscs.at(cam_idx);
        Intersection light_isc = light_iscs.at(light_idx);
        // dirn from cam_vertex to light_vertex
        Vec3f dirn = glm::normalize(light_isc.position - cam_isc.position);
        Float dist = glm::length(light_isc.position - cam_isc.position);

        // Check for delta bsdf
        if ((cam_isc.shape && cam_isc.shape->bsdf->has_flag(BSDFFlags::Delta)) ||
            light_isc.shape->bsdf->has_flag(BSDFFlags::Delta))
                return Vec3f{0};
        // Check for visibility
        Intersection tmp_isc;
        bool is_occluded = scene->ray_intersect(Ray{cam_isc.position + sign(glm::dot(cam_isc.normal, dirn)) * Epsilon * cam_isc.normal , dirn, 1e-2, dist - 1e-2f, true},
                                                tmp_isc);
        if (is_occluded)
            return Vec3f{0};
        // check for correct orientation of area light
        if (s == 1 && glm::dot(-dirn, light_isc.normal) <= 0)
            return Vec3f{0};
        if (cam_isc.shape && !cam_isc.shape->bsdf->has_flag(BSDFFlags::TwoSided) && glm::dot(dirn, cam_isc.normal) <= 0.0)
            return Vec3f{0};
        if (light_isc.shape && !light_isc.shape->bsdf->has_flag(BSDFFlags::TwoSided) && glm::dot(-dirn, light_isc.normal) <= 0.0)
            return Vec3f{0};
        // check if the connection is inside the view spectrum
        Float unused;
        Vec2f unused2; Vec3f unused3;
        if (t == 1) {
            if(!scene->sensor->worldToIplane(dirn, unused2, unused3))
                return Vec3f{0};
        }
        
        // compute the unweighted contribution
        Vec3f cam_bsdfval;
        if (t == 1) {
            Float pdf_we;
            scene->sensor->pdf_We(dirn, unused, pdf_we);
            if (pdf_we == 0)
                return Vec3f{0};
            cam_bsdfval = scene->sensor->We(dirn) / pdf_we ;  // TODO: should I divide by its pdf???
        } else {
            cam_bsdfval = cam_iscs.at(cam_idx).shape->bsdf->eval(cam_isc, worldToLocal(dirn, cam_isc.normal));
        }
        Vec3f light_bsdfval = s != 1 ?
                                light_iscs.at(light_idx).shape->bsdf->eval(light_isc, worldToLocal(-dirn, light_isc.normal))
                              : Vec3f{std::abs(glm::dot(light_isc.normal, dirn))};
        Vec3f L = cam_betas.at(cam_idx) * light_betas.at(light_idx) * cam_bsdfval * light_bsdfval / Sqr(dist);
        if (L == Vec3f{0})
            return L;
        // DEBUG
        if (!check_valid(L)) {
            std::cout << "\nThe unweighted contribution is invalid: " << L << ". cam_bsdfval: " << cam_bsdfval << ". light_bsdfval: " << light_bsdfval << ". dist: " << dist << "cam_beta: " << cam_betas.at(cam_idx) << ". light_beta: " << light_betas.at(light_idx) << ". Ignoring the path.\n";
            // return Vec3f{0};
            exit(EXIT_FAILURE);
        }
            
        // Compute probs
        Float p_s = cam_probs.at(cam_idx) * light_probs.at(light_idx);
        // DEBUG
        if (std::isnan(p_s) || std::isinf(p_s) || p_s < 0) {
            std::cout << "invalid p_s: " << p_s << std::endl;
            exit(EXIT_FAILURE);
        }

        std::vector<Float> other_tech_probs{};
        for (int i = s+1; i <= s+t-2; i++) {
            // TODO: handle i=s+t-1
            
            Intersection prev_isc = (i == s+1) ? light_isc : cam_iscs.at(cam_idx - (i - (s+2)));
            Intersection cur_isc  = cam_iscs.at(cam_idx - (i - (s+1)));
            Intersection next_isc = cam_iscs.at(cam_idx - (i - s));
            Vec3f dirn1 = glm::normalize(cur_isc.position - prev_isc.position);
            Vec3f dirn2 = glm::normalize(cur_isc.position - next_isc.position);
            Float dist1 = glm::length(cur_isc.position - prev_isc.position);
            Float dist2 = glm::length(cur_isc.position - next_isc.position);
            // TODO: Is this kind of delta bsdf handling correct?
            // TODO: fix this. right now it's hardcoded
            Float p1 = i == 2 ? cosineHemispherePDF(worldToLocal(dirn1, light_iscs.at(0).normal), worldToLocal(dirn1, light_iscs.at(0).normal))
                     : prev_isc.shape->bsdf->has_flag(BSDFFlags::Delta) ? 1.0
                     : prev_isc.shape->bsdf->pdf(prev_isc, worldToLocal(dirn1, prev_isc.normal));
            Float p2;
            if (i == s+t-1)
                scene->sensor->pdf_We(dirn2, unused, p2);
            else
                p2 = next_isc.shape->bsdf->has_flag(BSDFFlags::Delta) ? 1.0
                    : next_isc.shape->bsdf->pdf(next_isc, worldToLocal(dirn2, next_isc.normal));
            Float cos1 = std::abs(glm::dot(cur_isc.normal, dirn1));
            Float cos2 = std::abs(glm::dot(cur_isc.normal, dirn2));
            Float prev_prob = (i == s+1) ? p_s : other_tech_probs.at(other_tech_probs.size()-1);

            Float cur_prob = prev_prob / (p1 * cos1) * (p2 * cos2) * Sqr(dist1 / dist2);
            if (p1 == 0 || p2 == 0) {
                std::cout << "\nfirst loop(s=" << s << ", i=" << i << "): p1=" << p1 << ". p2=" << p2 << ". considering cur_prob=0\n";
                cur_prob = 0;
            }
            
            // DEBUG
            if (std::isnan(cur_prob) || std::isinf(cur_prob) || cur_prob < 0) {
                // FIXME: problem with p2 being 0 (for a rough conductor bsdf)
                std::cout << "\ninvalid cur_prob in the first loop: " << cur_prob << ". considering it 0\n";
                cur_prob = 0.0;

                // std::cout << "invalid prob in the first loop: " << cur_prob << std::endl;
                // std::cout << "dirn1: " << dirn1 << std::endl;
                // std::cout << "dirn2: " << dirn2 << std::endl;
                // std::cout << "dist1: " << dist1 << std::endl;
                // std::cout << "dist2: " << dist2 << std::endl;
                // std::cout << "p1: " << p1 << std::endl;
                // std::cout << "p2: " << p2 << std::endl;
                // std::cout << "cos1: " << cos1 << std::endl;
                // std::cout << "cos2: " << cos2 << std::endl;
                // std::cout << "prev_prob: " << prev_prob << std::endl; 
                // exit(EXIT_FAILURE);
            }
            
            other_tech_probs.push_back(cur_prob);
        }
        for (int i = s-1; i >= 1; i--) {
            Intersection prev_isc = light_iscs.at(i - 1);
            Intersection cur_isc  = light_iscs.at(i);
            Intersection next_isc = (i == s-1) ? cam_isc : light_iscs.at(i + 1);
            Vec3f dirn1 = glm::normalize(cur_isc.position - prev_isc.position);
            Vec3f dirn2 = glm::normalize(cur_isc.position - next_isc.position);
            Float dist1 = glm::length(cur_isc.position - prev_isc.position);
            Float dist2 = glm::length(cur_isc.position - next_isc.position);
            // TODO: Is this kind of delta bsdf handling correct?
            // TODO: fix this. right now it's hardcoded
            Float p1 = i == 1 ? cosineHemispherePDF(worldToLocal(dirn1, light_iscs.at(0).normal), worldToLocal(dirn1, light_iscs.at(0).normal))
                     : prev_isc.shape->bsdf->has_flag(BSDFFlags::Delta) ? 1.0
                     : prev_isc.shape->bsdf->pdf(prev_isc, worldToLocal(dirn1, prev_isc.normal));
            
            Float p2;
            if (i == s-1 && t == 1)
                scene->sensor->pdf_We(dirn2, unused, p2);
            else
                p2 = next_isc.shape->bsdf->has_flag(BSDFFlags::Delta) ? 1.0
                    : next_isc.shape->bsdf->pdf(next_isc, worldToLocal(dirn2, next_isc.normal));
            Float cos1 = std::abs(glm::dot(cur_isc.normal, dirn1));
            Float cos2 = std::abs(glm::dot(cur_isc.normal, dirn2));
            Float next_prob = (i == s-1) ? p_s : other_tech_probs.at(other_tech_probs.size()-1);
            
            Float cur_prob = next_prob * (p1 * cos1) / (p2 * cos2) * Sqr(dist2 / dist1);
            if (p2 == 0 || p1 == 0) {
                std::cout << "\nsecond loop(s=" << s << ", i=" << i << "): p1=" << "p1=" << p1 << ". p2=" << p2 << ". considering cur_prob=0\n";
                cur_prob = 0;
            }
            
            // DEBUG
            if (std::isnan(cur_prob) || std::isinf(cur_prob) || cur_prob < 0) {
                // FIXME: problem with p2 being 0 (for a rough conductor bsdf)
                std::cout << "\ninvalid cur_prob in the second loop: " << cur_prob << ". considering it 0\n";
                cur_prob = 0.0;
                
                // std::cout << "invalid prob in the second loop: " << cur_prob << std::endl;
                std::cout << "s=" << s << ", i=" << i << std::endl;
                std::cout << "dist1: " << dist1 << std::endl;
                std::cout << "dist2: " << dist2 << std::endl;
                std::cout << "p1: " << p1 << std::endl;
                std::cout << "p2: " << p2 << std::endl;
                std::cout << "cos1: " << cos1 << std::endl;
                std::cout << "cos2: " << cos2 << std::endl;
                std::cout << "next_prob: " << next_prob << std::endl; 
                std::cout << "next_isc bsdf: " << next_isc.shape->bsdf->to_string() << std::endl;
                // exit(EXIT_FAILURE);
            }

            other_tech_probs.push_back(cur_prob);
        }
        
        // compute the mis_weight with power heuristic (beta=2)
        Float denom = Sqr(p_s);
        for (Float prob : other_tech_probs)
            denom += Sqr(prob);
        mis_weight = Sqr(p_s) / denom;
        //TODO:
        if (p_s < 1e-5)
            mis_weight = 0;
        else if (p_s > 1e5)
            mis_weight = 1;
        // DEBUG
        if (!check_valid(mis_weight)) {
            std::cout << "\ninvalid mis_weight: " << mis_weight << ". p_s: " << p_s << "\n";
            exit(EXIT_FAILURE);
        }
        
        if (t == 1)
            scene->sensor->worldToIplane(dirn, p_new, unused3);
        return L;
    }
    
public:
    BidirectionalIntegrator(int max_depth, bool hide_emitters) : max_depth(max_depth), hide_emitters(hide_emitters) {}

    Vec3f sample_radiance(const Scene *scene, Sampler *sampler, 
                          const Ray &sensor_ray, int row, int col) const override {
        // ---------------- Sample a camera path ----------------
        int cam_max_length = 7;  // max number of segments
        std::vector<Intersection> cam_path_iscs{};
        std::vector<Vec3f> cam_path_betas{};
        std::vector<Float> cam_path_probs{};
        createCameraPath(scene, sampler, sensor_ray, cam_path_iscs, cam_path_betas, cam_path_probs, row, col, cam_max_length);
        int n_cam_vertices = cam_path_betas.size();
        if (n_cam_vertices == 1)
            return Vec3f{0};

        // ---------------- Sample a  light path ----------------
        int light_max_length = 7;  // max number of segments
        std::vector<Intersection> light_path_iscs{};
        std::vector<Vec3f> light_path_betas{};
        std::vector<Float> light_path_probs{};
        createLightPath(scene, sampler, light_path_iscs, light_path_betas, light_path_probs, light_max_length);
        int n_light_vertices = light_path_betas.size();

        // ------------------- Connect  paths -------------------
        Vec3f L{0};
        for (int s = 1; s <= n_light_vertices; s++) {
            for (int t = 2; t <= n_cam_vertices; t++) {
                int k = s + t - 1;
                if (k > max_depth)
                    continue;
                Float mis_weight;
                Vec2f p_new;
                Vec3f contrib = connect_bdpt(scene, cam_path_iscs, light_path_iscs,
                                             cam_path_betas, light_path_betas,
                                             cam_path_probs, light_path_probs,
                                             s, t, mis_weight, p_new);
                if (p_new.x < 0 || p_new.y < 0)
                    L += contrib * mis_weight;
                else
                    scene->sensor->film.commit_splat(contrib * mis_weight / (Float)(scene->sensor->sampler.spp) , p_new);
            }
        }
        return L;
    }

    std::string to_string() const override {
        return "BD to_string not implemented yet!";
    }
};

// ------------------- Registry functions -------------------
Integrator *createBidirectionalIntegrator(const std::unordered_map<std::string, std::string> &properties) {
    int max_depth = 10;
    bool hide_emitters = false;

    for (const auto &[key, value] : properties) {
        if (key == "max_depth") {
            max_depth = std::stoi(value);
        } else if (key == "hide_emitters") {
            hide_emitters = (value == "true" || value == "1");
        } else {
            throw std::runtime_error("Unknown property '" + key + "' for Path Tracer integrator");
        }
    }

    return new BidirectionalIntegrator(max_depth, hide_emitters);
}

namespace {
struct BidirectionalIntegratorRegistrar {
    BidirectionalIntegratorRegistrar() {
        IntegratorRegistry::registerIntegrator("bidirectional", createBidirectionalIntegrator);
    }
};

static BidirectionalIntegratorRegistrar registrar;
}  // namespace
