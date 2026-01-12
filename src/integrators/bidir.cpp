#include "core/Integrator.h"
#include "core/MathUtils.h"
#include "core/Scene.h"

enum class VertexType {
    CAMERA_VERTEX = 1 << 0,
    LIGHT_VERTEX = 1 << 1,
    SURFACE_VERTEX = 1 << 2
};
enum class PathType {
    CAMERA_PATH,
    LIGHT_PATH
};
class Vertex {
public:
    VertexType vertex_type;
    PathType path_type;
    Intersection isc;
    Vec3f wi, wo;
    Vec3f cum_beta;
    Float cum_pdf;

    Vertex(VertexType vtype, PathType ptype) : vertex_type(vtype), path_type(ptype) {}
};

class BidirIntegrator : public SamplingIntegrator {
private:
    int max_depth;
    bool hide_emitters;

    void randomWalk(std::vector<Vertex> &path,
                    const Scene *scene, Ray &ray, Intersection &isc,
                    Sampler *sampler, int max_depth, PathType path_type) const;

    void createCamPath(std::vector<Vertex> &cam_path, const Scene *scene, Sampler *sampler,
                       const Ray &sensor_ray, int max_vertices) const;

    void createLightPath(std::vector<Vertex> &light_path, const Scene *scene, Sampler *sampler,
                         int max_vertices) const;

    Vec3f connectPaths(const std::vector<Vertex> &cam_path, const std::vector<Vertex> &light_path,
                       const Scene *scene, int s, int t, Float &mis_weight, Vec2f &pfilm_new,
                       Sampler *sampler) const;

    bool isConnValid(const std::vector<Vertex> &cam_path, const std::vector<Vertex> &light_path,
                     const Scene *scene, int cam_idx, int light_idx) const;

    Float getMisWeight(const std::vector<Vertex> &cam_path, const std::vector<Vertex> &light_path,
                       const Scene *scene, int s, int t) const;

public:
    BidirIntegrator(int max_depth, bool hide_emitters) : max_depth(max_depth), hide_emitters(hide_emitters) {}

    Vec3f sample_radiance(const Scene *scene, Sampler *sampler, const Ray &ray, int row, int col) const override {
        // --------------------- Sample a CAMERA path ---------------------
        int cam_max_length = 5;
        std::vector<Vertex> cam_path{};
        createCamPath(cam_path, scene, sampler, ray, cam_max_length);
        int n_cam_vertices = cam_path.size();
        // TODO: this is just for when we don't consider t=1
        if (n_cam_vertices == 1)
            return Vec3f{0};

        // --------------------- Sample a LIGHT  path ---------------------
        int light_max_length = 5;
        std::vector<Vertex> light_path{};
        createLightPath(light_path, scene, sampler, light_max_length);
        int n_light_vertices = light_path.size();

        // ------------------------ Connect  paths ------------------------
        Vec3f L{0};
        for (int s = 1; s <= n_light_vertices; s++) {
            for (int t = 2; t <= n_cam_vertices; t++) {
                int k = s + t - 1;
                // TODO if (k > max_depth)  rightnow ignore
                //     continue;
                Vec2f pfilm_new{-1};
                Float mis_weight = 0;
                Vec3f contrib = connectPaths(cam_path, light_path, scene, s, t, mis_weight, pfilm_new, sampler);
                // TODO: right now ignore pfilm_new. this is for direct connection of light vertex to camera
                L += contrib * mis_weight;
            }
        }
        return L;
    }

    std::string to_string() const override {
        throw std::runtime_error("BidirIntegrator to_string not implemented yet!");
    }
};

// ------------------- Registry functions -------------------
Integrator *createBidirIntegrator(const std::unordered_map<std::string, std::string> &properties) {
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

    return new BidirIntegrator(max_depth, hide_emitters);
}

namespace {
struct BidirIntegratorRegistrar {
    BidirIntegratorRegistrar() {
        IntegratorRegistry::registerIntegrator("bidir", createBidirIntegrator);
    }
};

static BidirIntegratorRegistrar registrar;
}  // namespace

// ------------------ BDPT function definitions ------------------------

void BidirIntegrator::randomWalk(std::vector<Vertex> &path,
                                 const Scene *scene, Ray &ray, Intersection &isc,
                                 Sampler *sampler, int max_depth, PathType path_type) const {
    bool is_hit;
    Vertex v{VertexType::SURFACE_VERTEX, path_type};
    for (int i = 0; i < max_depth; i++) {
        auto [bsdf_sample, bsdf_value] = isc.shape->bsdf->sample(isc, sampler->get_1D(), sampler->get_2D());

        if (bsdf_sample.pdf == 0 || bsdf_value == Vec3f{0})
            return;

        Vec3f bsdf_wo = localToWorld(bsdf_sample.wo, isc.normal);
        ray = Ray{isc.position + sign(glm::dot(isc.normal, bsdf_wo))*Epsilon*isc.normal, bsdf_wo, ray.tmin, ray.tmax};
        is_hit = scene->ray_intersect(ray, isc);
        if (!is_hit || bsdf_sample.pdf == 0 || bsdf_value == Vec3f{0})
            return;

        // update the previous vertex params
        path[path.size() - 1].wo = bsdf_sample.wo;
        Vertex v_prev = path.at(path.size() - 1);

        v = Vertex{VertexType::SURFACE_VERTEX, path_type};
        v.isc = isc;
        v.wi = isc.dirn;
        v.wo = Vec3f{0};  // Updated at the next vertex
        v.cum_beta = v_prev.cum_beta * bsdf_value / bsdf_sample.pdf;
        v.cum_pdf = v_prev.cum_pdf * bsdf_sample.pdf * absDot(isc.normal, isc.dirn) / Sqr(isc.distance);
        path.push_back(v);
    }
}

void BidirIntegrator::createCamPath(std::vector<Vertex> &cam_path, const Scene *scene, Sampler *sampler,
                                    const Ray &sensor_ray, int max_vertices) const {
    if (max_vertices <= 0)
        return;
    Vertex camVertex{VertexType::CAMERA_VERTEX, PathType::CAMERA_PATH};
    Intersection cam_isc;
    cam_isc.position = scene->sensor->origin_world;
    cam_isc.normal = scene->sensor->forward_world;
    cam_isc.dirn = sensor_ray.d;
    camVertex.isc = cam_isc;
    camVertex.wi = Vec3f{0};
    camVertex.wo = sensor_ray.d;
    camVertex.cum_beta = Vec3f{1};  // TODO: Is this and the next line correct?
    camVertex.cum_pdf = 1.0;
    cam_path.push_back(camVertex);
    if (max_vertices == 1)
        return;

    Ray ray{sensor_ray};
    Intersection isc;
    bool is_hit = scene->ray_intersect(ray, isc);
    if (!is_hit)
        return;

    Vertex v{VertexType::SURFACE_VERTEX, PathType::CAMERA_PATH};
    v.isc = isc;
    v.wi = -sensor_ray.d;
    v.wo = Vec3f{0};  // Updated at the next vertex
    Vec3f We = scene->sensor->We(sensor_ray.d);
    Float pdf_We, unused;
    scene->sensor->pdf_We(sensor_ray.d, unused, pdf_We);
    // FIXME: This happens for some border pixels. Ignore for now
    if (We == Vec3f{0} || pdf_We == 0)
        return;
    // TODO: I'm not sure if the next two lines are correct
    v.cum_beta = Vec3f{We / pdf_We};
    v.cum_pdf = pdf_We * absDot(isc.normal, isc.dirn) / Sqr(isc.distance);
    cam_path.push_back(v);

    randomWalk(cam_path, scene, ray, isc, sampler, max_vertices - 2, PathType::CAMERA_PATH);
}

void BidirIntegrator::createLightPath(std::vector<Vertex> &light_path, const Scene *scene, Sampler *sampler,
                                      int max_vertices) const {
    if (max_vertices <= 0)
        return;
    Vertex lightVertex{VertexType::LIGHT_VERTEX, PathType::LIGHT_PATH};
    // sample a point on a light
    Float pdf_light_posn, pdf_light_dirn;
    Vec3f light_Le = scene->sampleEmitter(sampler->get_2D(), sampler->get_3D(), sampler->get_1D(),
                                          lightVertex.isc.position, lightVertex.isc.normal, lightVertex.isc.dirn,
                                          lightVertex.isc.shape, pdf_light_posn, pdf_light_dirn);
    lightVertex.wi = Vec3f{0};
    lightVertex.wo = lightVertex.isc.dirn;
    lightVertex.cum_beta = light_Le / pdf_light_posn;
    lightVertex.cum_pdf = pdf_light_posn;
    light_path.push_back(lightVertex);
    if (max_vertices == 1)
        return;

    Ray ray{lightVertex.isc.position + Epsilon * lightVertex.isc.normal, lightVertex.isc.dirn, 1e-3, 1e6};
    Intersection isc;
    bool is_hit = scene->ray_intersect(ray, isc);
    if (!is_hit)
        return;

    Vertex v{VertexType::SURFACE_VERTEX, PathType::LIGHT_PATH};
    v.isc = isc;
    v.wi = -lightVertex.wo;
    v.wo = Vec3f{0};  // Updated at the next vertex
    v.cum_beta = light_Le * absDot(lightVertex.isc.normal, lightVertex.isc.dirn) / (pdf_light_posn * pdf_light_dirn);
    v.cum_pdf = pdf_light_posn * pdf_light_dirn * absDot(isc.normal, isc.dirn) / Sqr(isc.distance);
    light_path.push_back(v);

    randomWalk(light_path, scene, ray, isc, sampler, max_vertices - 2, PathType::LIGHT_PATH);
}

Vec3f BidirIntegrator::connectPaths(const std::vector<Vertex> &cam_path, const std::vector<Vertex> &light_path,
                                    const Scene *scene, int s, int t, Float &mis_weight, Vec2f &pfilm_new,
                                    Sampler *sampler) const {
    if (!isConnValid(cam_path, light_path, scene, t - 1, s - 1))
        return Vec3f{0};
        
    int cam_idx = t - 1;
    int light_idx = s - 1;
    Vertex vcam = cam_path.at(cam_idx);
    Vertex vlight = light_path.at(light_idx);
    Vec3f dirn = glm::normalize(vlight.isc.position - vcam.isc.position);
    Float dist = glm::length(vlight.isc.position - vcam.isc.position);
    // ------------------- Compute the contribution of the path -------------------
    Vec3f cam_bsdfval = vcam.isc.shape->bsdf->eval(vcam.isc, worldToLocal(dirn, vcam.isc.normal));
    Vec3f light_bsdfval = vlight.vertex_type == VertexType::LIGHT_VERTEX 
                        ? Vec3f{absDot(vlight.isc.normal, dirn)} 
                        : vlight.isc.shape->bsdf->eval(vlight.isc, worldToLocal(-dirn, vlight.isc.normal));
    Vec3f L = vcam.cum_beta * vlight.cum_beta * cam_bsdfval * light_bsdfval / Sqr(dist);

    mis_weight = getMisWeight(cam_path, light_path, scene, s, t);

    return L;
}

bool BidirIntegrator::isConnValid(const std::vector<Vertex> &cam_path, const std::vector<Vertex> &light_path,
                                  const Scene *scene, int cam_idx, int light_idx) const {
    Vertex camVertex = cam_path.at(cam_idx);
    Vertex lightVertex = light_path.at(light_idx);

    // Check for delta BSDFs
    if ((camVertex.isc.shape && camVertex.isc.shape->bsdf->has_flag(BSDFFlags::Delta)) ||
        (lightVertex.isc.shape && lightVertex.isc.shape->bsdf->has_flag(BSDFFlags::Delta)))
        return false;

    Vec3f dirn = glm::normalize(lightVertex.isc.position - camVertex.isc.position);
    // Check for ray being in the view frustum
    if (camVertex.vertex_type == VertexType::CAMERA_VERTEX) {
        Vec2f unused2;
        Vec3f unused3;
        if (!scene->sensor->worldToIplane(dirn, unused2, unused3))
            return false;
    }

    // check for correct orientation of area lights
    if (lightVertex.vertex_type == VertexType::LIGHT_VERTEX && lightVertex.isc.shape &&
        glm::dot(-dirn, lightVertex.isc.normal) <= 0)
        return false;
    
    // Check for correct orientation of vertices
    if (camVertex.isc.shape && !camVertex.isc.shape->bsdf->has_flag(BSDFFlags::TwoSided) &&
        glm::dot(dirn, camVertex.isc.normal) <= 0)
        return false;
    if (lightVertex.isc.shape && !lightVertex.isc.shape->bsdf->has_flag(BSDFFlags::TwoSided) &&
        glm::dot(-dirn, lightVertex.isc.normal) <= 0)
        return false;

    // Check for occlusion
    Float dist = glm::length(lightVertex.isc.position - camVertex.isc.position);
    Ray occl_ray{camVertex.isc.position + sign(glm::dot(camVertex.isc.normal, dirn)) * Epsilon * dirn, dirn,
                 1e-3, dist - 1e-3f, true};
    Intersection tmp_isc;
    if (scene->ray_intersect(occl_ray, tmp_isc))
        return false;

    return true;
}

Float BidirIntegrator::getMisWeight(const std::vector<Vertex> &cam_path, const std::vector<Vertex> &light_path,
                                    const Scene *scene, int s, int t) const {
    // ------------------------- Compute relative probs -------------------------
    std::vector<Float> rel_probs;
    for (int i = s+1; i <= s+t-2; i++) {
        Vertex vprev = i == s+1 ? light_path.at(s-1) : cam_path.at(s+t+1-i);
        Vertex vcur = cam_path.at(s+t-i);
        Vertex vnext = cam_path.at(s+t-1-i);

        Vec3f dirn1 = glm::normalize(vcur.isc.position - vprev.isc.position);
        Vec3f dirn2 = glm::normalize(vcur.isc.position - vnext.isc.position);
        Float dist1 = glm::length(vcur.isc.position - vprev.isc.position);
        Float dist2 = glm::length(vcur.isc.position - vnext.isc.position);
        // TODO: this is currently hardcoded
        Float p1 = s == 0 ? cosineHemispherePDF(worldToLocal(dirn1, vprev.isc.normal), worldToLocal(dirn1, vprev.isc.normal))
                          : vprev.isc.shape->bsdf->pdf(vprev.isc, worldToLocal(dirn1, vprev.isc.normal));
        Float p2 = vnext.isc.shape->bsdf->pdf(vnext.isc, worldToLocal(dirn2, vnext.isc.normal));
        Float cos1 = absDot(vcur.isc.normal, dirn1);
        Float cos2 = absDot(vcur.isc.normal, dirn2);

        Float rel_prob = (p1 * cos1) / (p2 * cos2) * Sqr(dist2 / dist1);
        rel_prob *= i == s+1 ? 1 : rel_probs.at(rel_probs.size()-1);
        rel_probs.push_back(rel_prob);
    }
    for (int i = s-1; i >= 1; i--) {
        Vertex vprev = light_path.at(i-1);
        Vertex vcur = light_path.at(i);
        Vertex vnext = i == s-1 ? cam_path.at(s+t-2-i) : light_path.at(i+1);

        Vec3f dirn1 = glm::normalize(vcur.isc.position - vprev.isc.position);
        Vec3f dirn2 = glm::normalize(vcur.isc.position - vnext.isc.position);
        Float dist1 = glm::length(vcur.isc.position - vprev.isc.position);
        Float dist2 = glm::length(vcur.isc.position - vnext.isc.position);

        // TODO: this is currently hardcoded
        Float p1 = i == 1 ? cosineHemispherePDF(worldToLocal(dirn1, vprev.isc.normal), worldToLocal(dirn1, vprev.isc.normal))
                          : vprev.isc.shape->bsdf->pdf(vprev.isc, dirn1);
        Float p2 = vnext.isc.shape->bsdf->pdf(vnext.isc, dirn2);
        Float cos1 = absDot(vcur.isc.normal, dirn1);
        Float cos2 = absDot(vcur.isc.normal, dirn2);

        Float rel_prob = (p2 * cos2) / (p1 * cos1) * Sqr(dist1 / dist2);
        rel_prob *= i == s-1 ? 1 : rel_probs.at(rel_probs.size()-1);
        rel_probs.push_back(rel_prob);
    }

    Float denom = 1.0;
    for (Float rel_prob : rel_probs)
        denom += Sqr(denom);
    
    return 1.0 / denom;
}
