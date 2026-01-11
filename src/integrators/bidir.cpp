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

    void connectPaths(const std::vector<Vertex> &cam_path, const std::vector<Vertex> &light_path,
                      const Scene *scene, int s, int t, Float &mis_weight, Vec2f &pfilm_new) const;

public:
    BidirIntegrator(int max_depth, bool hide_emitters) : max_depth(max_depth), hide_emitters(hide_emitters) {}

    Vec3f sample_radiance(const Scene *scene, Sampler *sampler, const Ray &ray, int row, int col) const override {
        // --------------------- Sample a CAMERA path ---------------------
        int cam_max_length = 5;
        std::vector<Vertex> cam_path{};
        createCamPath(cam_path, scene, sampler, ray, cam_max_length);
        int n_cam_vertices = cam_path.size();

        // --------------------- Sample a LIGHT  path ---------------------
        int light_max_length = 5;
        std::vector<Vertex> light_path{};
        createLightPath(light_path, scene, sampler, light_max_length);
        int n_light_vertices = light_path.size();

        // ------------------------ Connect  paths ------------------------
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

        // DEBUG
        if (bsdf_sample.pdf == 0) {
            std::cout << "\nbsdf_sample.pdf=" << bsdf_sample.pdf << "\n";
            exit(EXIT_FAILURE);
        }
        
        if (bsdf_value == Vec3f{0})
            return;

        Vec3f bsdf_wo = localToWorld(bsdf_sample.wo, isc.normal);
        ray = Ray{isc.position + sign(glm::dot(isc.normal, bsdf_wo))*Epsilon*isc.normal, bsdf_wo, ray.tmin, ray.tmax};
        is_hit = scene->ray_intersect(ray, isc);
        if (!is_hit || bsdf_sample.pdf == 0)
            return;

        // update the previous vertex params
        path[path.size()-1].wo = bsdf_sample.wo;
        Vertex v_prev = path.at(path.size()-1);

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
    v.wi = sensor_ray.d;
    v.wo = Vec3f{0};  // Updated at the next vertex
    Vec3f We = scene->sensor->We(sensor_ray.d);
    Float pdf_We, unused;
    scene->sensor->pdf_We(sensor_ray.d, unused, pdf_We);
    // This happens for some border pixels. Ignore for now
    if (We == Vec3f{0} || pdf_We == 0)
        return;
    // TODO: I'm not sure if the next two lines are correct
    v.cum_beta = Vec3f{We / pdf_We};
    v.cum_pdf = pdf_We;
    cam_path.push_back(v);

    randomWalk(cam_path, scene, ray, isc, sampler, max_vertices-2, PathType::CAMERA_PATH);
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

    // shoot a ray to find an isc
    Ray ray{lightVertex.isc.position + Epsilon*lightVertex.isc.normal, lightVertex.isc.dirn, 1e-3, 1e6};
    Intersection isc;
    bool is_hit = scene->ray_intersect(ray, isc);
    if (!is_hit)
        return;

    Vertex v{VertexType::SURFACE_VERTEX, PathType::LIGHT_PATH};
    v.isc = isc;
    v.wi = lightVertex.wo;
    v.wo = Vec3f{0};  // Updated at the next vertex
    v.cum_beta = light_Le * absDot(lightVertex.isc.normal, lightVertex.isc.dirn) / (pdf_light_posn * pdf_light_dirn);
    v.cum_pdf = pdf_light_posn * pdf_light_dirn * absDot(isc.normal, isc.dirn) / Sqr(isc.distance);
    light_path.push_back(v);

    randomWalk(light_path, scene, ray, isc, sampler, max_vertices-2, PathType::LIGHT_PATH);
}

void BidirIntegrator::connectPaths(const std::vector<Vertex> &cam_path, const std::vector<Vertex> &light_path,
                                   const Scene *scene, int s, int t, Float &mis_weight, Vec2f &pfilm_new) const {

    
}
