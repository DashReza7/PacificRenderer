#include "core/Scene.h"
#include "utils/SceneParser.h"

void Scene::load_shapes(const std::vector<std::unique_ptr<ShapeDesc>>& shapes_desc) {
    for (const auto& shape_desc : shapes_desc) {
        
    }
}

void Scene::load_scene(const SceneDesc& scene_desc) {
    this->load_shapes(scene_desc.shapes);
}
