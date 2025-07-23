#include "core/Scene.h"

#include <assimp/postprocess.h>
#include <assimp/scene.h>

#include <assimp/Importer.hpp>

#include "utils/SceneParser.h"

void Scene::load_shapes(const std::vector<std::unique_ptr<ShapeDesc>>& shapes_desc) {
    for (const auto& shape_desc : shapes_desc) {
    }
}

void Scene::load_scene(const SceneDesc& scene_desc) {
    Scene::load_shapes(scene_desc.shapes);
}
