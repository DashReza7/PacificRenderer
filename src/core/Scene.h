#pragma once

#include <memory>
#include <vector>

#include "core/Integrator.h"
#include "core/Sensor.h"
#include "core/Shape.h"
#include "utils/SceneParser.h"

class Scene {
   private:
    std::vector<std::unique_ptr<Shape>> shapes;
    std::unique_ptr<Sensor> sensor;
    std::unique_ptr<Integrator> integrator;

    void load_shapes(const std::vector<std::unique_ptr<ShapeDesc>>& shapes_desc);

   public:
    Scene() = default;
    ~Scene() = default;

    // TODO: right now just load shapes
    void load_scene(const SceneDesc& scene_desc);
};
