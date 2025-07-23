#pragma once

#include <CLI/CLI.hpp>
#include <iostream>
#include <memory>

#include "core/BSDF.h"
#include "core/Geometry.h"
#include "core/MathUtils.h"
#include "core/Pacific.h"
#include "core/Primitives.h"
#include "core/Scene.h"
#include "core/Sensor.h"
#include "core/Shape.h"
#include "utils/Misc.h"
#include "utils/SceneParser.h"

void run(int argc, char** argv) {
    // parse arguments
    std::cout << "Hello friend!" << std::endl;
}

int main(int argc, char** argv) {
    try {
        run(argc, argv);
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
    }

    std::cout << "The end" << std::endl;
}
