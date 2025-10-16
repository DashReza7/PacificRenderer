#include <iostream>

#include "core/Integrator.h"
#include "core/Pacific.h"
#include "core/Registry.h"
#include "core/Sampler.h"
#include "core/Scene.h"
#include "utils/ArgParser.h"
#include "utils/SceneParser.h"
#include "utils/Logger.h"


bool g_DEBUG = false;
Logger g_logger = createLogger();
std::filesystem::path scene_file_path;

int run(int argc, char** argv) {
    auto props = ArgParser::parseArgs(argc, argv);

    // parse scene description file
    SceneParser scene_parser;
    auto scene_desc = scene_parser.parseFile(props["input_file"]);
    scene_file_path = std::filesystem::absolute(props["input_file"]);
    // load Scene object from scene description
    Scene scene;
    scene.load_scene(scene_desc);

    Integrator* integrator = IntegratorRegistry::createIntegrator(scene_desc.integrator->type, scene_desc.integrator->properties);

    integrator->render(&scene, scene.sensor, std::stoi(props["n_threads"]), props["show_progress"] == "true");

    scene.sensor->film.output_image(props["output_file"], false);

    return 0;
}

int main(int argc, char** argv) {
    std::cout << "Starting..." << std::endl;

#ifdef NDEBUG
    // Release mode
    g_DEBUG = false;
#else
    // Debug mode
    g_DEBUG = true;
#endif

    int result = 0;
    try {
        result = run(argc, argv);
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        exit(EXIT_FAILURE);
    } catch (...) {
        std::cerr << "Unknown exception occurred\n";
        exit(EXIT_FAILURE);
    }

    std::cout << "\nThe end" << std::endl;
    return result;
}
