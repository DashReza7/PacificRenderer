#include <tiny_obj_loader.h>

#include <CLI/CLI.hpp>
#include <iostream>
#include <memory>

#include "core/BSDF.h"
#include "core/Geometry.h"
#include "core/MathUtils.h"
#include "core/Pacific.h"
#include "core/Scene.h"
#include "core/Sensor.h"
#include "core/Shape.h"
#include "utils/Misc.h"
#include "utils/SceneParser.h"
#include "core/Sampler.h"
#include "core/Integrator.h"
#include "core/Registry.h"
// #include "utils/Logger.h"


int main(int argc, char** argv) {
    // create the global Logger
    // Logger g_logger{LogLevel::INFO, true, ""};
    
    std::string input_file;
    std::string output_file = "output.png";
    bool zip = false;
    bool show_progress = false;
    int n_threads = 1;

    // parse arguments
    CLI::App cli_app;
    // add options
    cli_app.add_option("input", input_file, "Input file path (*.xml)")->required()->check(CLI::ExistingFile);
    cli_app.add_option("-o, --output", output_file, "Output file (*.jpg, *.jpeg, *.png, *.ppm)")
        ->check(CLI::Validator(
            [](const std::string& str) -> std::string {
                std::string lower = str;
                std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);
                if (lower.ends_with(".jpg") || lower.ends_with(".jpeg") || lower.ends_with(".png") || lower.ends_with(".ppm")) {
                    return "";
                }
                return "File must have .jpg, .jpeg, .png, or .ppm extension";
            },
            "IMAGE_EXT"));
    cli_app.add_flag("-z, --zip", zip, "Zip the output file");
    cli_app.add_flag("-p, --progress", show_progress, "Show render progress");
    cli_app.add_option("-t, --threads", n_threads, "Number of running threads (0 for auto detect)")->check(CLI::Range(0, 64));

    // parse the arguments
    CLI11_PARSE(cli_app, argc, argv);

    // parse scene description file
    SceneParser scene_parser;
    SceneDesc scene_desc = scene_parser.parseFile(input_file);
    // load Scene object from scene description
    Scene scene;
    scene.scene_file_path = std::filesystem::absolute(input_file);
    scene.load_scene(scene_desc);
    
    Integrator* integrator = IntegratorRegistry::createIntegrator(scene_desc.integrator->type, scene_desc.integrator->properties);

    // std::cout << integrator->to_string() << "\n" << std::endl;
    // std::cout << scene.to_string() << std::endl;

    integrator->render(&scene, scene.sensor, n_threads);
    scene.sensor->film.output_image(output_file);
    

    std::cout << "\nThe end" << std::endl;
}
