#include <tiny_obj_loader.h>

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

int main(int argc, char** argv) {
    std::string input_file;
    std::string output_file = "output.png";
    bool zip = false;
    bool show_progress = false;
    int threads = 1;

    // parse arguments
    CLI::App cli_app;
    // add options
    cli_app.add_option("input", input_file, "Input file path (*.xml)")->required()->check(CLI::ExistingFile);
    cli_app.add_option("-o, --output", output_file, "Output file (*.jpg, *.jpeg, *.png)")
        ->check(CLI::Validator(
            [](const std::string& str) -> std::string {
                std::string lower = str;
                std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);
                if (lower.ends_with(".jpg") || lower.ends_with(".jpeg") || lower.ends_with(".png")) {
                    return "";
                }
                return "File must have .jpg, .jpeg, or .png extension";
            },
            "IMAGE_EXT"));
    cli_app.add_flag("-z, --zip", zip, "Zip the output file");
    cli_app.add_flag("-p, --progress", show_progress, "Show render progress");
    cli_app.add_option("-t, --threads", threads, "Number of running threads (0 for auto)")->check(CLI::Range(0, 64));

    // parse the arguments
    CLI11_PARSE(cli_app, argc, argv);

    SceneParser scene_parser;
    SceneDesc scene_desc = scene_parser.parseFile(input_file);

    Scene scene;
    scene.load_scene(scene_desc);

    std::cout << "The end" << std::endl;
}
