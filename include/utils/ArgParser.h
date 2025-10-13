#pragma once
#include <CLI/CLI.hpp>
#include <string>
#include <unordered_map>

class ArgParser {
public:
    static std::unordered_map<std::string, std::string> parseArgs(int argc, char** argv) {
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
        try {
            cli_app.parse(argc, argv);
        } catch(const CLI::ParseError &e) {
            std::cerr << e.what() << '\n';
        }

        std::unordered_map<std::string, std::string> props{};
        props["input_file"] = input_file;
        props["output_file"] = output_file;
        props["zip"] = zip ? "true" : "false";
        props["show_progress"] = show_progress ? "true" : "false";
        props["n_threads"] = std::to_string(n_threads);

        return props;
    }
};
