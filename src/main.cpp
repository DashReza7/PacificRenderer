#pragma once

#include <CLI/CLI.hpp>
#include <iostream>
#include <memory>

// Add tinyobjloader include
#include <tiny_obj_loader.h>

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

void test_tinyobjloader() {
    std::cout << "Testing tinyobjloader..." << std::endl;

    // Create a simple OBJ content in memory (a triangle)
    std::string obj_content =
        "v 0.0 0.0 0.0\n"
        "v 1.0 0.0 0.0\n"
        "v 0.5 1.0 0.0\n"
        "f 1 2 3\n";

    std::istringstream obj_stream(obj_content);

    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    std::string warn, err;

    bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err,
                                &obj_stream);

    if (!warn.empty()) {
        std::cout << "WARN: " << warn << std::endl;
    }

    if (!err.empty()) {
        std::cerr << "ERR: " << err << std::endl;
    }

    if (!ret) {
        std::cerr << "Failed to load OBJ" << std::endl;
        return;
    }

    std::cout << "Successfully loaded OBJ!" << std::endl;
    std::cout << "Number of vertices: " << attrib.vertices.size() / 3
              << std::endl;
    std::cout << "Number of shapes: " << shapes.size() << std::endl;

    if (!shapes.empty()) {
        std::cout << "First shape name: " << shapes[0].name << std::endl;
        std::cout << "Number of faces in first shape: "
                  << shapes[0].mesh.num_face_vertices.size() << std::endl;
    }

    // Print vertices
    std::cout << "Vertices:" << std::endl;
    for (size_t i = 0; i < attrib.vertices.size(); i += 3) {
        std::cout << "  v[" << i / 3 << "] = (" << attrib.vertices[i] << ", "
                  << attrib.vertices[i + 1] << ", " << attrib.vertices[i + 2]
                  << ")" << std::endl;
    }
}

void run(int argc, char** argv) {
    // parse arguments
    std::cout << "Hello friend!" << std::endl;

    // Test tinyobjloader
    test_tinyobjloader();
}

int main(int argc, char** argv) {
    try {
        run(argc, argv);
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
    }

    std::cout << "The end" << std::endl;
}