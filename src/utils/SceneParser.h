#pragma once

#include <algorithm>  // Add this include at the top
#include <filesystem>
#include <iostream>
#include <memory>
#include <pugixml.hpp>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>
#include <regex>
#include <unordered_set>

#include "core/MathUtils.h"
#include "utils/Misc.h"

// Utils
inline Vec3f strToVec3f(const std::string& value_str) {
    Vec3f vec;
    std::istringstream iss(value_str);
    char comma;
    iss >> vec.x >> comma >> vec.y >> comma >> vec.z;
    return vec;
}

inline Mat4f strToMat4f(std::string mat_str) {
    Mat4f mat;
    std::istringstream iss(mat_str);

    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            iss >> mat[i][j];

    return mat;
}

inline std::string mat4fToStr(Mat4f mat) {
    std::ostringstream oss;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            oss << mat[i][j];
            if (i < 3 || j < 3) oss << " ";
        }
    }
    return oss.str();
}

// Base class for all scene objects
struct SceneObjectDesc {
    std::string type;
    std::unordered_map<std::string, std::string> properties;

    virtual ~SceneObjectDesc() = default;

    virtual std::string to_string() {
        std::ostringstream oss;
        oss << "(SceneObjectDesc)\n";
        oss << "type: " << type << "\n";
        if (properties.size() > 0) {
            oss << "properties: {\n";
            for (const auto [name, value] : properties)
                oss << "    name: " << name << ", value: " << value << "\n";
            oss << "}";
        }
        return oss.str();
    }
};

// Specific scene object types
struct IntegratorDesc : public SceneObjectDesc {
    std::string to_string() override {
        std::ostringstream oss;
        oss << "(IntegratorDesc)\n";
        oss << "type: " << type << "\n";
        if (properties.size() > 0) {
            oss << "properties: {\n";
            for (const auto [name, value] : properties)
                oss << "    name: " << name << ", value: " << value << "\n";
            oss << "}";
        }
        return oss.str();
    }
};

struct BSDFDesc : public SceneObjectDesc {
    std::string id;

    std::string to_string() override {
        std::ostringstream oss;
        oss << "(BSDFDesc)\n";
        oss << "type: " << type << "\n";
        if (properties.size() > 0) {
            oss << "properties: {\n";
            for (const auto [name, value] : properties)
                oss << "    name: " << name << ", value: " << value << "\n";
            oss << "}\n";
        }
        if (id.length() != 0) oss << "id: " << id;
        return oss.str();
    }
};
struct EmitterDesc : public SceneObjectDesc {
    std::string to_string() override {
        std::ostringstream oss;
        oss << "(EmitterDesc)\n";
        oss << "type: " << type << "\n";
        if (properties.size() > 0) {
            oss << "properties: {\n";
            for (const auto [name, value] : properties)
                oss << "    name: " << name << ", value: " << value << "\n";
            oss << "}";
        }
        return oss.str();
    }
};
struct SamplerDesc : public SceneObjectDesc {
    std::string to_string() override {
        std::ostringstream oss;
        oss << "(SamplerDesc)\n";
        oss << "type: " << type << "\n";
        if (properties.size() > 0) {
            oss << "properties: {\n";
            for (const auto [name, value] : properties)
                oss << "    name: " << name << ", value: " << value << "\n";
            oss << "}";
        }
        return oss.str();
    }
};
struct FilmDesc : public SceneObjectDesc {
    std::string to_string() override {
        std::ostringstream oss;
        oss << "(FilmDesc)\n";
        oss << "type: " << type << "\n";
        if (properties.size() > 0) {
            oss << "properties: {\n";
            for (const auto [name, value] : properties)
                oss << "    name: " << name << ", value: " << value << "\n";
            oss << "}";
        }
        return oss.str();
    }
};
struct SensorDesc : public SceneObjectDesc {
    FilmDesc* film;
    SamplerDesc* sampler;
    Mat4f to_world = glm::mat<4, 4, Float>(1);

    std::string to_string() override {
        std::ostringstream oss;
        oss << "(SensorDesc)\n";
        oss << "type: " << type << "\n";
        if (properties.size() > 0) {
            oss << "properties: {\n";
            for (const auto [name, value] : properties)
                oss << "    name: " << name << ", value: " << value << "\n";
            oss << "}\n";
        }
        if (film) oss << "film: " << film->to_string() << "\n";
        if (sampler) oss << "sampler: " << sampler->to_string() << "\n";
        return oss.str();
    }

    ~SensorDesc() {
        delete film;
        delete sampler;
    }
};
struct ShapeDesc : public SceneObjectDesc {
    BSDFDesc* bsdf;
    EmitterDesc* emitter;
    Mat4f to_world = glm::mat<4, 4, Float>(1);

    std::string to_string() override {
        std::ostringstream oss;
        oss << "(ShapeDesc)\n";
        oss << "type: " << type << "\n";
        if (properties.size() > 0) {
            oss << "properties: {\n";
            for (const auto [name, value] : properties)
                oss << "    name: " << name << ", value: " << value << "\n";
            oss << "}\n";
        }
        if (bsdf) oss << bsdf->to_string();
        if (emitter) oss << emitter->to_string();
        return oss.str();
    }
};
struct SceneDesc {
    IntegratorDesc* integrator;
    SensorDesc* sensor;
    std::vector<ShapeDesc*> shapes;
    std::vector<BSDFDesc*> bsdfs;
    std::vector<EmitterDesc*> emitters;

    std::string to_string() {
        std::ostringstream oss;
        oss << integrator->to_string() << "\n\n";
        oss << sensor->to_string() << "\n\n";
        oss << "Shapes(" << shapes.size() << "):\n";
        for (const auto& shape : shapes)
            oss << shape->to_string() << "\n\n";
        oss << "\nGlobal BSDFs(" << bsdfs.size() << "):\n";
        for (const auto& bsdf : bsdfs)
            oss << bsdf->to_string() << "\n\n";
        // TODO: Do we even have Global Emitters?
        oss << "\nGlobal Emitters(" << emitters.size() << "):\n";
        for (const auto& emitter : emitters)
            oss << emitter->to_string() << "\n\n";

        return oss.str();
    }

    ~SceneDesc() {
        delete integrator;
        delete sensor;
        for (auto shape : shapes)
            delete shape;
        for (auto bsdf : bsdfs)
            delete bsdf;
        for (auto emitter : emitters)
            delete emitter;
    }
};

class SceneParser {
public:
    SceneDesc parseFile(const std::string& filename) {
        pugi::xml_document doc;
        pugi::xml_parse_result result = doc.load_file(filename.c_str());

        if (!result) {
            throw std::runtime_error("Failed to load XML file: " +
                                     std::string(result.description()));
        }

        return parseScene(doc);
    }

    SceneDesc parseString(const std::string& xml_content) {
        pugi::xml_document doc;
        pugi::xml_parse_result result = doc.load_string(xml_content.c_str());

        if (!result) {
            throw std::runtime_error("Failed to parse XML string: " +
                                     std::string(result.description()));
        }

        return parseScene(doc);
    }

private:
    std::unordered_map<std::string, std::string> defaults;
    std::unordered_map<std::string, BSDFDesc*> shared_bsdfs;

    SceneDesc parseScene(const pugi::xml_document& doc) {
        SceneDesc scene;

        pugi::xml_node scene_node = doc.child("scene");
        if (!scene_node) {
            throw std::runtime_error("No <scene> root element found");
        }

        // Parse all child elements
        for (pugi::xml_node child : scene_node.children()) {
            std::string node_name = child.name();

            if (node_name == "integrator")
                scene.integrator = parseIntegrator(child);
            else if (node_name == "sensor")
                scene.sensor = parseSensor(child);
            else if (node_name == "shape") {
                ShapeDesc* shape_desc = parseShape(child);
                scene.shapes.push_back(shape_desc);
                if (shape_desc->emitter != nullptr)
                    scene.emitters.push_back(shape_desc->emitter);
                if (shape_desc->bsdf != nullptr && std::find(scene.bsdfs.begin(), scene.bsdfs.end(), shape_desc->bsdf) == scene.bsdfs.end())
                    scene.bsdfs.push_back(shape_desc->bsdf);
            } else if (node_name == "bsdf")
                scene.bsdfs.push_back(parseBSDF(child));
            else if (node_name == "emitter")
                scene.emitters.push_back(parseEmitter(child));
            else if (node_name == "default")
                this->add_default(child);
            else
                throw std::runtime_error(std::string("Unknown scene object: ") +
                                         node_name);
        }

        return scene;
    }

    IntegratorDesc* parseIntegrator(const pugi::xml_node& node) {
        auto integrator = new IntegratorDesc{};
        integrator->type = get_default(node.attribute("type").value());

        parseProperties(node, integrator->properties);
        return integrator;
    }

    SensorDesc* parseSensor(const pugi::xml_node& node) {
        auto sensor = new SensorDesc{};
        sensor->type = get_default(node.attribute("type").value());

        parseProperties(node, sensor->properties, {"sampler", "film"});

        // Parse nested objects
        for (pugi::xml_node child : node.children()) {
            std::string child_name = child.name();

            if (child_name == "film") {
                sensor->film = parseFilm(child);
            } else if (child_name == "sampler") {
                sensor->sampler = parseSampler(child);
            } else if (child_name == "transform") {
                sensor->to_world = parseTransform(child);
            }
        }

        return sensor;
    }

    ShapeDesc* parseShape(const pugi::xml_node& node) {
        auto shape = new ShapeDesc{};
        shape->type = get_default(node.attribute("type").value());

        parseProperties(node, shape->properties, {"bsdf", "emitter", "ref", "transform"});

        // Parse nested objects
        for (pugi::xml_node child : node.children()) {
            std::string child_name = child.name();

            if (child_name == "bsdf") {
                shape->bsdf = parseBSDF(child);
            } else if (child_name == "ref") {
                // Handle references to other objects
                // Store reference for later resolution
                std::string id = child.attribute("id").value();
                shape->bsdf = shared_bsdfs[id];
            } else if (child_name == "emitter") {
                shape->emitter = parseEmitter(child);
            } else if (child_name == "transform") {
                shape->to_world = parseTransform(child);
            }
        }

        return shape;
    }

    BSDFDesc* parseBSDF(const pugi::xml_node& node) {
        auto bsdf = new BSDFDesc{};
        bsdf->type = get_default(node.attribute("type").value());
        if (node.attribute("id")) {
            bsdf->id = node.attribute("id").value();
            shared_bsdfs[bsdf->id] = bsdf;
        }

        parseProperties(node, bsdf->properties, {"id"});
        return bsdf;
    }

    EmitterDesc* parseEmitter(const pugi::xml_node& node) {
        auto emitter = new EmitterDesc{};
        emitter->type = get_default(node.attribute("type").value());

        parseProperties(node, emitter->properties);
        return emitter;
    }

    FilmDesc* parseFilm(const pugi::xml_node& node) {
        auto film = new FilmDesc{};
        film->type = get_default(node.attribute("type").value());

        parseProperties(node, film->properties);
        return film;
    }

    SamplerDesc* parseSampler(const pugi::xml_node& node) {
        auto sampler = new SamplerDesc{};
        sampler->type = get_default(node.attribute("type").value());

        parseProperties(node, sampler->properties);
        return sampler;
    }

    SceneObjectDesc* parseGenericObject(const pugi::xml_node& node) {
        auto obj = new SceneObjectDesc{};
        obj->type = get_default(node.attribute("type").value());

        parseProperties(node, obj->properties);
        return obj;
    }

    void parseProperties(
        const pugi::xml_node& node,
        std::unordered_map<std::string, std::string>& properties,
        std::unordered_set<std::string> exclude_keys = {}) {
        for (pugi::xml_node child : node.children()) {
            std::string child_name = child.name();
            if (exclude_keys.find(child_name) != exclude_keys.end())
                continue;
            std::string name = child.attribute("name").value();

            if (child_name == "string") {
                properties[name] = get_default(child.attribute("value").value());
            } else if (child_name == "integer") {
                properties[name] = get_default(child.attribute("value").value());
            } else if (child_name == "float") {
                properties[name] = get_default(child.attribute("value").value());
            } else if (child_name == "boolean") {
                properties[name] = get_default(child.attribute("value").value());
            } else if (child_name == "point" || child_name == "vector") {
                if (child.attribute("x")) {
                    std::string x = get_default(child.attribute("x").value());
                    std::string y = get_default(child.attribute("y").value());
                    std::string z = get_default(child.attribute("z").value());
                    properties[name] = x + ", " + y + ", " + z;
                } else if (child.attribute("value")) {
                    std::string value_str = get_default(child.attribute("value").value());
                    value_str = trim(value_str);
                    if (value_str.find(',') == std::string::npos) {
                        if (value_str.find(' ') == std::string::npos)
                            value_str = value_str + ", " + value_str + ", " + value_str;
                        else
                            value_str = std::regex_replace(value_str, std::regex(" "), ", ");
                    }
                    properties[name] = value_str;
                }
            } else if (child_name == "rgb") {
                if (child.attribute("r")) {
                    std::string r = get_default(child.attribute("r").value());
                    std::string g = get_default(child.attribute("g").value());
                    std::string b = get_default(child.attribute("b").value());
                    properties[name] = r + ", " + g + ", " + b;
                } else if (child.attribute("value")) {
                    std::string value_str = get_default(child.attribute("value").value());
                    value_str = trim(value_str);
                    if (value_str.find(',') == std::string::npos) {
                        if (value_str.find(' ') == std::string::npos)
                            value_str = value_str + ", " + value_str + ", " + value_str;
                        else
                            value_str = std::regex_replace(value_str, std::regex(" "), ", ");
                    }
                    properties[name] = value_str;
                }
            } else if (child_name == "transform") {
                properties[name] = mat4fToStr(parseTransform(child));
            } else {
                throw std::runtime_error(std::string("Unknown property type: ") + child_name);
            }
        }
    }

    Mat4f parseTransform(const pugi::xml_node& node) {
        Mat4f trafo = glm::mat<4, 4, Float>(1);

        for (pugi::xml_node child : node.children()) {
            std::string child_name = child.name();

            if (child_name == "translate") {
                // TODO: change as_float to as_double adaptively
                Float x = child.attribute("x") ? child.attribute("x").as_float() : 0.0f;
                Float y = child.attribute("y") ? child.attribute("y").as_float() : 0.0f;
                Float z = child.attribute("z") ? child.attribute("z").as_float() : 0.0f;

                if (child.attribute("value")) {
                    std::string value_str = child.attribute("value").value();
                    std::istringstream iss(value_str);
                    char comma;
                    iss >> x >> comma >> y >> comma >> z;
                }

                trafo = glm::translate(trafo, Vec3f{x, y, z});
            } else if (child_name == "rotate") {
                // Handle rotation - simplified version
                Float angle = child.attribute("angle")
                                  ? child.attribute("angle").as_float()
                                  : 0.0f;
                Float x = child.attribute("x") ? child.attribute("x").as_float()
                                               : 0.0f;
                Float y = child.attribute("y") ? child.attribute("y").as_float()
                                               : 0.0f;
                Float z = child.attribute("z") ? child.attribute("z").as_float()
                                               : 0.0f;

                if (child.attribute("value")) {
                    std::string value_str = child.attribute("value").value();
                    std::istringstream iss(value_str);
                    char comma;
                    iss >> x >> comma >> y >> comma >> z;
                }

                trafo = glm::rotate(trafo, glm::radians(angle), Vec3f{x, y, z});
            } else if (child_name == "scale") {
                Float x = child.attribute("x") ? child.attribute("x").as_float()
                                               : 1.0f;
                Float y = child.attribute("y") ? child.attribute("y").as_float()
                                               : 1.0f;
                Float z = child.attribute("z") ? child.attribute("z").as_float()
                                               : 1.0f;

                if (child.attribute("value")) {
                    std::string value_str = child.attribute("value").value();
                    std::istringstream iss(value_str);
                    char comma;
                    iss >> x >> comma >> y >> comma >> z;
                }

                trafo = glm::scale(trafo, Vec3f{x, y, z});
            } else if (child_name == "matrix") {
                Mat4f matrix;
                std::string value = child.attribute("value").value();
                std::istringstream iss(value);

                for (int i = 0; i < 16; ++i) iss >> matrix[i % 4][i / 4];

                trafo = matrix * trafo;
            } else if (child_name == "lookat") {
                std::string origin_str = child.attribute("origin").value();
                std::string target_str = child.attribute("target").value();
                std::string up_str = child.attribute("up").value();
                Vec3f origin, target, up;

                char comma;
                std::istringstream iss(origin_str);
                iss >> origin.x >> comma >> origin.y >> comma >> origin.z;
                iss = std::istringstream(target_str);
                iss >> target.x >> comma >> target.y >> comma >> target.z;
                iss = std::istringstream(up_str);
                iss >> up.x >> comma >> up.y >> comma >> up.z;

                Mat4f lookat_mat = glm::lookAt(origin, target, up);
                trafo = lookat_mat * trafo;
            }
        }

        return trafo;
    }

    void add_default(const pugi::xml_node& node) {
        defaults[node.attribute("name").value()] =
            node.attribute("value").value();
    }

    std::string get_default(const std::string& value) {
        if (value[0] == '$')
            return defaults[value.substr(1, value.length() - 1)];
        return value;
    }
};
