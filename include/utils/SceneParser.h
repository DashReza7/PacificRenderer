#pragma once

#include <algorithm>  // Add this include at the top
#include <filesystem>
#include <iostream>
#include <memory>
#include <pugixml.hpp>
#include <regex>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "core/MathUtils.h"
#include "utils/Misc.h"

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

struct TextureDesc : public SceneObjectDesc {
    std::string id;

    std::string to_string() override {
        std::ostringstream oss;
        oss << "(TextureDesc)\n";
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

struct BSDFDesc : public SceneObjectDesc {
    std::string id;
    std::unordered_map<std::string, const TextureDesc*> textures;

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
    std::unordered_map<std::string, const TextureDesc*> textures;

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

struct RFilterDesc : public SceneObjectDesc {
    std::string to_string() override {
        std::ostringstream oss;
        oss << "(RFilterDesc)\n";
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
    RFilterDesc* rfilter;

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
    Mat4f to_world = Mat4f{1.0};

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
    BSDFDesc* bsdf = nullptr;
    EmitterDesc* emitter = nullptr;

    ShapeDesc() {
        properties["to_world"] = mat4fToStr(Mat4f{1.0});
        properties["inv_to_world"] = mat4fToStr(Mat4f{1.0});
    }

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
    std::vector<const TextureDesc*> textures;
    std::vector<EmitterDesc*> emitters;
    bool has_envmap = false;

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
    std::unordered_map<std::string, TextureDesc*> shared_textures;
    bool has_envmap = false;

    /// @brief utility function for parsing a node containing a vector type(point, rgb, vector, etc.) `value` or `x`, `r`, etc.
    /// @param special_char the special character of that vector type. e.g. `point` and `vector` have "x", `rgb` has "r"
    std::string parseVectorType(const pugi::xml_node& node) {
        if (!node.attribute("value")) {
            std::string a = "0", b = "0", c = "0";
            if (node.attribute("x"))
                a = node.attribute("x").value();
            else if (node.attribute("r"))
                a = node.attribute("r").value();
            if (node.attribute("y"))
                b = node.attribute("y").value();
            else if (node.attribute("g"))
                b = node.attribute("g").value();
            if (node.attribute("z"))
                c = node.attribute("z").value();
            else if (node.attribute("b"))
                c = node.attribute("b").value();

            a = get_default(a);
            b = get_default(b);
            c = get_default(c);
            return a + ", " + b + ", " + c;
        } else {
            std::string value_str = get_default(node.attribute("value").value());
            value_str = trim(value_str);
            if (value_str.find(',') == std::string::npos) {
                if (value_str.find(' ') == std::string::npos)
                    value_str = value_str + ", " + value_str + ", " + value_str;
                else
                    value_str = std::regex_replace(value_str, std::regex(" "), ", ");
            }
            return value_str;
        }
    }

    SceneDesc parseScene(const pugi::xml_document& doc) {
        SceneDesc scene;

        pugi::xml_node scene_node = doc.child("scene");
        if (!scene_node) {
            throw std::runtime_error("No <scene> root element found");
        }

        // Parse all child elements
        for (pugi::xml_node child : scene_node.children()) {
            std::string node_name = child.name();

            if (node_name == "integrator") {
                scene.integrator = parseIntegrator(child);
            } else if (node_name == "sensor") {
                scene.sensor = parseSensor(child);
            } else if (node_name == "shape") {
                ShapeDesc* shape_desc = parseShape(child);
                scene.shapes.push_back(shape_desc);
                if (shape_desc->emitter != nullptr)
                    scene.emitters.push_back(shape_desc->emitter);
                if (shape_desc->bsdf != nullptr && std::find(scene.bsdfs.begin(), scene.bsdfs.end(), shape_desc->bsdf) == scene.bsdfs.end())
                    scene.bsdfs.push_back(shape_desc->bsdf);
            } else if (node_name == "bsdf") {
                BSDFDesc* bsdf_desc = parseBSDF(child);
                scene.bsdfs.push_back(bsdf_desc);
                for (const auto& [name, texture] : bsdf_desc->textures)
                    if (std::find(scene.textures.begin(), scene.textures.end(), texture) == scene.textures.end())
                        scene.textures.push_back(texture);
            } else if (node_name == "texture") {
                scene.textures.push_back(parseTexture(child));
            } else if (node_name == "emitter") {
                EmitterDesc* emitter_desc = parseEmitter(child);
                scene.emitters.push_back(emitter_desc);
                for (const auto& [name, texture] : emitter_desc->textures)
                    if (std::find(scene.textures.begin(), scene.textures.end(), texture) == scene.textures.end())
                        scene.textures.push_back(texture);
                if (emitter_desc->type == "envmap")
                    scene.has_envmap = true;
            } else if (node_name == "default") {
                this->add_default(child);
            } else {
                throw std::runtime_error(std::string("Unknown scene object: ") + node_name);
            }
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

        parseProperties(node, sensor->properties, {"sampler", "film", "rfilter"});

        // Parse nested objects
        for (pugi::xml_node child : node.children()) {
            std::string child_name = child.name();

            if (child_name == "film") {
                sensor->film = parseFilm(child);
            } else if (child_name == "sampler") {
                sensor->sampler = parseSampler(child);
            } else if (child_name == "transform") {
                sensor->to_world = parseTransform(child).first;
            }
        }

        return sensor;
    }

    ShapeDesc* parseShape(const pugi::xml_node& node) {
        auto shape = new ShapeDesc{};
        shape->type = get_default(node.attribute("type").value());

        parseProperties(node, shape->properties, {"bsdf", "emitter", "ref"});

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
            }
        }

        return shape;
    }

    TextureDesc* parseTexture(const pugi::xml_node& node) {
        auto texture = new TextureDesc{};
        texture->type = get_default(node.attribute("type").value());

        if (std::any_of(node.children().begin(), node.children().end(), [](const pugi::xml_node& child) { return std::string(child.name()) == "texture"; }))
            throw std::runtime_error("Nested textures are not supported.");
        
        parseProperties(node, texture->properties);

        if (node.attribute("id")) {
            texture->id = node.attribute("id").value();
            if (shared_textures.contains(texture->id))
                throw std::runtime_error("Duplicate texture id: " + texture->id);
            shared_textures[texture->id] = texture;
        }

        return texture;
    }

    BSDFDesc* parseBSDF(const pugi::xml_node& node, bool allow_twosided = true) {
        auto bsdf = new BSDFDesc{};
        bsdf->type = get_default(node.attribute("type").value());

        // TODO: handle resource deallocation on errors
        if (bsdf->type == "twosided") {
            if (std::distance(node.children().begin(), node.children().end()) != 1)
                throw std::runtime_error(  // TODO
                    "A twosided BSDF must have exactly one child BSDF. Two separate BSDFs are not yet supported.");

            if (!allow_twosided)
                throw std::runtime_error("Nested twosided BSDFs are not supported.");

            delete bsdf;
            bsdf = parseBSDF(*node.children().begin(), false);
            // TODO:
            bsdf->properties["twosided"] = "true";
            if (node.attribute("id")) {
                bsdf->id = node.attribute("id").value();
                if (shared_bsdfs.contains(bsdf->id))
                    throw std::runtime_error("Duplicate BSDF id: " + bsdf->id);
                shared_bsdfs[bsdf->id] = bsdf;
            }
        } else {
            parseProperties(node, bsdf->properties, {"id", "texture", "ref"});
            if (node.attribute("id")) {
                bsdf->id = node.attribute("id").value();
                if (shared_bsdfs.contains(bsdf->id))
                    throw std::runtime_error("Duplicate BSDF id: " + bsdf->id);
                shared_bsdfs[bsdf->id] = bsdf;
            }
            // parse textures
            for (pugi::xml_node child : node.children()) {
                std::string child_name = child.name();
                if (child_name == "texture") {
                    if (!child.attribute("name"))
                        throw std::runtime_error("Texture inside BSDF doesn't have a name.");
                    bsdf->textures[child.attribute("name").value()] = parseTexture(child);
                } else if (child_name == "ref") {
                    if (!child.attribute("name"))
                        throw std::runtime_error("Ref inside BSDF doesn't have a name.");
                    if (!shared_textures.contains(child.attribute("id").value()))
                        throw std::runtime_error("Referenced texture does not exist.");
                    bsdf->textures[child.attribute("name").value()] = shared_textures[child.attribute("id").value()];
                }
            }
        }

        return bsdf;
    }

    EmitterDesc* parseEmitter(const pugi::xml_node& node) {
        auto emitter = new EmitterDesc{};
        emitter->type = get_default(node.attribute("type").value());
        if (emitter->type == "envmap") {
            if (has_envmap)
                throw std::runtime_error("Only one environment map is allowed in the scene.");
            has_envmap = true;
        }

        parseProperties(node, emitter->properties, {"texture"});

        // parse textures
        for (pugi::xml_node child : node.children()) {
            std::string child_name = child.name();
            if (child_name == "texture") {
                if (!child.attribute("name"))
                    throw std::runtime_error("Texture inside BSDF doesn't have a name.");
                emitter->textures[child.attribute("name").value()] = parseTexture(child);
            } else if (child_name == "ref") {
                if (!child.attribute("name"))
                    throw std::runtime_error("Ref inside BSDF doesn't have a name.");
                if (!shared_textures.contains(child.attribute("id").value()))
                    throw std::runtime_error("Referenced texture does not exist.");
                emitter->textures[child.attribute("name").value()] = shared_textures[child.attribute("id").value()];
            }
        }

        return emitter;
    }

    FilmDesc* parseFilm(const pugi::xml_node& node) {
        auto film = new FilmDesc{};
        film->type = get_default(node.attribute("type").value());
        film->rfilter = new RFilterDesc{};
        film->rfilter->type = "gaussian";
        film->rfilter->properties["stddev"] = "0.5";

        // Parse nested objects
        for (pugi::xml_node child : node.children()) {
            std::string child_name = child.name();
            if (child_name == "rfilter") {
                delete film->rfilter;  // Delete the default rfilter
                film->rfilter = parseRFilter(child);
            }
        }

        // TODO
        parseProperties(node, film->properties, {"pixel_format", "rfilter"});
        return film;
    }

    RFilterDesc* parseRFilter(const pugi::xml_node& node) {
        auto rfilter = new RFilterDesc{};
        rfilter->type = get_default(node.attribute("type").value());

        parseProperties(node, rfilter->properties);
        return rfilter;
    }

    SamplerDesc* parseSampler(const pugi::xml_node& node) {
        auto sampler = new SamplerDesc{};
        sampler->type = get_default(node.attribute("type").value());

        sampler->properties["seed"] = "0";

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
                properties[name] = parseVectorType(child);
            } else if (child_name == "rgb") {
                properties[name] = parseVectorType(child);
            } else if (child_name == "transform") {
                auto [transform, inv_transform] = parseTransform(child);
                properties[name] = mat4fToStr(transform);
                // TODO: what if both name and inv_name are in the properties???
                properties["inv_" + name] = mat4fToStr(inv_transform);
            } else {
                throw std::runtime_error(std::string("Unknown property type: ") + child_name);
            }
        }
    }

    // Parses a <transform> node and returns the corresponding transformation and its inverse
    std::pair<Mat4f, Mat4f> parseTransform(const pugi::xml_node& node) {
        Mat4f trafo = Mat4f{1.0};
        Mat4f inv_trafo = Mat4f{1.0};

        for (const pugi::xml_node& child : node.children()) {
            std::string child_name = child.name();

            if (child_name == "translate") {
                auto translate_value_str = parseVectorType(child);
                trafo = glm::translate(Mat4f{1.0}, strToVec3f(translate_value_str)) * trafo;
                inv_trafo = inv_trafo * glm::translate(Mat4f{1.0}, -strToVec3f(translate_value_str));
            } else if (child_name == "rotate") {
                Float angle = child.attribute("angle") ? child.attribute("angle").as_float() : 0.0;
                auto axis_value_str = parseVectorType(child);
                Mat4f rot_mat = get_rotation_matrix(glm::normalize(strToVec3f(axis_value_str)), angle);
                Mat4f inv_rot_mat = get_rotation_matrix(glm::normalize(strToVec3f(axis_value_str)), -angle);
                trafo = rot_mat * trafo;
                inv_trafo = inv_trafo * inv_rot_mat;
            } else if (child_name == "scale") {
                auto scale_value_str = parseVectorType(child);
                trafo = glm::scale(Mat4f{1.0}, strToVec3f(scale_value_str)) * trafo;
                inv_trafo = inv_trafo * glm::scale(Mat4f{1.0}, Vec3f{1.0} / strToVec3f(scale_value_str));
            } else if (child_name == "matrix") {
                Mat4f matrix;
                std::string value = child.attribute("value").value();
                // replace commas if any
                value = std::regex_replace(value, std::regex(","), " ");
                std::istringstream iss(value);

                // matrix is column-major
                for (int i = 0; i < 16; ++i)
                    iss >> matrix[i % 4][i / 4];

                trafo = matrix * trafo;
                inv_trafo = inv_trafo * glm::inverse(matrix);
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

                // glm::lookAt gives -Z forward, +X right
                // We want +Z forward, +X left, so flip both axes
                Mat4f flip = Mat4f(1.0f);
                flip[0][0] = -1.0f;  // flip X (right -> left)
                flip[2][2] = -1.0f;  // flip Z (-Z forward -> +Z forward)

                Mat4f lookat_mat = glm::inverse(glm::lookAt(origin, target, up)) * flip;

                trafo = lookat_mat * trafo;
                inv_trafo = inv_trafo * glm::inverse(lookat_mat);
            }
        }

        return {trafo, inv_trafo};
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
