#pragma once
#include <string>
#include <vector>
#include <unordered_map>
#include <memory>
#include <iostream>
#include <sstream>
#include <pugixml.hpp>
#include "core/MathUtils.h"

// Base class for all scene objects
struct SceneObject {
	std::string type;
	std::unordered_map<std::string, std::string> properties;

	virtual ~SceneObject() = default;

	virtual std::string to_string() {
		std::ostringstream oss;
		oss << "(SceneObject)\n";
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
struct Integrator : public SceneObject {

	std::string to_string() override {
		std::ostringstream oss;
		oss << "(Integrator)\n";
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
struct Sensor : public SceneObject {
	std::unique_ptr<SceneObject> film;
	std::unique_ptr<SceneObject> sampler;
	Mat4f to_world;

	std::string to_string() override {
		std::ostringstream oss;
		oss << "(Sensor)\n";
		oss << "type: " << type << "\n";
		if (properties.size() > 0) {
			oss << "properties: {\n";
			for (const auto [name, value] : properties)
				oss << "    name: " << name << ", value: " << value << "\n";
			oss << "}\n";
		}
		if (film)
			oss << "film: " << film->to_string() << "\n";
		if (sampler)
			oss << "sampler: " << sampler->to_string() << "\n";
		return oss.str();
	}
};
struct BSDF : public SceneObject {
	std::string id;
		
	std::string to_string() override {
		std::ostringstream oss;
		oss << "(BSDF)\n";
		oss << "type: " << type << "\n";
		if (properties.size() > 0) {
			oss << "properties: {\n";
			for (const auto [name, value] : properties)
				oss << "    name: " << name << ", value: " << value << "\n";
			oss << "}\n";
		}
		if (id.length() != 0)
			oss << "id: " << id;
		return oss.str();
	}
};
struct Emitter : public SceneObject {

	std::string to_string() override {
		std::ostringstream oss;
		oss << "(Emitter)\n";
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
struct Sampler : public SceneObject {

	std::string to_string() override {
		std::ostringstream oss;
		oss << "(Sampler)\n";
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
struct Film : public SceneObject {

	std::string to_string() override {
		std::ostringstream oss;
		oss << "(Film)\n";
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
struct Shape : public SceneObject {
	std::shared_ptr<BSDF> bsdf;
	std::unique_ptr<Emitter> emitter;
	Mat4f to_world = glm::mat<4, 4, Float>(1);

	std::string to_string() override {
		std::ostringstream oss;
		oss << "(Shape)\n";
		oss << "type: " << type << "\n";
		if (properties.size() > 0) {
			oss << "properties: {\n";
			for (const auto [name, value] : properties)
				oss << "    name: " << name << ", value: " << value << "\n";
			oss << "}\n";
		}
		if (bsdf)
			oss << bsdf->to_string();
		if (emitter)
			oss << emitter->to_string();
		return oss.str();
	}
};
struct Scene {
	std::unique_ptr<Integrator> integrator;
	std::unique_ptr<Sensor> sensor;
	std::vector<std::unique_ptr<Shape>> shapes;
	std::vector<std::shared_ptr<BSDF>> bsdfs;
	std::vector<std::unique_ptr<Emitter>> emitters;

	std::string to_string() {
		std::ostringstream oss;
		oss << integrator->to_string() << "\n\n";
		oss << sensor->to_string() << "\n\n";
		oss << "Shapes(" << shapes.size() << "):\n";
		for (const auto& shape : shapes)
			oss << shape->to_string() << "\n";
		oss << "\nGlobal BSDFs(" << bsdfs.size() << "):\n";
		for (const auto& bsdf : bsdfs)
			oss << bsdf->to_string() << "\n";
		// TODO: Do we even have Global Emitters?
		oss << "\nGlobal Emitters(" << emitters.size() << "):\n";
		for (const auto& emitter : emitters)
			oss << emitter->to_string() << "\n";
		
		return oss.str();
	}
};

class PacificParser {
public:
	Scene parseFile(const std::string& filename) {
		pugi::xml_document doc;
		pugi::xml_parse_result result = doc.load_file(filename.c_str());

		if (!result) {
			throw std::runtime_error("Failed to load XML file: " + std::string(result.description()));
		}

		return parseScene(doc);
	}

	Scene parseString(const std::string& xml_content) {
		pugi::xml_document doc;
		pugi::xml_parse_result result = doc.load_string(xml_content.c_str());

		if (!result) {
			throw std::runtime_error("Failed to parse XML string: " + std::string(result.description()));
		}

		return parseScene(doc);
	}

private:
	std::unordered_map<std::string, std::string> defaults;
	std::unordered_map<std::string, std::shared_ptr<BSDF>> shared_bsdfs;


	Scene parseScene(const pugi::xml_document& doc) {
		Scene scene;

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
			else if (node_name == "shape")
				scene.shapes.push_back(parseShape(child));
			else if (node_name == "bsdf")
				scene.bsdfs.push_back(parseBSDF(child));
			else if (node_name == "emitter")
				scene.emitters.push_back(parseEmitter(child));
			else if (node_name == "default")
				this->add_default(child);
			else
				throw std::runtime_error(std::string("Unknown scene object: ") + node_name);
		}

		return scene;
	}

	std::unique_ptr<Integrator> parseIntegrator(const pugi::xml_node& node) {
		auto integrator = std::make_unique<Integrator>();
		integrator->type = get_default(node.attribute("type").value());
		
		parseProperties(node, integrator->properties);
		return integrator;
	}

	std::unique_ptr<Sensor> parseSensor(const pugi::xml_node& node) {
		auto sensor = std::make_unique<Sensor>();
		sensor->type = get_default(node.attribute("type").value());

		parseProperties(node, sensor->properties);

		// Parse nested objects
		for (pugi::xml_node child : node.children()) {
			std::string child_name = child.name();

			if (child_name == "film") {
				sensor->film = parseFilm(child);
			}
			else if (child_name == "sampler") {
				sensor->sampler = parseSampler(child);
			}
			else if (child_name == "transform") {
				sensor->to_world = parseTransform(child);
			}
		}

		return sensor;
	}

	std::unique_ptr<Shape> parseShape(const pugi::xml_node& node) {
		auto shape = std::make_unique<Shape>();
		shape->type = get_default(node.attribute("type").value());

		parseProperties(node, shape->properties);

		// Parse nested objects
		for (pugi::xml_node child : node.children()) {
			std::string child_name = child.name();

			if (child_name == "bsdf") {
				shape->bsdf = parseBSDF(child);
			}
			else if (child_name == "ref") {
				// Handle references to other objects
				// Store reference for later resolution
				std::string id = child.attribute("id").value();
				shape->bsdf = shared_bsdfs[id];
			}
			else if (child_name == "emitter") {
				shape->emitter = parseEmitter(child);
			}
			else if (child_name == "transform") {
				shape->to_world = parseTransform(child);
			}
		}

		return shape;
	}

	std::shared_ptr<BSDF> parseBSDF(const pugi::xml_node& node) {
		auto bsdf = std::make_shared<BSDF>();
		bsdf->type = get_default(node.attribute("type").value());
		if (node.attribute("id"))
		{
			bsdf->id = node.attribute("id").value();
			shared_bsdfs[bsdf->id] = bsdf;
		}

		parseProperties(node, bsdf->properties);
		return bsdf;
	}

	std::unique_ptr<Emitter> parseEmitter(const pugi::xml_node& node) {
		auto emitter = std::make_unique<Emitter>();
		emitter->type = get_default(node.attribute("type").value());

		parseProperties(node, emitter->properties);
		return emitter;
	}

	std::unique_ptr<Film> parseFilm(const pugi::xml_node& node) {
		auto film = std::make_unique<Film>();
		film->type = get_default(node.attribute("type").value());

		parseProperties(node, film->properties);
		return film;
	}

	std::unique_ptr<Sampler> parseSampler(const pugi::xml_node& node) {
		auto sampler = std::make_unique<Sampler>();
		sampler->type = get_default(node.attribute("type").value());

		parseProperties(node, sampler->properties);
		return sampler;
	}

	std::shared_ptr<SceneObject> parseGenericObject(const pugi::xml_node& node) {
		auto obj = std::make_shared<SceneObject>();
		obj->type = get_default(node.attribute("type").value());

		parseProperties(node, obj->properties);
		return obj;
	}

	void parseProperties(const pugi::xml_node& node, std::unordered_map<std::string, std::string>& properties) {
		for (pugi::xml_node child : node.children()) {
			std::string child_name = child.name();
			std::string name = child.attribute("name").value();

			if (child_name == "string") {
				properties[name] = get_default(child.attribute("value").value());
			}
			else if (child_name == "integer") {
				properties[name] = get_default(child.attribute("value").value());
			}
			else if (child_name == "float") {
				properties[name] = get_default(child.attribute("value").value());
			}
			else if (child_name == "boolean") {
				properties[name] = get_default(child.attribute("value").value());
			}
			else if (child_name == "point" || child_name == "vector" || child_name == "rgb") {
				properties[name] = get_default(child.attribute("value").value());
			}
		}
	}

	Mat4f parseTransform(const pugi::xml_node& node) {
		Mat4f transform = glm::mat<4, 4, Float>(1);

		for (pugi::xml_node child : node.children()) {
			std::string child_name = child.name();

			if (child_name == "translate") {
				float x = child.attribute("x") ? child.attribute("x").as_float() : 0.0f;
				float y = child.attribute("y") ? child.attribute("y").as_float() : 0.0f;
				float z = child.attribute("z") ? child.attribute("z").as_float() : 0.0f;

				if (child.attribute("value")) {
					std::string value_str = child.attribute("value").value();
					std::istringstream iss(value_str);
					char comma;
					iss >> x >> comma >> y >> comma >> z;
				}

				transform = translate(transform, Vec3f{ x, y, z });
			}
			else if (child_name == "rotate") {
				// Handle rotation - simplified version
				float angle = child.attribute("angle") ? child.attribute("angle").as_float() : 0.0f;
				float x = child.attribute("x") ? child.attribute("x").as_float() : 0.0f;
				float y = child.attribute("y") ? child.attribute("y").as_float() : 0.0f;
				float z = child.attribute("z") ? child.attribute("z").as_float() : 0.0f;

				if (child.attribute("value")) {
					std::string value_str = child.attribute("value").value();
					std::istringstream iss(value_str);
					char comma;
					iss >> x >> comma >> y >> comma >> z;
				}

				transform = rotate(transform, Vec3f{ x, y, z }, angle);
			}
			else if (child_name == "scale") {
				float x = child.attribute("x") ? child.attribute("x").as_float() : 1.0f;
				float y = child.attribute("y") ? child.attribute("y").as_float() : 1.0f;
				float z = child.attribute("z") ? child.attribute("z").as_float() : 1.0f;

				if (child.attribute("value")) {
					std::string value_str = child.attribute("value").value();
					std::istringstream iss(value_str);
					char comma;
					iss >> x >> comma >> y >> comma >> z;
				}

				transform = scale(transform, Vec3f{ x, y, z });
			}
			else if (child_name == "matrix") {
				Mat4f matrix;
				std::string value = child.attribute("value").value();
				std::istringstream iss(value);
				for (int i = 0; i < 16; ++i)
					iss >> matrix[i];

				transform = matrix * transform;
			}
			else if (child_name == "lookat") {
				std::string origin = child.attribute("origin").value();
				std::istringstream iss(origin);
				// TODO: 
				// throw std::runtime_error("lookat not implemented.");
				std::cout << "Warning: lookat not implemented. Returning identity." << std::endl;
			}
		}

		return transform;
	}

	void add_default(const pugi::xml_node& node) {
		defaults[node.attribute("name").value()] = node.attribute("value").value();
	}

	std::string get_default(const std::string& value) {
		if (value[0] == '$')
			return defaults[value.substr(1, value.length() - 1)];
		return value;
	}
};

