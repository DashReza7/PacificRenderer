#pragma once

#include <memory>
#include <vector>

#include "core/BSDF.h"
#include "core/Geometry.h"
#include "core/Emitter.h"

class Geometry;

class Shape {
public:
    enum class Type {
        OBJ,
        SPHERE,
    };

    std::vector<Geometry *> geometries{};
    BSDF *bsdf = nullptr;
    Emitter *emitter = nullptr;
    Type type;

    std::string to_string() {
        std::ostringstream oss;
        oss << "Shape";
        if (type == Type::OBJ)
            oss << "(OBJ)";
        else if (type == Type::SPHERE)
            oss << "(Sphere)";
        if (emitter)
            oss << "(Emitter)";
        oss << ":\n";
        oss << "  " << bsdf->to_string() << "\n";
        oss << "  Geometries" << "(" << geometries.size() << "):";
        if (geometries.size() == 0)
            oss << " None\n";
        else if (geometries.size() > 10)
            oss << " (Too many to display)\n";
        else {
            oss << "\n";
            for (const auto &geom : geometries)
               oss << "    - " << geom->to_string() << "\n";
        }

        return oss.str();
    }
};
