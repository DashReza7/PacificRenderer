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
        Mesh,
        Sphere,
        Disk
    };

    std::vector<Geometry *> geometries{};
    BSDF *bsdf = nullptr;
    // if AreaLight
    Emitter *emitter = nullptr;
    Type type;

    /// @brief Samples a point on the surface of the shape.
    /// @param sample1 A 1D sample point in [0, 1].
    /// @param sample2 A 2D sample point in [0, 1]^2.
    /// @return A tuple containing the position, normal, and PDF of the sampled point.
    std::tuple<Vec3f, Vec3f, Float> sample_point_on_surface(Float sample1, const Vec2f &sample2) const;

    std::string to_string() {
        std::ostringstream oss;
        oss << "Shape";
        if (type == Type::Mesh)
            oss << "(OBJ)";
        else if (type == Type::Sphere)
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
