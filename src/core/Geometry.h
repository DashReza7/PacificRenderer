#pragma once

#include <core/Primitives.h>

#include <array>
#include <memory>
#include <optional>
#include <vector>

#include "core/MathUtils.h"
#include "core/Pacific.h"

class Geometry {
   public:
    std::shared_ptr<AABB> bbox;

    Geometry() = default;
    virtual ~Geometry() = default;

    virtual bool intersect(const Ray &ray, Intersection &isc) = 0;
    virtual void build_bbox() = 0;
    virtual Vec3f get_normal(const Vec3f &position) = 0;
};

class Triangle : public Geometry {
   public:
    Triangle() = default;
    Triangle(const Vec3f &a, const Vec3f &b, const Vec3f &c) {
        positions = std::array<Vec3f, 3>{a, b, c};
    }
    Triangle(const Vec3f &a, const Vec3f &b, const Vec3f &c, const Vec3f &an, const Vec3f &bn,
             const Vec3f &cn) {
        positions = std::array<Vec3f, 3>{a, b, c};
        normals = std::array<Vec3f, 3>{an, bn, cn};
    }
    Triangle(const Vec3f &a, const Vec3f &b, const Vec3f &c, const Vec2f &at, const Vec2f &bt,
             const Vec2f &ct) {
        positions = std::array<Vec3f, 3>{a, b, c};
        tex_coords = std::array<Vec2f, 3>{at, bt, ct};
    }
    Triangle(const Vec3f &a, const Vec3f &b, const Vec3f &c, const Vec3f &an, const Vec3f &bn,
             const Vec3f &cn, const Vec2f &at, const Vec2f &bt, const Vec2f &ct) {
        positions = std::array<Vec3f, 3>{a, b, c};
        normals = std::array<Vec3f, 3>{an, bn, cn};
        tex_coords = std::array<Vec2f, 3>{at, bt, ct};
    }

    bool intersect(const Ray &ray, Intersection &isc) override {
        throw std::runtime_error("Triangle intersection not implemented");
    }

    void build_bbox() override { throw std::runtime_error("Triangle build_bbox not implemented"); }

    Vec3f get_normal(const Vec3f &position) override {
        // based on face_normals, either interpolate vn's or compute the (constant)
        // normal manually
        throw std::runtime_error("Triangle get_normal not implemented");
    }

   private:
    std::array<Vec3f, 3> positions;
    std::optional<std::array<Vec3f, 3>> normals;
    std::optional<std::array<Vec2f, 3>> tex_coords;
};

class Quad : public Geometry {
   public:
    Quad() = default;
    Quad(const Vec3f &a, const Vec3f &b, const Vec3f &c, const Vec3f &d) {
        positions = std::array<Vec3f, 4>{a, b, c, d};
    }
    Quad(const Vec3f &a, const Vec3f &b, const Vec3f &c, const Vec3f &d, const Vec3f &an,
         const Vec3f &bn, const Vec3f &cn, const Vec3f &dn) {
        positions = std::array<Vec3f, 4>{a, b, c, d};
        normals = std::array<Vec3f, 4>{an, bn, cn, dn};
    }
    Quad(const Vec3f &a, const Vec3f &b, const Vec3f &c, const Vec3f &d, const Vec2f &at,
         const Vec2f &bt, const Vec2f &ct, const Vec2f &dt) {
        positions = std::array<Vec3f, 4>{a, b, c, d};
        tex_coords = std::array<Vec2f, 4>{at, bt, ct, dt};
    }
    Quad(const Vec3f &a, const Vec3f &b, const Vec3f &c, const Vec3f &d, const Vec3f &an,
         const Vec3f &bn, const Vec3f &cn, const Vec3f &dn, const Vec2f &at, const Vec2f &bt,
         const Vec2f &ct, const Vec2f &dt) {
        positions = std::array<Vec3f, 4>{a, b, c, d};
        normals = std::array<Vec3f, 4>{an, bn, cn, dn};
        tex_coords = std::array<Vec2f, 4>{at, bt, ct, dt};
    }

    bool intersect(const Ray &ray, Intersection &isc) override {
        throw std::runtime_error("Quad intersection not implemented");
    }

    void build_bbox() override { throw std::runtime_error("Quad build_bbox not implemented"); }

    Vec3f get_normal(const Vec3f &position) override {
        // based on face_normals, either interpolate vn's or compute the (constant)
        // normal manually
        throw std::runtime_error("Quad get_normal not implemented");
    }

   private:
    std::array<Vec3f, 4> positions;
    std::optional<std::array<Vec3f, 4>> normals;
    std::optional<std::array<Vec2f, 4>> tex_coords;
};

class Sphere : public Geometry {
   public:
    Sphere() = default;
    Sphere(const Vec3f &center, Float radius) : center(center), radius(radius) {}
    ~Sphere() = default;

    bool intersect(const Ray &ray, Intersection &isc) override {
        throw std::runtime_error("Sphere intersect not implemented");
    }

    void build_bbox() override { throw std::runtime_error("Sphere build_bbox not implemented"); }

    Vec3f get_normal(const Vec3f &position) override {
        throw std::runtime_error("Sphere get_normal not implemented");
    }

   private:
    Vec3f center;
    Float radius;
};
