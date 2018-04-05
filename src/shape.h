#pragma once

#include <iostream>

#include "aabb.h"
#include "ray.h"
#include "hitinfo.h"

namespace mcp
{
namespace geometry
{

    class Shape
    {
    public:
        Shape();
        virtual ~Shape();

        uint32_t id() const;
        virtual bool intersect(const Ray3f& ray, float tMin, float tMax, HitInfo& info) const;
        virtual bool intersect_fast(const Ray3f& ray, float tMin, float tMax, float& t) const;
        virtual AABB3f aabb() const;

    private:
        static uint32_t mShapeCounter;
        const  uint32_t mShapeId;
    };

    uint32_t Shape::mShapeCounter = 1;

    Shape::Shape() : mShapeId(mShapeCounter++)
    {
    }

    Shape::~Shape()
    {
    }

    uint32_t Shape::id() const {
        return mShapeId;
    }

    bool Shape::intersect(const Ray3f& ray, float tMin, float tMax, HitInfo& info) const {
        std::cerr << "Error: Called unimplemented intersect method on shape: " << mShapeId << std::endl;
        return false;
    }

    bool Shape::intersect_fast(const Ray3f& ray, float tMin, float tMax, float& t) const {
        std::cerr << "Error: Called unimplemented intersect_fast method on shape: " << mShapeId << std::endl;
        return false;
    }

    AABB3f Shape::aabb() const {
        std::cerr << "Error: Called unimplemented aabb method on shape: " << mShapeId << std::endl;
        const float zero = 0.f;
        return AABB3f(Vector3f(zero, zero, zero),
                      Vector3f(zero, zero, zero));
    }
}
}
