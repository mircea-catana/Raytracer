#pragma once

#include "shape.h"

namespace mcp
{
namespace geometry
{
    using namespace math;

    template <typename T>
    class Sphere : public Shape
    {
    public:
        Sphere();
        Sphere(const Vector<T, 3>& center, T radius);

        Vector<T, 3> center() const;
        T            radius() const;

        bool intersect(const Ray3f& ray, float tMin, float tMax, HitInfo& info) const override;
        bool intersect_fast(const Ray3f& ray, float tMin, float tMax, float& t) const override;
        AABB3f aabb() const override;

    private:
        Vector<T, 3> mCenter;
        T            mRadius;
        AABB<T, 3>   mAABB;
    };

    typedef Sphere<float>  Spheref;
    typedef Sphere<double> Sphered;

    template <typename T>
    Sphere<T>::Sphere()
        : mCenter(static_cast<T>(0))
        , mRadius(static_cast<T>(1))
    {
        Vector<T, 3> pMin = mCenter - Vector<T, 3>(mRadius, mRadius, mRadius);
        Vector<T, 3> pMax = mCenter + Vector<T, 3>(mRadius, mRadius, mRadius);
        mAABB = AABB<T, 3>(pMin, pMax);
    }

    template <typename T>
    Sphere<T>::Sphere(const Vector<T, 3>& center, T radius)
        : mCenter(center)
        , mRadius(radius)
    {
        Vector<T, 3> pMin = mCenter - Vector<T, 3>(mRadius, mRadius, mRadius);
        Vector<T, 3> pMax = mCenter + Vector<T, 3>(mRadius, mRadius, mRadius);
        mAABB = AABB<T, 3>(pMin, pMax);
    }

    template <typename T>
    Vector<T, 3> Sphere<T>::center() const {
        return mCenter;
    }

    template <typename T>
    T Sphere<T>::radius() const {
        return mRadius;
    }

    template <typename T>
    AABB3f Sphere<T>::aabb() const {
        return mAABB;
    }

    template <typename T>
    bool Sphere<T>::intersect(const Ray3f& ray, float tMin, float tMax, HitInfo& info) const {
        Vector<T, 3> oc = ray.origin() - mCenter;

        T a = dot(ray.direction(), ray.direction());
        T b = dot(oc, ray.direction());
        T c = dot(oc, oc) - mRadius * mRadius;

        T disc = b * b - a * c;
        if (disc > static_cast<T>(0)) {
            T t = (-b - sqrt(disc)) / a;
            if (t > tMin && t < tMax) {
                info.t      = t;
                info.point  = ray.origin() + t * ray.direction();
                info.normal = normalize(info.point - mCenter);

                Vector3f cs = info.normal * -1.f;
                info.u = 0.5f + atan2f(cs.z(), cs.x()) / kPI * 2.0f;
                info.v = 0.5f - asinf(cs.y()) / kPI;

                return true;
            }

            t = (-b + sqrt(disc)) / a;
            if (t > tMin && t < tMax) {
                info.t      = t;
                info.point  = ray.origin() + t * ray.direction();
                info.normal = normalize(info.point - mCenter);

                Vector3f cs = info.normal * -1.f;
                info.u = 0.5f + atan2f(cs.z(), cs.x()) / kPI * 2.0f;
                info.v = 0.5f - asinf(cs.y()) / kPI;

                return true;
            }
        }

        return false;
    }

    template <typename T>
    bool Sphere<T>::intersect_fast(const Ray3f& ray, float tMin, float tMax, float& t) const {
        Vector<T, 3> oc = ray.origin() - mCenter;

        T a = dot(ray.direction(), ray.direction());
        T b = dot(oc, ray.direction());
        T c = dot(oc, oc) - mRadius * mRadius;

        T disc = b * b - a * c;
        if (disc > static_cast<T>(0)) {
            t = (-b - sqrt(disc)) / a;
            if (t > tMin && t < tMax) {
                return true;
            }

            t = (-b + sqrt(disc)) / a;
            if (t > tMin && t < tMax) {
                return true;
            }

            t = static_cast<T>(0);
        }

        return false;
    }
}
}
