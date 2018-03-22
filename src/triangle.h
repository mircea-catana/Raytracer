#pragma once

#include "ray.h"

namespace mcp
{
namespace geometry
{
    using namespace math;

    template <typename T>
    class Triangle
    {
    public:
        Triangle();
        Triangle(const Vector<T, 3>& v1, const Vector<T, 3>& v2, const Vector<T, 3>& v3);

        Vector<T, 3>  v1() const;
        Vector<T, 3>& v1();
        Vector<T, 3>  v2() const;
        Vector<T, 3>& v2();
        Vector<T, 3>  v3() const;
        Vector<T, 3>& v3();

        Vector<T, 3> normal() const;
        AABB<T, 3>   aabb()   const;

        bool intersect(const Ray<T, 3>& ray, T tMin, T tMax, HitInfo<T>& info) const;
        bool intersect_fast(const Ray<T, 3>& ray, T tMin, T tMax, T& t) const;

    private:
        Vector<T, 3> mV1;
        Vector<T, 3> mV2;
        Vector<T, 3> mV3;

        Vector<T, 3> mFlatNormal;

        AABB<T, 3>   mAABB;
    };

    typedef Triangle<float>  Trianglef;
    typedef Triangle<double> Triangled;

    template <typename T>
    Triangle<T>::Triangle()
        : mV1(Vector<T, 3>(static_cast<T>(0), static_cast<T>(0), static_cast<T>(0)))
        , mV2(Vector<T, 3>(static_cast<T>(0), static_cast<T>(0), static_cast<T>(0)))
        , mV3(Vector<T, 3>(static_cast<T>(0), static_cast<T>(0), static_cast<T>(0)))
    {
    }

    template <typename T>
    Triangle<T>::Triangle(const Vector<T, 3>& v1, const Vector<T, 3>& v2, const Vector<T, 3>& v3)
        : mV1(v1)
        , mV2(v2)
        , mV3(v3)
    {
        Vector<T, 3> v1v2 = v2 - v1;
        Vector<T, 3> v1v3 = v3 - v1;

        //mFlatNormal = normalize(cross(v1v2, v1v3));
        mFlatNormal = cross(v1v2, v1v3);

        Vector<T, 3> pMin(std::min(v1.x(), v2.x()), std::min(v1.y(), v2.y()), std::min(v1.z(), v2.z()));
        pMin.x() = std::min(pMin.x(), v3.x());
        pMin.y() = std::min(pMin.y(), v3.y());
        pMin.z() = std::min(pMin.z(), v3.z());

        Vector<T, 3> pMax(std::max(v1.x(), v2.x()), std::max(v1.y(), v2.y()), std::max(v1.z(), v2.z()));
        pMax.x() = std::max(pMax.x(), v3.x());
        pMax.y() = std::max(pMax.y(), v3.y());
        pMax.z() = std::max(pMax.z(), v3.z());

        mAABB = AABB<T, 3>(pMin, pMax);
    }

    template <typename T>
    Vector<T, 3> Triangle<T>::v1() const {
        return mV1;
    }

    template <typename T>
    Vector<T, 3>& Triangle<T>::v1() {
        return mV1;
    }

    template <typename T>
    Vector<T, 3> Triangle<T>::v2() const {
        return mV2;
    }

    template <typename T>
    Vector<T, 3>& Triangle<T>::v2() {
        return mV2;
    }

    template <typename T>
    Vector<T, 3> Triangle<T>::v3() const {
        return mV3;
    }

    template <typename T>
    Vector<T, 3>& Triangle<T>::v3() {
        return mV3;
    }

    template <typename T>
    Vector<T, 3> Triangle<T>::normal() const {
        return mFlatNormal;
    }

    template <typename T>
    AABB<T, 3> Triangle<T>::aabb() const {
        return mAABB;
    }

    template <typename T>
    bool Triangle<T>::intersect(const Ray<T, 3>& ray, T tMin, T tMax, HitInfo<T>& info) const {
        if (!intersect_fast(ray, tMin, tMax, info.t)) {
            return false;
        }

        info.point = ray.origin() + info.t * ray.direction();
        info.normal = mFlatNormal;

        Vector<T, 3> e0 = mV2 - mV1;
        Vector<T, 3> e1 = mV3 - mV1;
        Vector<T, 3> e2 = info.point - mV1;

        float d00 = dot(e0, e0);
        float d01 = dot(e0, e1);
        float d11 = dot(e1, e1);
        float d20 = dot(e2, e0);
        float d21 = dot(e2, e1);
        float denom = d00 * d11 - d01 * d01;

        float v = (d11 * d20 - d01 * d21) / denom;
        float w = (d00 * d21 - d01 * d20) / denom;
        float u = static_cast<T>(1) - v - w;

        info.u = u;
        info.v = v;

        return true;
    }

    template <typename T>
    bool Triangle<T>::intersect_fast(const Ray<T, 3>& ray, T tMin, T tMax, T& t) const {
        Vector<T, 3> v1v2 = mV2 - mV1;
        Vector<T, 3> v1v3 = mV3 - mV1;

        Vector<T, 3> pVec = cross(ray.direction(), v1v3);
        T det = dot(v1v2, pVec);

        if (std::abs(det) < kEpsilon) {
            return false;
        }

        T invDet = static_cast<T>(1) / det;

        Vector<T, 3> tVec = ray.origin() - mV1;
        T u = dot(tVec, pVec) * invDet;
        if (u < static_cast<T>(0) || u > static_cast<T>(1)) {
            return false;
        }

        Vector<T, 3> qVec = cross(tVec, v1v2);
        T v = dot(ray.direction(), qVec) * invDet;
        if (v < static_cast<T>(0) || u + v > static_cast<T>(1)) {
            return false;
        }

        t = dot(v1v3, qVec) * invDet;
        if (t < tMin || t > tMax) {
            return false;
        }

        return true;
    }
}
}

