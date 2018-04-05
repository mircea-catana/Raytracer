#pragma once

#include <array>
#include <algorithm>
#include "ray.h"

namespace mcp
{
namespace math
{
    template <typename T, int Dimension>
    class AABB
    {
    public:
        AABB();
        AABB(const Vector<T, Dimension>& pMin, const Vector<T, Dimension>& pMax);

        Vector<T, Dimension> min() const;
        Vector<T, Dimension> max() const;

        AABB<T, Dimension> box_union(const AABB<T, Dimension>& other) const;
        AABB<T, Dimension> box_union(const Vector<T, Dimension>& other) const;

        uint32_t maximumExtent() const;
        T        surfaceArea()   const;

        bool intersect(const Ray<T, Dimension>& ray, T tMin, T tMax) const;

    private:
        Vector<T, Dimension> mMin;
        Vector<T, Dimension> mMax;
    };

    typedef AABB<float, 2>  AABB2f;
    typedef AABB<double, 2> AABB2d;
    typedef AABB<float, 3>  AABB3f;
    typedef AABB<double, 3> AABB3d;

    template <typename T, int Dimension>
    inline AABB<T, Dimension> box_union(const AABB<T, Dimension>& b1, const AABB<T, Dimension>& b2) {
        return b1.box_union(b2);
    }

    template <typename T, int Dimension>
    inline AABB<T, Dimension> box_union(const AABB<T, Dimension>& b, const Vector<T, Dimension>& v) {
        return b.box_union(v);
    }

    template <typename T, int Dimension>
    inline AABB<T, Dimension> box_union(const Vector<T, Dimension>& v, const AABB<T, Dimension>& b) {
        return b.box_union(v);
    }

    template <typename T, int Dimension>
    AABB<T, Dimension>::AABB()
        : mMin(static_cast<T>(0))
        , mMax(static_cast<T>(0))
    {
    }

    template <typename T, int Dimension>
    AABB<T, Dimension>::AABB(const Vector<T, Dimension>& p1, const Vector<T, Dimension>& p2)
    {
        for (int i = 0; i < Dimension; ++i) {
            mMin[i] = std::min(p1[i], p2[i]);
            mMax[i] = std::max(p1[i], p2[i]);
        }
    }

    template <typename T, int Dimension>
    Vector<T, Dimension> AABB<T, Dimension>::min() const {
        return mMin;
    }

    template <typename T, int Dimension>
    Vector<T, Dimension> AABB<T, Dimension>::max() const {
        return mMax;
    }

    template <typename T, int Dimension>
    AABB<T, Dimension> AABB<T, Dimension>::box_union(const AABB<T, Dimension>& other) const {
        Vector<T, Dimension> pMin;
        Vector<T, Dimension> pMax;

        for (int i = 0; i < Dimension; ++i) {
            pMin[i] = std::min(mMin[i], other.min()[i]);
            pMax[i] = std::max(mMax[i], other.max()[i]);
        }

        return AABB(pMin, pMax);
    }

    template <typename T, int Dimension>
    AABB<T, Dimension> AABB<T, Dimension>::box_union(const Vector<T, Dimension>& other) const {
        Vector<T, Dimension> pMin;
        Vector<T, Dimension> pMax;

        for (int i = 0; i < Dimension; ++i) {
            pMin[i] = std::min(mMin[i], other[i]);
            pMax[i] = std::max(mMax[i], other[i]);
        }

        return AABB(pMin, pMax);
    }

    template <typename T, int Dimension>
    uint32_t AABB<T, Dimension>::maximumExtent() const {
        Vector<T, Dimension> diagonal = mMax - mMin;

        for (int i = 0; i < Dimension - 1; ++i) {
            bool largest = true;
            for (int j = i + 1; j < Dimension; ++j) {
                if (diagonal[j] > diagonal[i]) {
                    largest = false;
                }
            }

            if (largest) {
                return static_cast<uint32_t>(i);
            }
        }

        return static_cast<uint32_t>(Dimension - 1);
    }

    template <typename T, int Dimension>
    T AABB<T, Dimension>::surfaceArea() const {
        Vector<T, Dimension> diagonal = mMax - mMin;
        T sum = static_cast<T>(0);

        for (int i = 0; i < Dimension - 1; ++i) {
            for (int j = i + 1; j < Dimension; ++j) {
                sum += diagonal[i] * diagonal[j];
            }
        }

        return static_cast<T>(2) * sum;
    }

    template <typename T, int Dimension>
    bool AABB<T, Dimension>::intersect(const Ray<T, Dimension>& ray, T tMin, T tMax) const {
        for (int i = 0; i < Dimension; ++i) {
            T t0 = std::min((mMin[i] - ray.origin()[i]) / ray.direction()[i],
                            (mMax[i] - ray.origin()[i]) / ray.direction()[i]);
            T t1 = std::max((mMin[i] - ray.origin()[i]) / ray.direction()[i],
                            (mMax[i] - ray.origin()[i]) / ray.direction()[i]);

            tMin = std::max(t0, tMin);
            tMax = std::min(t1, tMax);

            if (tMax <= tMin) {
                return false;
            }
        }

        return true;
    }
}
}
