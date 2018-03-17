#pragma once

#include <array>
#include <algorithm>
#include "vector.h"

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
}
}
