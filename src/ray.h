#pragma once

#include "vector.h"

namespace mcp
{
namespace math
{

    template <typename T, int Dimension>
    class Ray
    {
    public:
        Ray();
        Ray(const Vector<T, Dimension>& origin, const Vector<T, Dimension>& direction);

        void set(const Vector<T, Dimension>& origin, const Vector<T, Dimension>& direction);

        Vector<T, Dimension> origin() const;
        Vector<T, Dimension> direction() const;

        Vector<T, Dimension> parametric(T t) const;

    private:
        Vector<T, Dimension> mOrigin;
        Vector<T, Dimension> mDirection;
    };

    typedef Ray<float, 3>  Ray3f;
    typedef Ray<double, 3> Ray3d;

    template <typename T, int Dimension>
    Ray<T, Dimension>::Ray()
        : mOrigin(static_cast<T>(0))
        , mDirection(static_cast<T>(0))
    {
    }

    template <typename T, int Dimension>
    Ray<T, Dimension>::Ray(const Vector<T, Dimension>& origin, const Vector<T, Dimension>& direction)
        : mOrigin(origin)
        , mDirection(normalize(direction))
    {
    }

    template <typename T, int Dimension>
    void Ray<T, Dimension>::set(const Vector<T, Dimension>& origin, const Vector<T, Dimension>& direction) {
        mOrigin = origin;
        mDirection = direction;
    }

    template <typename T, int Dimension>
    Vector<T, Dimension> Ray<T, Dimension>::origin() const {
        return mOrigin;
    }

    template <typename T, int Dimension>
    Vector<T, Dimension> Ray<T, Dimension>::direction() const {
        return mDirection;
    }

    template <typename T, int Dimension>
    Vector<T, Dimension> Ray<T, Dimension>::parametric(T t) const {
        return mOrigin + t * mDirection;
    }
}
}
