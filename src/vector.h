#pragma once

#include <cmath>
#include <algorithm>

namespace mcp
{
namespace math
{
    template <typename T, int Dimension>
    class Vector
    {
    public:
        static const int kDimension = Dimension;

        Vector();
        explicit Vector(T x);
        Vector(T x, T y);
        Vector(T x, T y, T z);
        Vector(T x, T y, T z, T w);
        template <int OtherDimension>
        explicit Vector(const Vector<T, OtherDimension>& other);
        Vector(const Vector<T, 2>& other, T z);
        Vector(const Vector<T, 2>& other, T z, T w);
        Vector(const Vector<T, 3>& other, T w);

        T  x() const;
        T  y() const;
        T  z() const;
        T  w() const;
        T& x();
        T& y();
        T& z();
        T& w();

        void set(T x, T y);
        void set(T x, T y, T z);
        void set(T x, T y, T z, T w);

        Vector<T, Dimension> abs() const;
        Vector<T, Dimension> min(const Vector& v) const;
        Vector<T, Dimension> max(const Vector& v) const;

        Vector<T, 2>&       vec2();
        const Vector<T, 2>& vec2() const;
        Vector<T, 3>&       vec3();
        const Vector<T, 3>& vec3() const;

        Vector<float, 2>    vec2f() const;
        Vector<double, 2>   vec2d() const;
        Vector<float, 3>    vec3f() const;
        Vector<double, 3>   vec3d() const;

        bool isZero(float epsilon = 1e-5f) const;
        bool isNormalized() const;

        T distance(const Vector<T, Dimension>& point) const;
        T squaredDistance(const Vector<T, Dimension>& point) const;

        Vector<T, Dimension> normalized() const;
        T magnitude() const;
        T squaredMagnitude() const;

        T  operator[] (int i) const;
        T& operator[] (int i);

    private:
        T mData[kDimension];
    };

    typedef Vector<int, 2>      Vector2i;
    typedef Vector<unsigned, 2> Vector2u;
    typedef Vector<float, 2>    Vector2f;
    typedef Vector<double, 2>   Vector2d;
    typedef Vector<int, 3>      Vector3i;
    typedef Vector<unsigned, 3> Vector3u;
    typedef Vector<float, 3>    Vector3f;
    typedef Vector<double, 3>   Vector3d;
    typedef Vector<int, 4>      Vector4i;
    typedef Vector<unsigned, 4> Vector4u;
    typedef Vector<float, 4>    Vector4f;
    typedef Vector<double, 4>   Vector4d;

    template <typename T, int Dimension>
    inline Vector<T, Dimension> normalize(const Vector<T, Dimension>& v) {
        return v.normalized();
    }

    template <typename T, int Dimension>
    inline Vector<T, Dimension> magnitude(const Vector<T, Dimension>& v) {
        return v.magnitude();
    }

    template <typename T, int Dimension>
    inline T dot(const Vector<T, Dimension>& v1, const Vector<T, Dimension>& v2) {
        T result = static_cast<T>(0);
        for (int i = 0; i < Dimension; ++i) {
            result += v1[i] * v2[i];
        }
        return result;
    }

    template <typename T>
    inline Vector<T, 3> cross(const Vector<T, 3>& v1, const Vector<T, 3>& v2) {
        return Vector<T, 3>(v1.y() * v2.z() - v1.z() * v2.y(),
                            v1.z() * v2.x() - v1.x() * v2.z(),
                            v1.x() * v2.y() - v1.y() * v2.x());
    }

    /// V2 operators
    template <typename T>
    Vector<T, 2> operator+ (T s, const Vector<T, 2>& v) {
        return Vector<T, 2>(s + v.x(), s + v.y());
    }
    template <typename T>
    Vector<T, 2> operator+ (const Vector<T, 2>& v, T s) {
        return Vector<T, 2>(v.x() + s, v.y() + s);
    }
    template <typename T>
    Vector<T, 2> operator+ (const Vector<T, 2>& v1, const Vector<T, 2>& v2) {
        return Vector<T, 2>(v1.x() + v2.x(), v1.y() + v2.y());
    }

    template <typename T>
    Vector<T, 2> operator- (T s, const Vector<T, 2>& v) {
        return Vector<T, 2>(s - v.x(), s - v.y());
    }
    template <typename T>
    Vector<T, 2> operator- (const Vector<T, 2>& v, T s) {
        return Vector<T, 2>(v.x() - s, v.y() - s);
    }
    template <typename T>
    Vector<T, 2> operator- (const Vector<T, 2>& v1, const Vector<T, 2>& v2) {
        return Vector<T, 2>(v1.x() - v2.x(), v1.y() - v2.y());
    }

    template <typename T>
    Vector<T, 2> operator* (T s, const Vector<T, 2>& v) {
        return Vector<T, 2>(s * v.x(), s * v.y());
    }
    template <typename T>
    Vector<T, 2> operator* (const Vector<T, 2>& v, T s) {
        return Vector<T, 2>(v.x() * s, v.y() * s);
    }
    template <typename T>
    Vector<T, 2> operator* (const Vector<T, 2>& v1, const Vector<T, 2>& v2) {
        return Vector<T, 2>(v1.x() * v2.x(), v1.y() * v2.y());
    }

    template <typename T>
    Vector<T, 2> operator/ (T s, const Vector<T, 2>& v) {
        return Vector<T, 2>(s / v.x(), s / v.y());
    }
    template <typename T>
    Vector<T, 2> operator/ (const Vector<T, 2>& v, T s) {
        return Vector<T, 2>(v.x() / s, v.y() / s);
    }
    template <typename T>
    Vector<T, 2> operator/ (const Vector<T, 2>& v1, const Vector<T, 2>& v2) {
        return Vector<T, 2>(v1.x() / v2.x(), v1.y() / v2.y());
    }

    /// V3 operators
    template <typename T>
    Vector<T, 3> operator+ (T s, const Vector<T, 3>& v) {
        return Vector<T, 3>(s + v.x(), s + v.y(), s + v.z());
    }
    template <typename T>
    Vector<T, 3> operator+ (const Vector<T, 3>& v, T s) {
        return Vector<T, 3>(v.x() + s, v.y() + s, v.z() + s);
    }
    template <typename T>
    Vector<T, 3> operator+ (const Vector<T, 3>& v1, const Vector<T, 3>& v2) {
        return Vector<T, 3>(v1.x() + v2.x(), v1.y() + v2.y(), v1.z() + v2.z());
    }

    template <typename T>
    Vector<T, 3> operator- (T s, const Vector<T, 3>& v) {
        return Vector<T, 3>(s - v.x(), s - v.y(), s - v.z());
    }
    template <typename T>
    Vector<T, 3> operator- (const Vector<T, 3>& v, T s) {
        return Vector<T, 3>(v.x() - s, v.y() - s, v.z() - s);
    }
    template <typename T>
    Vector<T, 3> operator- (const Vector<T, 3>& v1, const Vector<T, 3>& v2) {
        return Vector<T, 3>(v1.x() - v2.x(), v1.y() - v2.y(), v1.z() - v2.z());
    }

    template <typename T>
    Vector<T, 3> operator* (T s, const Vector<T, 3>& v) {
        return Vector<T, 3>(s * v.x(), s * v.y(), s * v.z());
    }
    template <typename T>
    Vector<T, 3> operator* (const Vector<T, 3>& v, T s) {
        return Vector<T, 3>(v.x() * s, v.y() * s, v.z() * s);
    }
    template <typename T>
    Vector<T, 3> operator* (const Vector<T, 3>& v1, const Vector<T, 3>& v2) {
        return Vector<T, 3>(v1.x() * v2.x(), v1.y() * v2.y(), v1.z() * v2.z());
    }

    template <typename T>
    Vector<T, 3> operator/ (T s, const Vector<T, 3>& v) {
        return Vector<T, 3>(s / v.x(), s / v.y(), s / v.z());
    }
    template <typename T>
    Vector<T, 3> operator/ (const Vector<T, 3>& v, T s) {
        return Vector<T, 3>(v.x() / s, v.y() / s, v.z() / s);
    }
    template <typename T>
    Vector<T, 3> operator/ (const Vector<T, 3>& v1, const Vector<T, 3>& v2) {
        return Vector<T, 3>(v1.x() / v2.x(), v1.y() / v2.y(), v1.z() / v2.z());
    }

    /////////////////////////////////////////////////////////////////////

    template <typename T, int Dimension>
    Vector<T, Dimension>::Vector() {
    }

    template <typename T, int Dimension>
    Vector<T, Dimension>::Vector(T x) {
        for (int i = 0; i < Dimension; ++i) {
            mData[i] = x;
        }
    }

    template <typename T, int Dimension>
    Vector<T, Dimension>::Vector(T x, T y) : mData{x, y} {
    }

    template <typename T, int Dimension>
    Vector<T, Dimension>::Vector(T x, T y, T z) : mData{x, y, z} {
    }

    template <typename T, int Dimension>
    Vector<T, Dimension>::Vector(T x, T y, T z, T w) : mData{x, y, z, w} {
    }

    template <typename T, int Dimension>
    template <int OtherDimension>
    Vector<T, Dimension>::Vector(const Vector<T, OtherDimension>& other) {
        ((void)(other));
        // TODO: do this thing
    }

    template <typename T, int Dimension>
    Vector<T, Dimension>::Vector(const Vector<T, 2>& other, T z)
        : mData{other.x(), other.y(), z}
    {
    }

    template <typename T, int Dimension>
    Vector<T, Dimension>::Vector(const Vector<T, 2>& other, T z, T w)
        : mData{other.x(), other.y(), z, w}
    {
    }

    template <typename T, int Dimension>
    Vector<T, Dimension>::Vector(const Vector<T, 3>& other, T w)
        : mData{other.x(), other.y(), other.z(), w}
    {
    }

    template <typename T, int Dimension>
    T Vector<T, Dimension>::x() const {
        return mData[0];
    }

    template <typename T, int Dimension>
    T Vector<T, Dimension>::y() const {
        return mData[1];
    }

    template <typename T, int Dimension>
    T Vector<T, Dimension>::z() const {
        return mData[2];
    }

    template <typename T, int Dimension>
    T Vector<T, Dimension>::w() const {
        return mData[3];
    }

    template <typename T, int Dimension>
    T& Vector<T, Dimension>::x() {
        return mData[0];
    }

    template <typename T, int Dimension>
    T& Vector<T, Dimension>::y() {
        return mData[1];
    }

    template <typename T, int Dimension>
    T& Vector<T, Dimension>::z() {
        return mData[2];
    }

    template <typename T, int Dimension>
    T& Vector<T, Dimension>::w() {
        return mData[3];
    }

    template <typename T, int Dimension>
    void Vector<T, Dimension>::set(T x, T y) {
        mData[0] = x;
        mData[1] = y;
    }

    template <typename T, int Dimension>
    void Vector<T, Dimension>::set(T x, T y, T z) {
        mData[0] = x;
        mData[1] = y;
        mData[2] = z;
    }

    template <typename T, int Dimension>
    void Vector<T, Dimension>::set(T x, T y, T z, T w) {
        mData[0] = x;
        mData[1] = y;
        mData[2] = z;
        mData[3] = w;
    }

    template <typename T, int Dimension>
    inline Vector<T, Dimension> Vector<T, Dimension>::abs() const {
        Vector<T, Dimension> result;
        for (int i = 0; i < Dimension; ++i) {
            result[i] = abs(mData[i]);
        }
        return result;
    }

    template <typename T, int Dimension>
    inline Vector<T, Dimension> Vector<T, Dimension>::min(const Vector& v) const {
        Vector<T, Dimension> result;
        for (int i = 0; i < Dimension; ++i) {
            result[i] = std::min(mData[i], v[i]);
        }
        return result;
    }

    template <typename T, int Dimension>
    inline Vector<T, Dimension> Vector<T, Dimension>::max(const Vector& v) const {
        Vector<T, Dimension> result;
        for (int i = 0; i < Dimension; ++i) {
            result[i] = std::max(mData[i], v[i]);
        }
        return result;
    }

    template <typename T, int Dimension>
    inline Vector<T, 2>& Vector<T, Dimension>::vec2() {
        return reinterpret_cast<Vector<T, 2>&>(*this);
    }

    template <typename T, int Dimension>
    inline const Vector<T, 2>& Vector<T, Dimension>::vec2() const {
        return reinterpret_cast<const Vector<T, 2>&>(*this);
    }

    template <typename T, int Dimension>
    inline Vector<T, 3>& Vector<T, Dimension>::vec3() {
        return reinterpret_cast<Vector<T, 3>&>(*this);
    }

    template <typename T, int Dimension>
    inline const Vector<T, 3>& Vector<T, Dimension>::vec3() const {
        return reinterpret_cast<const Vector<T, 3>&>(*this);
    }

    template <typename T, int Dimension>
    inline Vector<float, 2> Vector<T, Dimension>::vec2f() const {
        return Vector2f(static_cast<float>(x()), static_cast<float>(y()));
    }

    template <typename T, int Dimension>
    inline Vector<double, 2> Vector<T, Dimension>::vec2d() const {
        return Vector2d(static_cast<double>(x()), static_cast<double>(y()));
    }

    template <typename T, int Dimension>
    inline Vector<float, 3> Vector<T, Dimension>::vec3f() const {
        return Vector3f(static_cast<float>(x()), static_cast<float>(y()), static_cast<float>(z()));
    }

    template <typename T, int Dimension>
    inline Vector<double, 3> Vector<T, Dimension>::vec3d() const {
        return Vector3d(static_cast<double>(x()), static_cast<double>(y()), static_cast<double>(z()));
    }

    template <typename T, int Dimension>
    inline bool Vector<T, Dimension>::isZero(float epsilon) const {
        for (int i = 0; i < Dimension; ++i) {
            if (abs(mData[i] > epsilon)) return false;
        }
        return true;
    }

    template <typename T, int Dimension>
    inline bool Vector<T, Dimension>::isNormalized() const {
        return magnitude() == static_cast<T>(1);
    }

    template <typename T, int Dimension>
    inline T Vector<T, Dimension>::distance(const Vector<T, Dimension>& point) const {
        const Vector<T, Dimension> distance = *this - point;
        return distance.magnitude();
    }

    template <typename T, int Dimension>
    inline T Vector<T, Dimension>::squaredDistance(const Vector<T, Dimension>& point) const {
        const Vector<T, Dimension> distance = *this - point;
        return distance.squaredMagnitude();
    }

    template <typename T, int Dimension>
    inline Vector<T, Dimension> Vector<T, Dimension>::normalized() const {
        return *this / magnitude();
    }

    template <typename T, int Dimension>
    inline T Vector<T, Dimension>::magnitude() const {
        return sqrt(squaredMagnitude());
    }

    template <typename T, int Dimension>
    inline T Vector<T, Dimension>::squaredMagnitude() const {
        T result = static_cast<T>(0);
        for (int i = 0; i < Dimension; ++i) {
            result += mData[i] * mData[i];
        }
        return result;
    }

    template <typename T, int Dimension>
    T  Vector<T, Dimension>::operator[] (int i) const {
        return mData[i];
    }

    template <typename T, int Dimension>
    T& Vector<T, Dimension>::operator[] (int i) {
        return mData[i];
    }
}
}
