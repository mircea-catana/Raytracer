#pragma once

#include <array>
#include <cassert>
#include <cstdint>
#include <fstream>
#include <cstring>
#include <vector>

#include "ray.h"

namespace mcp
{
    using namespace math;

    template <typename T>
    struct Pixel
    {
        Pixel() : r(0), g(0), b(0), a(0) {}
        Pixel(T r, T g, T b) : r(r), g(g), b(b), a(0) {}
        Pixel(T r, T g, T b, T a) : r(r), g(g), b(b), a(a) {}

        T r;
        T g;
        T b;
        union {
            T a;
            T padding;
        };
    };

    typedef Pixel<uint8_t>  Pixel8u;
    typedef Pixel<uint32_t> Pixel32u;

    template <typename PixelType>
    class Film
    {
    public:
        Film() : mWidth(0), mHeight(0) {}
        Film(uint32_t width, uint32_t height);

        void write(const std::string& filePath) const;

        uint32_t width() const {
            return mWidth;
        }

        uint32_t height() const {
            return mHeight;
        }

        float aspectRatio() const {
            return static_cast<float>(mWidth) / mHeight;
        }

        std::vector<PixelType>& pixels() {
            return mPixels;
        }

        PixelType& pixel(uint32_t x, uint32_t y) {
            assert(x < mWidth && y < mHeight);
            return mPixels[y * mWidth + x];
        }

    private:
        uint32_t mWidth;
        uint32_t mHeight;
        std::vector<PixelType> mPixels;
    };

    template <typename PixelType>
    Film<PixelType>::Film(uint32_t width, uint32_t height)
        : mWidth(width)
        , mHeight(height)
    {
        mPixels.reserve(width * height);
    }

    template <typename PixelType>
    void Film<PixelType>::write(const std::string& filePath) const {
        std::ofstream fileStream;
        fileStream.open(filePath);

        fileStream << "P3\n" << mWidth << " " << mHeight << "\n255\n";

        for (uint32_t j = 0; j < mHeight; ++j) {
            const uint32_t jw = j * mWidth;
            for (uint32_t i = 0; i < mWidth; ++i) {
                const uint32_t idx = jw + i;
                fileStream << static_cast<int>(mPixels[idx].r) << " "
                           << static_cast<int>(mPixels[idx].g) << " "
                           << static_cast<int>(mPixels[idx].b) << "\n";
            }
        }

        fileStream.close();
    }

    class Camera
    {
    public:
        Camera(const Vector3f& position, const Vector3f& lookAt, float vericalFOV,
               float nearPlane, float farPlane, uint32_t width, uint32_t height);

        Ray3f getRay(float u, float v) const;

        float nearPlane() const {
            return mNearPlane;
        }

        float farPlane() const {
            return mFarPlane;
        }

        Film<Pixel8u>& film() {
            return mFilm;
        }

    private:
        Vector3f mPosition;
        Vector3f mLookAt;

        float mNearPlane;
        float mFarPlane;

        float mVerticalFOV;
        float mHorizontalFOV;

        float mViewportTop;
        float mViewportRight;

        Vector3f mCameraForward;
        Vector3f mCameraRight;
        Vector3f mCameraUp;

        Film<Pixel8u> mFilm;
    };


    Camera::Camera(const Vector3f& position, const Vector3f& lookAt, float verticalFOV,
                   float nearPlane, float farPlane, uint32_t width, uint32_t height)
        : mPosition(position)
        , mLookAt(lookAt)
        , mNearPlane(nearPlane)
        , mFarPlane(farPlane)
        , mVerticalFOV(verticalFOV)
    {
        mFilm = Film<Pixel8u>(width, height);

        float aspectRatio = mFilm.aspectRatio();
        mHorizontalFOV    = mVerticalFOV * aspectRatio;

        mViewportTop   = tan(mVerticalFOV / 2.f) * mNearPlane;
        mViewportRight = tan(mHorizontalFOV / 2.f) * mNearPlane;

        mCameraForward = normalize(mLookAt - mPosition);
        mCameraRight   = cross(Vector3f(0.f, 1.f, 0.f), mCameraForward);
        mCameraUp      = cross(mCameraForward, mCameraRight);
    }

    Ray3f Camera::getRay(float u, float v) const {
        // Transform (u, v) from [0, 1] to [-1, 1]
        u = u * 2.f - 1.f;
        v = v * 2.f - 1.f;

        Vector3f rayOrigin = mPosition + mCameraForward * mNearPlane;
        rayOrigin = rayOrigin + mCameraRight * mViewportRight * u;
        rayOrigin = rayOrigin + mCameraUp * mViewportTop * v;
        Vector3f rayDirection = normalize(rayOrigin - mPosition);

        return Ray3f(rayOrigin, rayDirection);
    }
}
