#pragma once

#include <cstdint>
#include <cfloat>
#include <random>

namespace mcp
{
    static const float kEpsilon  = 1e-6f;
    static const float kInfinity = FLT_MAX;
    static const float kPI       = 3.14159265358979323846f;

    template <typename T>
    inline T clamp(T value, T min, T max) {
        if (value < min) {
            return min;
        }

        if (value > max) {
            return max;
        }

        return value;
    }

    template <typename T>
    inline T clamp01(T value) {
        return clamp(value, static_cast<T>(0), static_cast<T>(1));
    }

    inline float random01() {
        static std::random_device rd;
        static std::mt19937 mt(rd());
        static std::uniform_real_distribution<float> dist(0.f, 1.f);
        return dist(mt);
    }

    template <typename T>
    inline T random(T min, T max) {
        return min + (max - min) * random01();
    }

}
