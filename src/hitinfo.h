#pragma once

#include "vector.h"

namespace mcp
{
    template <typename T>
    struct HitInfo
    {
        T            t;
        math::Vector<T, 3> point;
        math::Vector<T, 3> normal;

        float u;
        float v;
    };
}
