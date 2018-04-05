#pragma once

#include "vector.h"

namespace mcp
{
    struct HitInfo
    {
        float          t;
        math::Vector3f point;
        math::Vector3f normal;

        float u;
        float v;
    };
}
