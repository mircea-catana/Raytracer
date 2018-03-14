#include <iostream>
#include "vector.h"

using namespace mcp::math;

int main()
{
    Vector3f a(1.f, 0.f, 0.f);
    Vector3f b(0.f, 1.f, 0.f);

    auto c = cross(a, b);

    std::cout << c.x() << " " << c.y() << " " << c.z() << std::endl;

    return 0;
}
