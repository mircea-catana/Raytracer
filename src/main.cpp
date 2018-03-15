#include <iostream>
#include "ray.h"

using namespace mcp::math;

int main()
{
    Vector3f a(1.f, 0.f, 0.f);
    Vector3f b(0.f, 1.f, 0.f);

    auto c = cross(a, b);

    std::cout << c.x() << " " << c.y() << " " << c.z() << std::endl;

    Ray3f r(Vector3f(1.f, 1.f, 0.f), a);

    auto d = r.parametric(0.5f);

    std::cout << d.x() << " " << d.y() << " " << d.z() << std::endl;

    return 0;
}
