#include <iostream>
#include "aabb.h"
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

    AABB3f b1(a, b);
    AABB3f b2(a, b + Vector3f(1.f, 1.f, 1.f));
    auto e = box_union(b1, b2);

    std::cout << e.min().x() << " " << e.min().y() << " " << e.min().z() << " - ";
    std::cout << e.max().x() << " " << e.max().y() << " " << e.max().z() << std::endl;

    return 0;
}
