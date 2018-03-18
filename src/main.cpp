#include <iostream>

#include "util.h"
#include "sphere.h"

using namespace mcp::math;
using namespace mcp::geometry;

int main()
{
    Vector3f c(3.f, 2.f, 3.f);
    Spheref s(c, 2.f);

    auto box = s.aabb();

    std::cout << box.min().x() << " " << box.min().y() << " " << box.min().z() << " - ";
    std::cout << box.max().x() << " " << box.max().y() << " " << box.max().z() << std::endl;

    auto r = mcp::random(2.0, 5.0);
    std::cout << r << std::endl;

    return 0;
}
