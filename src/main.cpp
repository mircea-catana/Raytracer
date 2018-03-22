#include <iostream>

#include "util.h"
#include "camera.h"
#include "sphere.h"
#include "triangle.h"

using namespace mcp::math;
using namespace mcp::geometry;

Spheref   gSphere   = Spheref(Vector3f(0.f, 0.f, 0.f), 1.f);
Trianglef gTriangle = Trianglef(Vector3f(-1.f,  1.0f, 0.f),
                                Vector3f( 1.f,  1.0f, 0.f),
                                Vector3f( 0.f, -1.0f, 0.f));

void render(const Ray3f& ray, mcp::Pixel8u& pixel)
{
    mcp::HitInfo<float> info;
    if (gTriangle.intersect(ray, 0.1f, 100.0f, info)) {
        pixel.r = 255 * info.u;
        pixel.g = 255 * info.v;
        pixel.b = 255 * (1.f - info.u - info.v);
    }
}

int main()
{
    Vector3f cameraPosition(0.f, 0.f, -1.f);
    Vector3f cameraLookAt(0.f, 0.f, 0.f);
    uint32_t width  = 500;
    uint32_t height = 500;
    mcp::Camera camera(cameraPosition, cameraLookAt, 40.f, 0.1f, 100.f, width, height);

    for (uint32_t i = 0; i < width * height; ++i) {
        camera.film().pixels().push_back(mcp::Pixel8u(0, 0, 0));
    }

    for (uint32_t j = 0; j < height; ++j) {
        for (uint32_t i = 0; i < width; ++i) {
            const float u = (static_cast<float>(i) + 0.5f) / static_cast<float>(width);
            const float v = (static_cast<float>(j) + 0.5f) / static_cast<float>(height);
            Ray3f cameraRay = camera.getRay(u, v);

            mcp::Pixel8u& pixel = camera.film().pixel(i, j);

            render(cameraRay, pixel);
        }
    }

    camera.film().write(std::string("image.ppm"));

    return 0;
}
