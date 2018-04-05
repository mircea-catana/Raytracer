#pragma once

#include "aggregate.h"
#include "memory.h"

#include <vector>
#include <functional>

namespace mcp
{
namespace accelerator
{
    using namespace math;

    class BVH : public Aggregate
    {
    public:
        enum BuildMethod {
            eMIDDLE,
            eEQUAL_COUNT,
            eSAH
        };

        BVH(const std::vector<std::reference_wrapper<Shape> >& shapes, uint32_t maxShapesPerNode,
            BuildMethod buildMethod);

        bool intersect(const Ray3f& ray, float tMin, float tMax, HitInfo& info) const override;
        bool intersect_fast(const Ray3f& ray, float tMin, float tMax, float& t) const override;
        AABB3f aabb() const override;

    private:
        struct BVHShapeInfo {
            BVHShapeInfo() {}
            BVHShapeInfo(uint32_t shapeNumber, const AABB3f& box)
                : shapeNumber(shapeNumber)
                , aabb(box)
            {
                centroid = box.min() * 0.5f + box.max() * 0.5f;
            }

            uint32_t shapeNumber;
            Vector3f centroid;
            AABB3f   aabb;
        };

        struct BVHBuildNode {
            BVHBuildNode() {
                children[0] = nullptr;
                children[1] = nullptr;
            }

            void InitLeaf(uint32_t first, uint32_t num, const AABB3f& b) {
                firstShapeOffset = first;
                numShapes = num;
                aabb = b;
            }

            void InitInterior(uint32_t axis, BVHBuildNode* c0, BVHBuildNode* c1) {
                children[0] = c0;
                children[1] = c1;
                aabb = box_union(c0->aabb, c1->aabb);
                splitAxis = axis;
                numShapes = 0;
            }

            AABB3f        aabb;
            BVHBuildNode* children[2];
            uint32_t      splitAxis;
            uint32_t      firstShapeOffset;
            uint32_t      numShapes;
        };

        struct BVHLinearNode {
            AABB3f aabb;

            union {
                /// Leaf Nodes
                uint32_t firstShapeOffset;
                /// Interior Nodes
                uint32_t secondChildOffset;
            };

            uint8_t numShapes;
            uint8_t axis;
            uint8_t padding[2];
        };

        BVHBuildNode* recursiveBuild(std::vector<BVHShapeInfo>& buildData, uint32_t start, uint32_t end,
                            uint32_t* totalNodes, std::vector<std::reference_wrapper<Shape> >& orderedShapes);
        void flattenBVH(BVHBuildNode* node);

        uint32_t                                    mMaxShapesPerNode;
        BuildMethod                                 mBuildMethod;
        std::vector<std::reference_wrapper<Shape> > mShapes;
        BVHLinearNode*                              mNodes;

    };

    BVH::BVH(const std::vector<std::reference_wrapper<Shape> >& shapes, uint32_t maxShapesPerNode,
             BuildMethod buildMethod)
        : mMaxShapesPerNode(maxShapesPerNode)
        , mBuildMethod(buildMethod)
    {
        for (uint32_t i = 0; i < shapes.size(); ++i) {
            // TODO: Refine incoming shapes as they might be composites.
            //       For now assume incoming list is of non-aggregates.
            mShapes.push_back(shapes[i]);
        }

        if (mShapes.size() == 0) {
            mNodes = nullptr;
            return;
        }

        std::vector<BVHShapeInfo> buildData;
        buildData.reserve(mShapes.size());
        for (uint32_t i = 0; i < mShapes.size(); ++i) {
            AABB3f box =  mShapes[i].get().aabb();
            buildData.push_back(BVHShapeInfo(i, box));
        }

        uint32_t totalNodes = 0;
        std::vector<std::reference_wrapper<Shape> > orderedShapes;
        orderedShapes.reserve(mShapes.size());
        BVHBuildNode* root = recursiveBuild(buildData, 0, mShapes.size(), &totalNodes, orderedShapes);

        mShapes.swap(orderedShapes);

        mNodes = memory::allocAligned<BVHLinearNode>(totalNodes);
        for (uint32_t i = 0; i < totalNodes; ++i) {
            new (&mNodes[i]) BVHLinearNode;
        }

        flattenBVH(root);
    }

    BVH::BVHBuildNode* BVH::recursiveBuild(std::vector<BVHShapeInfo>& buildData, uint32_t start, uint32_t end,
                                           uint32_t* totalNodes, std::vector<std::reference_wrapper<Shape> >& orderedShapes) {
        return nullptr;
    }

    void BVH::flattenBVH(BVHBuildNode* node) {

    }
}
}
