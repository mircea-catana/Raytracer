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
        ~BVH();

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

        struct CompareToMid {
            CompareToMid(uint32_t dimension, float mid) : dimension(dimension), mid(mid) {}

            bool operator() (const BVHShapeInfo& shape) const {
                return shape.centroid[dimension] < mid;
            }

            uint32_t dimension;
            float    mid;
        };

        struct ComparePoints {
            ComparePoints(uint32_t dimension) : dimension(dimension) {}

            bool operator() (const BVHShapeInfo& shape1, const BVHShapeInfo& shape2) const {
                return shape1.centroid[dimension] < shape2.centroid[dimension];
            }

            uint32_t dimension;
        };

        struct CompareToBucket {
            CompareToBucket(uint32_t split, uint32_t num, uint32_t dimension, const AABB3f& box)
                : splitBucket(split)
                , numBuckets(num)
                , dimension(dimension)
                , centroidBounds(box)
            {
            }

            bool operator() (const BVHShapeInfo& shape) const {
                uint32_t b = numBuckets * ((shape.centroid[dimension] - centroidBounds.min()[dimension]) /
                                           (centroidBounds.max()[dimension] - centroidBounds.min()[dimension]));
                if (b == numBuckets) {
                    b = numBuckets - 1;
                }

                return b <= splitBucket;
            }

            uint32_t splitBucket;
            uint32_t numBuckets;
            uint32_t dimension;
            AABB3f   centroidBounds;
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
        uint32_t flattenBVH(BVHBuildNode* node, uint32_t* offset);

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

        uint32_t offset = 0;
        flattenBVH(root, &offset);
    }

    BVH::~BVH() {
        memory::freeAligned(mNodes);
    }

    BVH::BVHBuildNode* BVH::recursiveBuild(std::vector<BVHShapeInfo>& buildData, uint32_t start, uint32_t end,
                                           uint32_t* totalNodes, std::vector<std::reference_wrapper<Shape> >& orderedShapes) {
        ++(*totalNodes);

        BVHBuildNode* node = new BVHBuildNode();

        AABB3f bbox;
        for (uint32_t i = start; i < end; ++i) {
            bbox = box_union(bbox, buildData[i].aabb);
        }

        uint32_t numShapes = end - start;
        if (numShapes == 1) {
            uint32_t firstShapeOffset = orderedShapes.size();

            for (uint32_t i = start; i < end; ++i) {
                uint32_t shapeNumber = buildData[i].shapeNumber;
                orderedShapes.push_back(mShapes[shapeNumber]);
            }

            node->InitLeaf(firstShapeOffset, numShapes, bbox);

        } else {

            AABB3f centroidBounds;
            for (uint32_t i = start; i < end; ++i) {
                centroidBounds = box_union(centroidBounds, buildData[i].centroid);
            }

            uint32_t dimension = centroidBounds.maximumExtent();
            uint32_t mid       = (start + end) / 2u;

            if (centroidBounds.min()[dimension] == centroidBounds.max()[dimension]) {
                if (numShapes <= mMaxShapesPerNode) {
                    uint32_t firstShapeOffset = orderedShapes.size();

                    for (uint32_t i = start; i < end; ++i) {
                        uint32_t shapeNumber = buildData[i].shapeNumber;
                        orderedShapes.push_back(mShapes[shapeNumber]);
                    }

                    node->InitLeaf(firstShapeOffset, numShapes, bbox);
                    return node;

                } else {
                    node->InitInterior(dimension,
                                       recursiveBuild(buildData, start, mid, totalNodes, orderedShapes),
                                       recursiveBuild(buildData, mid,   end, totalNodes, orderedShapes));
                    return node;
                }
            }

            /// Partition based on split method
            switch(mBuildMethod) {

            case eMIDDLE: {
                float tMid = (centroidBounds.min()[dimension] + centroidBounds.max()[dimension]) * 0.5f;
                BVHShapeInfo* midPtr = std::partition(&buildData[start],
                                                      &buildData[end - 1] + 1,
                                                      CompareToMid(dimension, tMid));
                mid = midPtr - &buildData[0];
                if (mid != start && mid != end) {
                    break;
                }

                /// In the case there are a lot of primitives with large overlapping bboxes
                /// we want to fall through and use eEQUAL_COUNT
            }

            case eEQUAL_COUNT: {
                mid = (start + end) / 2u;
                std::nth_element(&buildData[start],
                                 &buildData[mid],
                                 &buildData[end - 1] + 1,
                                 ComparePoints(dimension));
                break;
            }

            default:
            case eSAH: {
                if (numShapes <= 4) {
                    mid = (start + end) / 2u;
                    std::nth_element(&buildData[start],
                                     &buildData[mid],
                                     &buildData[end - 1] + 1,
                                     ComparePoints(dimension));
                    break;
                }

                const uint32_t numBuckets = 12;

                struct BucketInfo {
                    BucketInfo() : count(0u) {}

                    uint32_t count;
                    AABB3f   bounds;
                };

                BucketInfo buckets[numBuckets];

                for (uint32_t i = start; i < end; ++i) {
                    uint32_t b = numBuckets * ((buildData[i].centroid[dimension] - centroidBounds.min()[dimension]) /
                                               (centroidBounds.max()[dimension] - centroidBounds.min()[dimension]));
                    if (b == numBuckets) {
                        b = numBuckets - 1;
                    }

                    buckets[b].count++;
                    buckets[b].bounds = box_union(buckets[b].bounds, buildData[i].aabb);
                }

                float cost[numBuckets - 1];
                for (uint32_t i = 0; i < numBuckets - 1; ++i) {
                    AABB3f   b0, b1;
                    uint32_t c0, c1;

                    for (uint32_t j = 0; j <= i; ++j) {
                        b0  = box_union(b0, buckets[j].bounds);
                        c0 += buckets[j].count;
                    }

                    for (uint32_t j = i + 1; j < numBuckets; ++j) {
                        b1  = box_union(b1, buckets[j].bounds);
                        c1 += buckets[j].count;
                    }

                    cost[i] = 0.125f + (c0 * b0.surfaceArea() + c1 * b1.surfaceArea()) / bbox.surfaceArea();
                }

                float    minCost      = cost[0];
                uint32_t minCostSplit = 0;
                for (uint32_t i = 1; i < numBuckets - 1; ++i) {
                    if (cost[i] < minCost) {
                        minCost      = cost[i];
                        minCostSplit = i;
                    }
                }

                if (numShapes > mMaxShapesPerNode || minCost < numShapes) {
                    BVHShapeInfo* midPtr = std::partition(&buildData[start],
                                                          &buildData[end - 1] + 1,
                                                          CompareToBucket(minCostSplit, numBuckets, dimension, centroidBounds));
                    mid = midPtr - &buildData[0];
                } else {
                    uint32_t firstShapeOffset = orderedShapes.size();

                    for (uint32_t i = start; i < end; ++i) {
                        uint32_t shapeNumber = buildData[i].shapeNumber;
                        orderedShapes.push_back(mShapes[shapeNumber]);
                    }

                    node->InitLeaf(firstShapeOffset, numShapes, bbox);
                    return node;
                }

                break;
            }
            }

            node->InitInterior(dimension,
                               recursiveBuild(buildData, start, mid, totalNodes, orderedShapes),
                               recursiveBuild(buildData, mid,   end, totalNodes, orderedShapes));
        }

        return node;
    }

    uint32_t BVH::flattenBVH(BVHBuildNode* node, uint32_t* offset) {
        BVHLinearNode* linearNode = &mNodes[*offset];
        linearNode->aabb = node->aabb;

        uint32_t myOffset = (*offset)++;

        if (node->numShapes > 0) {
            linearNode->firstShapeOffset = node->firstShapeOffset;
            linearNode->numShapes        = node->numShapes;

        } else {
            linearNode->axis      = node->splitAxis;
            linearNode->numShapes = 0;

            flattenBVH(node->children[0], offset);
            linearNode->secondChildOffset = flattenBVH(node->children[1], offset);
        }

        return myOffset;
    }

    bool BVH::intersect(const Ray3f& ray, float tMin, float tMax, HitInfo& info) const {
        return false;
    }

    bool BVH::intersect_fast(const Ray3f& ray, float tMin, float tMax, float& t) const {
        return false;
    }

    AABB3f BVH::aabb() const {
        return math::AABB3f(Vector3f(), Vector3f());
    }
}
}
