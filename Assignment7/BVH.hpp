//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_BVH_H
#define RAYTRACING_BVH_H

#include <atomic>
#include <vector>
#include <memory>
#include <ctime>
#include "Object.hpp"
#include "Ray.hpp"
#include "Bounds3.hpp"
#include "Intersection.hpp"
#include "Vector.hpp"

#ifdef _DEBUG
#ifndef RECORD_RAY_HIT_PATH
#define RECORD_RAY_HIT_PATH 1
#endif // !RECORD_RAY_HIT_PATH
#endif // _DEBUG

class BVHBuildNode
{
public:
    Bounds3 bounds;
    BVHBuildNode* left;
    BVHBuildNode* right;
    Object* object;
    float area;

#ifdef RECORD_RAY_HIT_PATH
    BVHBuildNode* rayHitNodePathLeft;
    BVHBuildNode* rayHitNodePathRight;
    BVHBuildNode* rayHitNodePathParent;
#endif

public:
    int splitAxis = 0, firstPrimOffset = 0, nPrimitives = 0;
    // BVHBuildNode Public Methods
    BVHBuildNode()
    {
        bounds = Bounds3();
        left = nullptr; right = nullptr;
        object = nullptr;
        area = 0.0f;

        //jingz
#ifdef RECORD_RAY_HIT_PATH
        rayHitNodePathLeft = nullptr;
        rayHitNodePathRight = nullptr;
        rayHitNodePathParent = nullptr;
#endif
    }
};

// BVHAccel Forward Declarations
struct BVHPrimitiveInfo;

// BVHAccel Declarations
inline int leafNodes, totalLeafNodes, totalPrimitives, interiorNodes;
class BVHAccel {

public:
    // BVHAccel Public Types
    enum class SplitMethod { NAIVE, SAH };

    // BVHAccel Public Methods
    BVHAccel(std::vector<Object*> p, int maxPrimsInNode = 1, SplitMethod splitMethod = SplitMethod::NAIVE);
    Bounds3 WorldBound() const;
    ~BVHAccel();

    Intersection Intersect(const Ray &ray) const;
    Intersection getIntersection(BVHBuildNode* node, const Ray& ray)const;
    bool IntersectP(const Ray &ray) const;
    BVHBuildNode* root;

    // BVHAccel Private Methods
    BVHBuildNode* recursiveBuild(std::vector<Object*>objects);

    // BVHAccel Private Data
    const int maxPrimsInNode;
    const SplitMethod splitMethod;
    std::vector<Object*> primitives;

    void getSample(BVHBuildNode* node, float p, Intersection &pos, float &pdf);
    void Sample(Intersection &pos, float &pdf);
};

#endif //RAYTRACING_BVH_H
