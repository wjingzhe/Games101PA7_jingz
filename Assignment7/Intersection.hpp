//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_INTERSECTION_H
#define RAYTRACING_INTERSECTION_H
#include "Vector.hpp"
#include "Material.hpp"
class Object;
class Sphere;

#ifdef _DEBUG
#ifndef RECORD_RAY_HIT_PATH
#define RECORD_RAY_HIT_PATH 1
#endif // !RECORD_RAY_HIT_PATH
#endif // _DEBUG

#ifdef RECORD_RAY_HIT_PATH
class BVHBuildNode;
#endif

struct Intersection
{
    Intersection(){
        happened=false;
        coords=Vector3f();
        normal=Vector3f();
        emit=Vector3f();
        distance= std::numeric_limits<double>::max();
        obj =nullptr;
        pMaterial=nullptr;

#ifdef RECORD_RAY_HIT_PATH
        curBVHNode = nullptr;;
#endif
    }
    bool happened;
    Vector3f coords;
    Vector3f tcoords;
    Vector3f normal;
    Vector3f emit;
    double distance;
    Object* obj;
    Material* pMaterial;

#ifdef RECORD_RAY_HIT_PATH
    BVHBuildNode* curBVHNode;
#endif
};
#endif //RAYTRACING_INTERSECTION_H
