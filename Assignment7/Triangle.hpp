#pragma once

#include "BVH.hpp"
#include "Intersection.hpp"
#include "Material.hpp"
#include "OBJ_Loader.hpp"
#include "Object.hpp"
#include "Triangle.hpp"
#include <cassert>
#include <array>

bool rayTriangleIntersect(const Vector3f& v0, const Vector3f& v1,
                          const Vector3f& v2, const Vector3f& orig,
                          const Vector3f& dir, float& tnear, float& u, float& v)
{
    Vector3f edge1 = v1 - v0;
    Vector3f edge2 = v2 - v0;
    Vector3f pvec = crossProduct(dir, edge2);
    float det = dotProduct(edge1, pvec);
    if (det == 0 || det < 0)
        return false;

    Vector3f tvec = orig - v0;
    u = dotProduct(tvec, pvec);
    if (u < 0 || u > det)
        return false;

    Vector3f qvec = crossProduct(tvec, edge1);
    v = dotProduct(dir, qvec);
    if (v < 0 || u + v > det)
        return false;

    float invDet = 1 / det;

    tnear = dotProduct(edge2, qvec) * invDet;
    u *= invDet;
    v *= invDet;

    return true;
}


#define TEMP_EPSILON 1e-6
bool rayTriangleIntersect_MollerTrumbore(const Vector3f& v0, const Vector3f& v1, const Vector3f& v2, 
    const Vector3f& orig,const Vector3f& dir, float& tnear, float& u, float& v)
{
    //{
    //    Vector3f edge1 = v1 - v0;
    //    Vector3f edge2 = v2 - v0;

    //    Vector3f S1 = crossProduct(dir, edge2);
    //    float E1S1 = dotProduct(edge1, S1);
    //    if (abs(E1S1) <= TEMP_EPSILON)//克莱姆法则：为0，矩阵方程无解
    //    {
    //        return false;
    //    }
    //    float invDet = 1 / E1S1;

    //    Vector3f tvec = orig - v0;//S
    //    Vector3f qvec = crossProduct(tvec, edge1);//S2
    //    tnear = dotProduct(edge2, qvec) * invDet;

    //    u = dotProduct(tvec, S1);
    //    if (u < 0 || u > E1S1)
    //        return false;

    //    v = dotProduct(dir, qvec);
    //    if (v < 0 || u + v > E1S1)
    //        return false;

    //    u *= invDet;
    //    v *= invDet;

    //    return true;
    //}

    // TODO: Implement this function that tests whether the triangle
    // that's specified bt v0, v1 and v2 intersects with the ray (whose
    // origin is *orig* and direction is *dir*)
    // Also don't forget to update tnear, u and v.

    Vector3f E1 = v1 - v0;
    Vector3f E2 = v2 - v0;

    Vector3f S1 = crossProduct(dir, E2);//p,射线与边2方向所在平面的法线___当作某种切线用？
    float E1S1 = dotProduct(E1,S1);//det
    //射线与平面平行
    if (abs(E1S1) <= TEMP_EPSILON)//克莱姆法则：为0，矩阵方程无解
    {
        return false;
    }
    float inv_E1S1 = 1.0f / E1S1;//inv_det

    Vector3f S = orig - v0;//其他算法中的大Tvec
    Vector3f S2 = crossProduct(S, E1);//Q=Tvec x E1
    float t = dotProduct(E2,S2) * inv_E1S1;
    if ( t < 0.0f)
    {
        return false;
    }

    float S_S1 = dotProduct(S,S1);
    if (S_S1 < 0.0f)
        return false;

    float dir_S2 = dotProduct(dir,S2);
    if (dir_S2 < 0.0f)
        return false;

    u = S_S1 * inv_E1S1;
    v = dir_S2 * inv_E1S1;
    if (1.0f > u && 1.0f > v && (1 - u - v) >= TEMP_EPSILON)
    {
        tnear = t;
        return true;
    }

    return false;
}

class Triangle : public Object
{
public:
    Vector3f v0, v1, v2; // vertices A, B ,C , counter-clockwise order
    Vector3f e1, e2;     // 2 edges v1-v0, v2-v0;
    Vector3f t0, t1, t2; // texture coords
    Vector3f normal;
    float area;
    Material *pMaterial;

    Triangle(Vector3f _v0, Vector3f _v1, Vector3f _v2, Material *_m = nullptr)
        : v0(_v0), v1(_v1), v2(_v2), pMaterial(_m)
    {
        e1 = v1 - v0;
        e2 = v2 - v0;
        normal = normalize(crossProduct(e1, e2));
        area = crossProduct(e1, e2).norm()*0.5f;
    }

    bool intersect(const Ray& ray) override;
    bool intersect(const Ray& ray, float& tnear,
                   uint32_t& index) const override;
    Intersection getIntersection(Ray ray) override;
    void getSurfaceProperties(const Vector3f& P, const Vector3f& I,
                              const uint32_t& index, const Vector2f& uv,
                              Vector3f& N, Vector2f& st) const override
    {
        N = normal;
        //        throw std::runtime_error("triangle::getSurfaceProperties not
        //        implemented.");
    }
    Vector3f evalDiffuseColor(const Vector2f&) const override;
    Bounds3 getBounds() override;
    void Sample(Intersection &pos, float &pdf)
    {
        float x = std::sqrt(get_random_float()), y = get_random_float();
        pos.coords = v0 * (1.0f - x) + v1 * (x * (1.0f - y)) + v2 * (x * y);
        pos.normal = this->normal;
        pdf = 1.0f / area;
    }
    float getArea()
    {
        return area;
    }
    bool hasEmit()
    {
        return pMaterial->hasEmission();
    }
};

class MeshTriangle : public Object
{
public:
    MeshTriangle(const std::string& filename, Material *mt = new Material())
    {
        objl::Loader loader;
        loader.LoadFile(filename);
        area = 0;
        pMaterial = mt;
        assert(loader.LoadedMeshes.size() == 1);
        auto mesh = loader.LoadedMeshes[0];

        Vector3f min_vert = Vector3f{std::numeric_limits<float>::infinity(),
                                     std::numeric_limits<float>::infinity(),
                                     std::numeric_limits<float>::infinity()};
        Vector3f max_vert = Vector3f{-std::numeric_limits<float>::infinity(),
                                     -std::numeric_limits<float>::infinity(),
                                     -std::numeric_limits<float>::infinity()};
        for (int i = 0; i < mesh.Vertices.size(); i += 3)
        {
            std::array<Vector3f, 3> face_vertices;

            for (int j = 0; j < 3; j++)
            {
                auto vert = Vector3f(mesh.Vertices[i + j].Position.X,
                                     mesh.Vertices[i + j].Position.Y,
                                     mesh.Vertices[i + j].Position.Z);
                face_vertices[j] = vert;

                min_vert = Vector3f(std::min(min_vert.x, vert.x),
                                    std::min(min_vert.y, vert.y),
                                    std::min(min_vert.z, vert.z));
                max_vert = Vector3f(std::max(max_vert.x, vert.x),
                                    std::max(max_vert.y, vert.y),
                                    std::max(max_vert.z, vert.z));
            }

            triangles.emplace_back(face_vertices[0], face_vertices[1],
                                   face_vertices[2], mt);
        }

        bounding_box = Bounds3(min_vert, max_vert);

        std::vector<Object *> ptrs;
        for (auto &tri : triangles)
        {
            ptrs.push_back(&tri);
            area += tri.area;
        }
        bvh = new BVHAccel(ptrs);
    }

    bool intersect(const Ray& ray) { return true; }

    bool intersect(const Ray& ray, float& tnear, uint32_t& index) const
    {
        bool intersect = false;
        for (uint32_t k = 0; k < numTriangles; ++k)
        {
            const Vector3f &v0 = vertices[vertexIndex[k * 3]];
            const Vector3f &v1 = vertices[vertexIndex[k * 3 + 1]];
            const Vector3f &v2 = vertices[vertexIndex[k * 3 + 2]];
            float t, u, v;
            //可能和模型的多个三角形相交，只取最近一个 rayTriangleIntersect_MollerTrumbore
            if (rayTriangleIntersect_MollerTrumbore(v0, v1, v2, ray.origin, ray.direction, t,u, v) &&
                t < tnear)
            {//jingz
                tnear = t;
                index = k;
                intersect |= true;
            }
        }

        return intersect;
    }

    Bounds3 getBounds() { return bounding_box; }

    void getSurfaceProperties(const Vector3f& P, const Vector3f& I,
                              const uint32_t& index, const Vector2f& uv,
                              Vector3f& N, Vector2f& st) const
    {
        const Vector3f& v0 = vertices[vertexIndex[index * 3]];
        const Vector3f& v1 = vertices[vertexIndex[index * 3 + 1]];
        const Vector3f& v2 = vertices[vertexIndex[index * 3 + 2]];
        Vector3f e0 = normalize(v1 - v0);
        Vector3f e1 = normalize(v2 - v1);
        N = normalize(crossProduct(e0, e1));
        const Vector2f& st0 = stCoordinates[vertexIndex[index * 3]];
        const Vector2f& st1 = stCoordinates[vertexIndex[index * 3 + 1]];
        const Vector2f& st2 = stCoordinates[vertexIndex[index * 3 + 2]];
        st = st0 * (1 - uv.x - uv.y) + st1 * uv.x + st2 * uv.y;
    }

    Vector3f evalDiffuseColor(const Vector2f& st) const
    {
        float scale = 5;
        float pattern =
            (fmodf(st.x * scale, 1) > 0.5) ^ (fmodf(st.y * scale, 1) > 0.5);
        return lerp(Vector3f(0.815, 0.235, 0.031),
                    Vector3f(0.937, 0.937, 0.231), pattern);
    }

    Intersection getIntersection(Ray ray)
    {
        Intersection intersec;

        if (bvh)
        {
            intersec = bvh->Intersect(ray);
        }

        return intersec;
    }

    void Sample(Intersection &pos, float &pdf)
    {
        bvh->Sample(pos, pdf);
        pos.emit = pMaterial->getEmission();
    }
    float getArea()
    {
        return area;
    }
    bool hasEmit()
    {
        return pMaterial->hasEmission();
    }

public:
    Bounds3 bounding_box;
    std::unique_ptr<Vector3f[]> vertices;
    uint32_t numTriangles;
    std::unique_ptr<uint32_t[]> vertexIndex;
    std::unique_ptr<Vector2f[]> stCoordinates;

    std::vector<Triangle> triangles;

    BVHAccel* bvh;
    float area;

    Material *pMaterial;
};

inline bool Triangle::intersect(const Ray& ray) { return true; }
inline bool Triangle::intersect(const Ray& ray, float& tnear,
                                uint32_t& index) const
{
    return false;
}

inline Bounds3 Triangle::getBounds() { return Union(Bounds3(v0, v1), v2); }

inline Intersection Triangle::getIntersection(Ray ray)
{
    Intersection inter;
    inter.happened = false;

    float tempT = 0.0f;
    float u = 0.0f, v = 0.0f;
    inter.happened = rayTriangleIntersect_MollerTrumbore(v0, v1, v2, ray.origin, ray.direction, tempT, u, v);

    if (!inter.happened|| tempT<0.0f)
    {
        return inter;
    }

    //jingz 有效交点
    inter.distance = tempT;
    inter.happened = true;
    inter.pMaterial = pMaterial;
    inter.obj = this;
    inter.normal = normal;
    inter.coords = ray(tempT);
    return inter;

    //if (dotProduct(ray.direction, normal) > 0)
    //    return inter;
    //double u, v, t_tmp = 0;
    //Vector3f pvec = crossProduct(ray.direction, e2);
    //double det = dotProduct(e1, pvec);
    //if (fabs(det) < EPSILON)
    //    return inter;

    //double det_inv = 1. / det;
    //Vector3f tvec = ray.origin - v0;
    //u = dotProduct(tvec, pvec) * det_inv;
    //if (u < 0 || u > 1)
    //    return inter;
    //Vector3f qvec = crossProduct(tvec, e1);
    //v = dotProduct(ray.direction, qvec) * det_inv;
    //if (v < 0 || u + v > 1)
    //    return inter;
    //t_tmp = dotProduct(e2, qvec) * det_inv;

    ////TODO find ray triangle intersection
    //    if (t_tmp < 0)
    //    return inter;
    ////jingz 有效交点
    //inter.distance = t_tmp;
    //inter.happened = true;
    //inter.pMaterial = pMaterial;
    //inter.obj = this;
    //inter.normal = normal;
    //inter.coords = ray(t_tmp);

    //return inter;
}

inline Vector3f Triangle::evalDiffuseColor(const Vector2f&) const
{
    return Vector3f(0.5, 0.5, 0.5);
}
