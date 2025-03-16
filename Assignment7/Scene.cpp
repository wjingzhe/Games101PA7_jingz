//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"

void Scene::buildBVH()
{
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::getIntersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection & pos, float & pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}



void Scene::JingzSampleLight(Intersection& result_pos, float& result_pdf) const
{
    float p = 1.0f * lights_emit_area_sum;//由总面积生成阈值作为有效门槛
    //float p = get_random_float() * lights_emit_area_sum;//由总面积生成阈值作为有效门槛
    float cur_emit_area_sum = 0.0f;
    for (uint32_t k = 0; k < objects.size(); ++k)
    {
        if (objects[k]->hasEmit())
        {
            //每次累计一定面积超过随机阈值概率才采样生成采样点和概率密度?
            cur_emit_area_sum += objects[k]->getArea();
            if (cur_emit_area_sum >= p)
            {
                objects[k]->Sample(result_pos, result_pdf);
                break;
            }
        }
    }
}

void Scene::calculateLightEmitArea()//扫描场景内所有物体，累计有效发光区域面积
{
    for (uint32_t k = 0; k < objects.size(); ++k)
    {
        if (objects[k]->hasEmit())
        {
            lights_emit_area_sum += objects[k]->getArea();
        }
    }
}

bool Scene::trace(
    const Ray &ray,
    const std::vector<Object *> &objects,
    float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k)
    {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear)
        {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }

    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    Intersection intersection = getIntersect(ray);
    if (!intersection.happened)
        return Vector3f();

    if (intersection.pMaterial->hasEmission())
    {
        return intersection.pMaterial->getEmission();
    }

    Vector3f wo = ray.direction;//镜头射线的入射方向,但我们计算利用光路可逆性来计算，属于wo
    Vector3f L_direct_factor(0.0f, 0.0f, 0.0f);

    Intersection inter_L_direct;
    float pdf_light = 0.0f;
    // 在场景的所有光源上按面积 uniform 地 sampley一个，并计算该sample地概率密度
    JingzSampleLight(inter_L_direct, pdf_light);
    Vector3f lightPos = inter_L_direct.coords;//把光源限定在一个标准几何面元的几何表中心处

    Vector3f curPos = intersection.coords;//场景内要与光线求教的位置
    Vector3f tempToLight = (lightPos - curPos);
    Vector3f wi = tempToLight.normalized();

    Ray curPos_2_light_ray(curPos, wi);
    Intersection curPos_2_light_inter = getIntersect(curPos_2_light_ray);
    //return curPos_2_light_inter.normal;

    if (curPos_2_light_inter.distance - (lightPos - curPos).norm() > -0.005f)//与发光面元中心距离属于合理误差内，计算直接光照
    {
        //L_direct_factor = Vector3f(0.1f, 0.0f, 0.0f);
        Vector3f f_r = intersection.pMaterial->eval(wo, wi, intersection.normal);
        float distance2_inv = 1.0f / dotProduct(tempToLight, tempToLight);//距离衰减部分系数
        L_direct_factor = inter_L_direct.emit * f_r * dotProduct(wi, intersection.normal) * dotProduct(-wi, inter_L_direct.normal) * distance2_inv / pdf_light;
    }

    //间接光，在漫反射物体上计算间接光照部分，假设所有非漫射物体都是镜面反射，可以考虑增加漫反射弹射次层数
    Vector3f L_indir_factor(0.0f, 0.0f, 0.0f);
    {
        if (get_random_float() > RussianRoulette)//赌输了就没有间接光照衍生的射线
        {
            return L_direct_factor;
        }

        // 按照该材质的性质，给定入射方向和法向量，用某种分布采样一个出射方向
        Vector3f wo2 = (intersection.pMaterial->sample(wo, intersection.normal)).normalized();

        Ray ray_indir(curPos, wo2);
        Intersection inter_L_indirect = getIntersect(ray_indir);
        if (inter_L_indirect.happened && !inter_L_indirect.pMaterial->hasEmission())//非直接光源
        {
            // 给定一对入射、出射方向和法向量，计算sample方法得到该出射方向的概率密度
            float pdf = inter_L_indirect.pMaterial->pdf(wo, wo2, intersection.normal);
            L_indir_factor = castRay(ray_indir, depth + 1)
                * (inter_L_indirect.pMaterial->eval(wo, wo2, intersection.normal) * dotProduct(wo2, intersection.normal) / pdf / RussianRoulette);
        }
    }

    return L_direct_factor + L_indir_factor;
}