//
// Created by goksu on 2/25/20.
//

#include <fstream>
#include "Scene.hpp"
#include "Renderer.hpp"


inline float deg2rad(const float& deg) { return deg * M_PI / 180.0; }

const float EPSILON = 0.00001;

// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.
//简单预设了摄像机在坐标远点，没有Viewport这类投影计算，直接将ndc空间硬换成3D坐标系，真的是服了这类教程作业
void Renderer::Render(const Scene& scene)
{
    std::vector<Vector3f> framebuffer(scene.width * scene.height);

    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = scene.width / (float)scene.height;
    Vector3f eye_pos(278, 273, -800);
    int m = 0;

    // change the spp value to change sample ammount
    int spp = 16;
    std::cout << "SPP: " << spp << "\n";
    for (uint32_t j = 0; j < scene.height; ++j) {
        for (uint32_t i = 0; i < scene.width; ++i) {
            // generate primary ray direction
            float u = 2 * ((float)i + 0.5) / scene.width;
            float v = 2 * ((float)j + 0.5) / scene.height;

            //将屏幕空间类的坐标偏移到和NDC一样的原点
            u = u - 1.0f;
            v = 1.0f - v;//屏幕空间y方向从左上角作为起始点，而NDC左下角为起始点

            //通过屏幕和NDC空间的比例算出原屏幕方向下对应的NDC空间方向
            float x = u * scale * imageAspectRatio;
            float y = v * scale;

            //因为ndc空间和view所在的模型空间重合，所以不需要坐标转换,即可在世界坐标系空间做相交检测
            Vector3f dir_world = normalize(Vector3f(-x, y, 1)); //jingz 我CTM为什么要随意加一些正负号// Don't forget to normalize this direction!
            for (int k = 0; k < spp; k++)
            {
                framebuffer[m] += scene.castRay(Ray(eye_pos, dir_world), 0) / spp;
            }
            m++;
        }
        UpdateProgress(j / (float)scene.height);
    }
    UpdateProgress(1.f);

    // save framebuffer to file
    FILE* fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].x), 0.6f));
        color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].y), 0.6f));
        color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].z), 0.6f));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);    
}
