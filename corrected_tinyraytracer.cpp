#define _USE_MATH_DEFINES     // 定义数学常量，如M_PI（π）
#include <cmath>             // 包含数学函数库
#include <limits>            // 包含限制头文件，用于获取数值类型的最大值、最小值等
#include <iostream>          // 包含输入输出流
#include <fstream>           // 包含文件流
#include <vector>            // 包含向量容器
#include <algorithm>         // 包含算法库，如min、max等函数

#define STB_IMAGE_WRITE_IMPLEMENTATION    // 定义STB图像写入库的实现部分
#include "stb_image_write.h"              // 包含STB图像写入库，用于保存图像
#define STB_IMAGE_IMPLEMENTATION          // 定义STB图像库的实现部分
#include "stb_image.h"                    // 包含STB图像库，用于加载图像

#include "model.h"           // 包含3D模型处理的头文件
#include "geometry.h"        // 包含几何库（向量、矩阵等）

int envmap_width, envmap_height;      // 环境贴图的宽度和高度
std::vector<Vec3f> envmap;            // 存储环境贴图的像素颜色数据（Vec3f表示RGB三个浮点数）
Model duck("../duck.obj");            // 加载鸭子3D模型文件（位于上一级目录）

// 光源结构体，包含位置和强度
struct Light {
    Light(const Vec3f &p, const float i) : position(p), intensity(i) {}  // 构造函数，初始化光源位置和强度
    Vec3f position;    // 光源的三维位置
    float intensity;   // 光源的强度
};

// 材质结构体，定义物体的表面特性
struct Material {
    // 构造函数，初始化所有材质属性
    Material(const float r, const Vec4f &a, const Vec3f &color, const float spec) : refractive_index(r), albedo(a), diffuse_color(color), specular_exponent(spec) {}
    Material() : refractive_index(1), albedo(1,0,0,0), diffuse_color(), specular_exponent() {}  // 默认构造函数，设置默认值
    float refractive_index;   // 折射率，控制光线通过物体时的弯曲程度
    Vec4f albedo;            // 反照率，四个分量分别控制：漫反射、镜面反射、反射率、折射率
    Vec3f diffuse_color;     // 漫反射颜色，物体的基本颜色
    float specular_exponent; // 镜面反射指数，控制高光的锐度
};

// 球体结构体，用于场景中的球形物体
struct Sphere {
    Vec3f center;     // 球心位置
    float radius;     // 球体半径
    Material material;  // 球体材质

    // 构造函数，初始化球心、半径和材质
    Sphere(const Vec3f &c, const float r, const Material &m) : center(c), radius(r), material(m) {}

    // 光线与球体求交函数
    bool ray_intersect(const Vec3f &orig, const Vec3f &dir, float &t0) const {
        Vec3f L = center - orig;      // 从光线原点到球心的向量
        float tca = L*dir;            // L在光线方向上的投影（点积）
        float d2 = L*L - tca*tca;     // 光线到球心的最短距离的平方
        if (d2 > radius*radius) return false;  // 如果距离大于半径，没有交点
        float thc = sqrtf(radius*radius - d2); // 从最近点到交点的距离
        t0       = tca - thc;         // 第一个交点（较近的交点）
        float t1 = tca + thc;         // 第二个交点（较远的交点）
        if (t0 < 0) t0 = t1;          // 如果第一个交点在光线起点后面，使用第二个交点
        if (t0 < 0) return false;     // 如果两个交点都在起点后面，没有有效交点
        return true;                  // 返回有交点
    }
};

// 反射函数：计算反射光线的方向
Vec3f reflect(const Vec3f &I, const Vec3f &N) {
    return I - N*2.f*(I*N);  // 反射公式：I - 2(I·N)N，I是入射光方向，N是法线方向
}

// 折射函数：使用斯涅尔定律计算折射光线的方向
Vec3f refract(const Vec3f &I, const Vec3f &N, const float eta_t, const float eta_i=1.f) { // Snell's law
    float cosi = - std::max(-1.f, std::min(1.f, I*N));  // 计算入射角余弦，并限制在[-1,1]范围内
    if (cosi<0) return refract(I, -N, eta_i, eta_t);    // 如果光线从物体内部射出，交换折射率并翻转法线
    float eta = eta_i / eta_t;                          // 计算相对折射率比
    float k = 1 - eta*eta*(1 - cosi*cosi);              // 计算中间变量k
    // 如果k<0表示发生全反射，否则返回折射光线方向；这里即使全反射也返回一个值，但物理上没有意义
    return k<0 ? Vec3f(1,0,0) : I*eta + N*(eta*cosi - sqrtf(k)); 
}

// 场景求交函数：检测光线与场景中所有物体的交点
bool scene_intersect(const Vec3f &orig, const Vec3f &dir, const std::vector<Sphere> &spheres, Vec3f &hit, Vec3f &N, Material &material) {
    float spheres_dist = std::numeric_limits<float>::max();  // 初始化为最大浮点数
    // 遍历所有球体，查找最近的交点
    for (size_t i=0; i < spheres.size(); i++) {
        float dist_i;  // 当前球体的交点距离
        // 如果光线与当前球体相交，且交点比之前找到的更近
        if (spheres[i].ray_intersect(orig, dir, dist_i) && dist_i < spheres_dist) {
            spheres_dist = dist_i;                  // 更新最近距离
            hit = orig + dir*dist_i;                // 计算交点坐标
            N = (hit - spheres[i].center).normalize(); // 计算交点法线（从球心指向交点的单位向量）
            material = spheres[i].material;           // 获取交点物体的材质
        }
    }

    // 检查与地面（棋盘格）的交点
    float checkerboard_dist = std::numeric_limits<float>::max();
    if (fabs(dir.y)>1e-3)  {  // 如果光线不平行于地面
        float d = -(orig.y+4)/dir.y; // 计算与y=-4平面的交点距离（地面方程为y=-4）
        Vec3f pt = orig + dir*d;    // 计算交点坐标
        // 如果交点在地面上且在有效范围内，且比之前找到的交点更近
        if (d>0 && fabs(pt.x)<10 && pt.z<-10 && pt.z>-30 && d<spheres_dist) {
            checkerboard_dist = d;       // 更新为地面交点距离
            hit = pt;                    // 更新交点坐标
            N = Vec3f(0,1,0);            // 地面法线向上
            // 创建棋盘格图案的颜色
            material.diffuse_color = (int(.5*hit.x+1000) + int(.5*hit.z)) & 1 ? Vec3f(.3, .3, .3) : Vec3f(.3, .2, .1);
        }
    }
    // 如果找到有效交点（距离小于1000），返回true
    return std::min(spheres_dist, checkerboard_dist)<1000;

}

// 光线追踪函数：计算一条光线的颜色
Vec3f cast_ray(const Vec3f& orig, const Vec3f& dir, const std::vector<Sphere>& spheres, const std::vector<Light>& lights, size_t depth = 0) {
    Vec3f point, N;     // 交点坐标和法线
    Material material;  // 交点处的材质

    // 如果递归深度超过4或没有交点，返回环境贴图的颜色
    if (depth>4 || !scene_intersect(orig, dir, spheres, point, N, material)) {
        // 计算光线方向对应的环境贴图坐标
        Vec3f p = dir;                        // 获取光线方向
        float theta = acosf(p.y / p.norm());  // 计算极角（与y轴的夹角）
        float phi = atan2f(p.z, p.x) + M_PI;  // 计算方位角（与x轴的夹角）+π

        // 将球面坐标(theta,phi)转换为环境贴图的像素坐标(x,y)
        int y = theta / (M_PI) * (envmap_height);
        int x = phi / (2 * M_PI) * (envmap_width);

        // 返回环境贴图对应位置的颜色
        return envmap[x + y * envmap_width];
    }

    // 计算反射光线方向（归一化）
    Vec3f reflect_dir = reflect(dir, N).normalize();
    // 计算折射光线方向（归一化）
    Vec3f refract_dir = refract(dir, N, material.refractive_index).normalize();
    
    // 计算反射光线的起点（稍微偏移以避免自相交）
    Vec3f reflect_orig = reflect_dir*N < 0 ? point - N*1e-3 : point + N*1e-3;
    // 计算折射光线的起点（稍微偏移以避免自相交）
    Vec3f refract_orig = refract_dir*N < 0 ? point - N*1e-3 : point + N*1e-3;
    
    // 递归追踪反射光线，获取反射颜色
    Vec3f reflect_color = cast_ray(reflect_orig, reflect_dir, spheres, lights, depth + 1);
    // 递归追踪折射光线，获取折射颜色
    Vec3f refract_color = cast_ray(refract_orig, refract_dir, spheres, lights, depth + 1);

    // 初始化漫反射和镜面反射光照强度
    float diffuse_light_intensity = 0, specular_light_intensity = 0;
    // 遍历所有光源，计算总光照
    for (size_t i=0; i<lights.size(); i++) {
        // 计算从交点到光源的方向（归一化）
        Vec3f light_dir = (lights[i].position - point).normalize();
        // 计算交点到光源的距离
        float light_distance = (lights[i].position - point).norm();

        // 计算阴影射线的起点（稍微偏移以避免自相交）
        Vec3f shadow_orig = light_dir*N < 0 ? point - N*1e-3 : point + N*1e-3;
        Vec3f shadow_pt, shadow_N;
        Material tmpmaterial;
        
        // 检查是否在阴影中：如果阴影射线与场景相交，且交点在光源之前，则在阴影中
        if (scene_intersect(shadow_orig, light_dir, spheres, shadow_pt, shadow_N, tmpmaterial) && (shadow_pt-shadow_orig).norm() < light_distance)
            continue;  // 如果在阴影中，跳过这个光源的贡献

        // 累加漫反射光照强度：光源强度 * max(0, 光源方向·法线)
        diffuse_light_intensity += lights[i].intensity * std::max(0.f, light_dir*N);
        // 累加镜面反射光照强度：(max(0, 视线方向·反射光方向))^指数 * 光源强度
        specular_light_intensity += powf(std::max(0.f, -reflect(-light_dir, N)*dir), material.specular_exponent)*lights[i].intensity;
    }
    
    // 计算最终颜色：漫反射部分 + 镜面反射部分 + 反射颜色部分 + 折射颜色部分
    // 各部分根据材质的albedo向量的四个分量进行加权
    return material.diffuse_color * diffuse_light_intensity * material.albedo[0] +  // 漫反射颜色
           Vec3f(1., 1., 1.)*specular_light_intensity * material.albedo[1] +      // 镜面反射颜色（白色）
           reflect_color*material.albedo[2] +                                    // 反射颜色
           refract_color*material.albedo[3];                                     // 折射颜色
}

// 渲染函数：生成整个图像
void render(const std::vector<Sphere> &spheres, const std::vector<Light> &lights) {
    const int   width    = 1024;         // 图像宽度（像素）
    const int   height   = 768;          // 图像高度（像素）
    const float fov      = M_PI/3.;      // 视场角（60度）
    std::vector<Vec3f> framebuffer(width*height);  // 帧缓冲区，存储每个像素的颜色

    #pragma omp parallel for              // OpenMP并行处理指令，多线程加速渲染
    for (int j = 0; j<height; j++) {      // 遍历每一行像素
        for (int i = 0; i<width; i++) {   // 遍历每一列像素
            // 计算从视点到像素的方向向量
            float dir_x =  (i + 0.5) -  width/2.;              // x方向分量
            float dir_y = -(j + 0.5) + height/2.;              // y方向分量（负号使图像上下翻转）
            float dir_z = -height/(2.*tan(fov/2.));            // z方向分量（确定视场角）
            
            // 对每个像素投射光线并计算颜色
            framebuffer[i+j*width] = cast_ray(Vec3f(0,0,0), Vec3f(dir_x, dir_y, dir_z).normalize(), spheres, lights);
        }
    }

    // 创建输出图像的像素数据数组（每个像素3字节，RGB）
    std::vector<unsigned char> pixmap(width*height*3);
    // 将framebuffer中的浮点颜色值转换为0-255的整数
    for (size_t i = 0; i < height*width; ++i) {
        Vec3f &c = framebuffer[i];                            // 获取当前像素颜色
        float max = std::max(c[0], std::max(c[1], c[2]));     // 找出RGB中的最大值
        if (max>1) c = c*(1./max);                            // 如果最大值>1，进行颜色修正（缩放）
        // 将浮点颜色值[0,1]转换为字节值[0,255]并存入pixmap
        for (size_t j = 0; j<3; j++) {
            pixmap[i*3+j] = (unsigned char)(255 * std::max(0.f, std::min(1.f, framebuffer[i][j])));
        }
    }
    // 将图像保存为JPEG文件，质量为100
    stbi_write_jpg("out_corrected.jpg", width, height, 3, pixmap.data(), 100);
}

// 主函数
int main() {
    int n = -1;  // 用于存储图像通道数
    // 加载环境贴图（从上一级目录的nfr.jpg文件）
    unsigned char *pixmap = stbi_load("../nfr.jpg", &envmap_width, &envmap_height, &n, 0);
    // 检查加载是否成功且图像是否为3通道（RGB）
    if (!pixmap || 3!=n) {
        std::cerr << "Error: can not load the environment map" << std::endl;
        return -1;  // 失败则退出程序
    }
    // 初始化环境贴图数组
    envmap = std::vector<Vec3f>(envmap_width*envmap_height);
    // 将加载的像素数据转换为浮点颜色值
    for (int j = envmap_height-1; j>=0 ; j--) {       // 从底行到顶行遍历（图像坐标系原点在左上角）
        for (int i = 0; i<envmap_width; i++) {        // 从左到右遍历每一列
            // 读取RGB值并除以255转换为[0,1]范围
            envmap[i+j*envmap_width] = Vec3f(
                pixmap[(i+j*envmap_width)*3+0],      // 红色分量
                pixmap[(i+j*envmap_width)*3+1],      // 绿色分量
                pixmap[(i+j*envmap_width)*3+2]       // 蓝色分量
            )*(1/255.);
        }
    }
    stbi_image_free(pixmap);  // 释放原始图像数据内存

    // 定义不同的材质
    // 参数：折射率, albedo(漫反射权重,镜面反射权重,反射权重,折射权重), 颜色, 镜面反射指数
    Material      ivory(1.0, Vec4f(0.6,  0.3, 0.1, 0.0), Vec3f(0.4, 0.4, 0.3),   50.);    // 象牙材质
    Material      glass(1.5, Vec4f(0.0,  0.5, 0.1, 0.8), Vec3f(0.6, 0.7, 0.8),  125.);    // 玻璃材质
    Material red_rubber(1.0, Vec4f(0.9,  0.1, 0.0, 0.0), Vec3f(0.3, 0.1, 0.1),   10.);    // 红色橡胶材质
    Material     mirror(1.0, Vec4f(0.0, 10.0, 0.8, 0.0), Vec3f(1.0, 1.0, 1.0), 1425.);    // 镜面材质

    // 创建场景中的球体数组
    std::vector<Sphere> spheres;
    // 添加球体：参数为球心坐标、半径、材质
    spheres.push_back(Sphere(Vec3f(-3,    0,   -16), 2,      ivory));      // 象牙球
    spheres.push_back(Sphere(Vec3f(-1.0, -1.5, -12), 2,      glass));      // 玻璃球
    spheres.push_back(Sphere(Vec3f( 1.5, -0.5, -18), 3, red_rubber));      // 红色橡胶球
    spheres.push_back(Sphere(Vec3f( 7,    5,   -18), 4,     mirror));      // 镜面球

    // 创建光源数组
    std::vector<Light>  lights;
    // 添加光源：参数为位置、强度
    lights.push_back(Light(Vec3f(-20, 20,  20), 1.5));  // 左上前方光源
    lights.push_back(Light(Vec3f( 30, 50, -25), 1.8));  // 右上后方光源
    lights.push_back(Light(Vec3f( 30, 20,  30), 1.7));  // 右上前方光源

    // 调用渲染函数，生成图像
    render(spheres, lights);

    return 0;  // 程序正常结束
}

