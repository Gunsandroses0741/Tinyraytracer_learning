#include <iostream>     // 包含输入输出流
#include <cassert>      // 包含断言功能
#include <fstream>      // 包含文件流
#include <sstream>      // 包含字符串流
#include "model.h"      // 包含模型头文件

// 填充顶点和面数组，假设.obj文件的"f "条目没有斜杠
Model::Model(const char *filename) : verts(), faces() {  // 构造函数，初始化顶点和面数组
    std::ifstream in;                               // 创建输入文件流
    in.open (filename, std::ifstream::in);          // 打开指定的文件
    if (in.fail()) {                                // 如果文件打开失败
        std::cerr << "Failed to open " << filename << std::endl;  // 输出错误信息
        return;                                     // 返回，构造失败
    }
    std::string line;                               // 存储每一行的字符串
    while (!in.eof()) {                             // 当未到达文件末尾时循环
        std::getline(in, line);                     // 读取一行到line
        std::istringstream iss(line.c_str());       // 创建字符串流用于解析
        char trash;                                 // 用于存储无用字符
        if (!line.compare(0, 2, "v ")) {            // 如果行以"v "开头(顶点数据)
            iss >> trash;                           // 跳过"v"字符
            Vec3f v;                                // 创建3D向量存储顶点坐标
            for (int i=0;i<3;i++) iss >> v[i];      // 读取xyz三个坐标
            verts.push_back(v);                     // 将顶点添加到顶点数组
        } else if (!line.compare(0, 2, "f ")) {     // 如果行以"f "开头(面数据)
            Vec3i f;                                // 创建整型向量存储面的顶点索引
            int idx, cnt=0;                         // idx存储顶点索引，cnt计数
            iss >> trash;                           // 跳过"f"字符
            while (iss >> idx) {                    // 读取顶点索引
                idx--; // in wavefront obj all indices start at 1, not zero  // obj文件索引从1开始，转为从0开始
                f[cnt++] = idx;                     // 存储顶点索引
            }
            if (3==cnt) faces.push_back(f);         // 如果是三角形面(3个顶点)，添加到面数组
        }
    }
    // 输出模型信息：顶点数量和面数量
    std::cerr << "# v# " << verts.size() << " f# "  << faces.size() << std::endl;

    Vec3f min, max;                                 // 创建变量存储包围盒
    get_bbox(min, max);                             // 计算模型的包围盒
}

// Moller and Trumbore算法实现光线与三角形相交检测
bool Model::ray_triangle_intersect(const int &fi, const Vec3f &orig, const Vec3f &dir, float &tnear) {
    Vec3f edge1 = point(vert(fi,1)) - point(vert(fi,0));  // 计算三角形的第一条边
    Vec3f edge2 = point(vert(fi,2)) - point(vert(fi,0));  // 计算三角形的第二条边
    Vec3f pvec = cross(dir, edge2);                       // 计算叉积 P = D × E2
    float det = edge1*pvec;                               // 计算行列式 det = E1·P
    if (det<1e-5) return false;                           // 如果行列式接近零，光线与三角形平行，无相交

    Vec3f tvec = orig - point(vert(fi,0));                // 计算原点到三角形第一个顶点的向量
    float u = tvec*pvec;                                  // 计算重心坐标u
    if (u < 0 || u > det) return false;                   // 如果u超出范围，光线未击中三角形

    Vec3f qvec = cross(tvec, edge1);                      // 计算叉积 Q = T × E1
    float v = dir*qvec;                                   // 计算重心坐标v
    if (v < 0 || u + v > det) return false;               // 如果v超出范围或u+v>1，光线未击中三角形

    tnear = edge2*qvec * (1./det);                        // 计算交点距离t
    return tnear>1e-5;                                    // 如果t大于一个小阈值，返回有效碰撞
}

// 返回顶点数量
int Model::nverts() const {
    return (int)verts.size();                             // 返回顶点数组大小
}

// 返回面数量
int Model::nfaces() const {
    return (int)faces.size();                             // 返回面数组大小
}

// 计算模型的包围盒
void Model::get_bbox(Vec3f &min, Vec3f &max) {
    min = max = verts[0];                                 // 初始化min和max为第一个顶点
    for (int i=1; i<(int)verts.size(); ++i) {             // 遍历所有其他顶点
        for (int j=0; j<3; j++) {                         // 对每个坐标分量(x,y,z)
            min[j] = std::min(min[j], verts[i][j]);       // 更新最小值
            max[j] = std::max(max[j], verts[i][j]);       // 更新最大值
        }
    }
    // 输出包围盒信息
    std::cerr << "bbox: [" << min << " : " << max << "]" << std::endl;
}

// 获取指定索引的顶点(常量版本)
const Vec3f &Model::point(int i) const {
    assert(i>=0 && i<nverts());                           // 断言索引在有效范围内
    return verts[i];                                      // 返回顶点
}

// 获取指定索引的顶点(非常量版本)
Vec3f &Model::point(int i) {
    assert(i>=0 && i<nverts());                           // 断言索引在有效范围内
    return verts[i];                                      // 返回顶点引用
}

// 获取指定面的指定顶点索引
int Model::vert(int fi, int li) const {
    assert(fi>=0 && fi<nfaces() && li>=0 && li<3);        // 断言索引在有效范围内
    return faces[fi][li];                                 // 返回顶点索引
}

// 重载输出运算符，将模型以OBJ格式输出
std::ostream& operator<<(std::ostream& out, Model &m) {
    for (int i=0; i<m.nverts(); i++) {                    // 遍历所有顶点
        out << "v " << m.point(i) << std::endl;           // 输出顶点数据
    }
    for (int i=0; i<m.nfaces(); i++) {                    // 遍历所有面
        out << "f ";
        for (int k=0; k<3; k++) {                         // 对面的三个顶点
            out << (m.vert(i,k)+1) << " ";                // 输出顶点索引(+1转回OBJ格式)
        }
        out << std::endl;
    }
    return out;                                           // 返回输出流
}

