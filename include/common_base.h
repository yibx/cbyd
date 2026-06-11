#ifndef COMMON_BASE_H
#define COMMON_BASE_H

#include <iostream>
using namespace std;

struct Point3d {
    double x, y, z;
    Point3d() : x(0), y(0), z(0) {}
    Point3d(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
};

// 船舶位姿
struct ShipPose {
    double tx, ty, tz;   // 位置（雷达坐标系）
    double rx, ry, rz;   // 姿态：横摇、纵摇、艏向（度）
    uint64_t timestamp;  // 时间戳
};
#endif
