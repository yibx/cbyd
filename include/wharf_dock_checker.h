#ifndef WHARF_DOCK_CHECKER_H
#define WHARF_DOCK_CHECKER_H

#include <vector>
#include <deque>
#include "common_base.h"

// 功能：码头直线拟合 → 计算码头角度 → 自动筛选停稳基准帧
class WharfDockChecker {
public:
    WharfDockChecker();

    // 输入码头边缘N个点 → 拟合直线 Ax+By+C=0
    bool fitWharfLine(const std::vector<Point3d>& wharf_points);

    // 计算码头朝向角（固定值）
    double calcWharfYawDegree();

    // 输入每一帧船舶位姿
    void addPoseFrame(const ShipPose& pose);

    // 【核心】自动判断是否达到停稳状态 → 若是，返回基准帧
    bool getStableDockBaseFrame(ShipPose& base_frame);

    // 获取码头直线参数
    void getWharfLine(double& A, double& B, double& C);

private:
    // 角度归一化 [-180, 180]
    double wrapAngle(double deg);

    // 点到码头距离
    double distanceToWharf(double x, double y);

private:
    double wharf_A_, wharf_B_, wharf_C_;  // 码头直线 Ax+By+C=0
    double wharf_yaw_;                    // 码头朝向角（固定）

    // 滑动窗口
    std::deque<ShipPose> pose_buf_;
    const int WINDOW_SIZE = 50;           // 可根据帧率调整

    // 停稳阈值
    const double MAX_ANGLE_ERR = 3.0;     // 艏向最大误差
    const double MAX_DIST_MIN = 1.0;      // 停靠最近距离
    const double MAX_DIST_MAX = 8.0;      // 停靠最远距离
    const double MAX_YAW_FLUCTUATION = 2.0;
    const double MAX_DIST_FLUCTUATION = 0.5;

    bool has_found_base_frame_;
    ShipPose base_frame_;
};

#endif
