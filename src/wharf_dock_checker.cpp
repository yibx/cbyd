#include "wharf_dock_checker.h"
#include <cmath>
#include <algorithm>

WharfDockChecker::WharfDockChecker()
    : wharf_A_(0), wharf_B_(0), wharf_C_(0), wharf_yaw_(0), has_found_base_frame_(false) {
}

// 最小二乘拟合码头直线 Ax+By+C=0
bool WharfDockChecker::fitWharfLine(const std::vector<Point3d>& wharf_points) {
    int n = wharf_points.size();
    if (n < 2) return false;

    double x_sum = 0, y_sum = 0;
    double x2_sum = 0, y2_sum = 0, xy_sum = 0;
    for (auto& p : wharf_points) {
        x_sum += p.x;
        y_sum += p.y;
        x2_sum += p.x * p.x;
        y2_sum += p.y * p.y;
        xy_sum += p.x * p.y;
    }

    double A = n * xy_sum - x_sum * y_sum;
    double B = y_sum * y_sum - n * y2_sum;
    double C = x_sum * (y2_sum - xy_sum) - y_sum * (x2_sum - xy_sum);

    double norm = sqrt(A*A + B*B);
    if (norm < 1e-6) return false;

    wharf_A_ = A / norm;
    wharf_B_ = B / norm;
    wharf_C_ = C / norm;
    return true;
}

// 计算码头朝向角
double WharfDockChecker::calcWharfYawDegree() {
    double angle_rad = atan2(-wharf_A_, wharf_B_);
    wharf_yaw_ = angle_rad * 180.0 / M_PI;
    return wharf_yaw_;
}

// 角度归一化
double WharfDockChecker::wrapAngle(double deg) {
    deg = fmod(deg + 180.0, 360.0);
    if (deg < 0) deg += 360.0;
    return deg - 180.0;
}

// 点到码头距离
double WharfDockChecker::distanceToWharf(double x, double y) {
    return fabs(wharf_A_ * x + wharf_B_ * y + wharf_C_);
}

// 添加一帧位姿
void WharfDockChecker::addPoseFrame(const ShipPose& pose) {
    if (has_found_base_frame_) return;

    pose_buf_.push_back(pose);
    if (pose_buf_.size() > WINDOW_SIZE)
        pose_buf_.pop_front();
}

// 自动筛选停稳基准帧
bool WharfDockChecker::getStableDockBaseFrame(ShipPose& base_frame) {
    if (has_found_base_frame_) {
        base_frame = base_frame_;
        return true;
    }

    if (pose_buf_.size() < WINDOW_SIZE)
        return false;

    // 计算窗口内均值与波动
    double yaw_sum = 0, dist_sum = 0;
    double ymin = 1e9, ymax = -1e9;
    double dmin = 1e9, dmax = -1e9;

    for (auto& p : pose_buf_) {
        double yaw = wrapAngle(p.rz);
        double dist = distanceToWharf(p.tx, p.ty);
        yaw_sum += yaw;
        dist_sum += dist;

        if (yaw < ymin) ymin = yaw;
        if (yaw > ymax) ymax = yaw;
        if (dist < dmin) dmin = dist;
        if (dist > dmax) dmax = dist;
    }

    double yaw_mean = wrapAngle(yaw_sum / WINDOW_SIZE);
    double dist_mean = dist_sum / WINDOW_SIZE;
    double yaw_fluct = ymax - ymin;
    double dist_fluct = dmax - dmin;
    double angle_err = fabs(wrapAngle(yaw_mean - wharf_yaw_));

    // 停稳判定
    bool c1 = (angle_err < MAX_ANGLE_ERR);
    bool c2 = (dist_mean >= MAX_DIST_MIN && dist_mean <= MAX_DIST_MAX);
    bool c3 = (yaw_fluct < MAX_YAW_FLUCTUATION);
    bool c4 = (dist_fluct < MAX_DIST_FLUCTUATION);

    if (c1 && c2 && c3 && c4) {
        base_frame_ = pose_buf_[WINDOW_SIZE / 2];
        has_found_base_frame_ = true;
        base_frame = base_frame_;
        return true;
    }

    return false;
}

void WharfDockChecker::getWharfLine(double& A, double& B, double& C) {
    A = wharf_A_;
    B = wharf_B_;
    C = wharf_C_;
}
