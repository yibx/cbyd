#ifndef LIDAR_BG_DIFF_H
#define LIDAR_BG_DIFF_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <vector>
#include <mutex>
#include <Eigen/Dense>

// 点云类型统一定义
using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;
using PointCloudPtr = PointCloudT::Ptr;

// 船舶运行状态枚举
enum class ShipMonitorStatus
{
    SHIP_LEAVE,     // 船舶已离开泊位
    SHIP_MOVING,    // 船舶移动中
    SHIP_STABLE     // 船舶完全停稳
};

// 船舶基础位姿信息（原有结构保留）
struct ShipMove
{
    bool has_ship = false;
    // 包围盒极值
    double bbox_min_x = 0.0;
    double bbox_min_y = 0.0;
    double bbox_min_z = 0.0;
    double bbox_max_x = 0.0;
    double bbox_max_y = 0.0;
    double bbox_max_z = 0.0;
    // 包围盒中心
    double center_x = 0.0;
    double center_y = 0.0;
    double center_z = 0.0;
};

// 完整监测输出结果（新增，包含状态+位姿）
struct ShipMonitorResult
{
    ShipMonitorStatus status;
    ShipMove pose;
    int ship_point_count; // 当前船舶有效点数
};

// 历史船舶中心缓存，用于位移判定
struct ShipCenterRecord
{
    double cx, cy, cz;
};

class LidarBgDiff
{
public:
    LidarBgDiff();

    // 原有接口不修改，兼容旧调用
    void setDistanceThreshold(double dist_thresh);
    bool loadBackground(const std::string& bg_pcd_path);
    void setBackground(PointCloudPtr bg_cloud);
    void extractForeground(PointCloudPtr in_cloud, PointCloudPtr out_cloud);
    void removeSmallClusters(PointCloudPtr in_cloud, PointCloudPtr out_cloud,
        double eps, int min_points);
    ShipMove calculateShipPose(PointCloudPtr ship_cloud);
    ShipMove process(PointCloudPtr raw_cloud, PointCloudPtr ship_cloud,
        double cluster_eps, int cluster_min_pts);

    // ===================== 新增监测参数配置接口 =====================
    // 设置判定连续稳定帧数
    void setStableFrameCount(int frame_cnt);
    // 设置停稳质心位移阈值(米)
    void setStableDistThreshold(double dist_m);
    // 设置判定无船最小船舶点数
    void setMinValidShipPoints(int min_pts);

    // ===================== 新增完整监测主接口（推荐使用） =====================
    ShipMonitorResult monitorShip(PointCloudPtr raw_cloud,
        PointCloudPtr out_ship_cloud,
        double cluster_eps, int cluster_min_pts);

private:
    // 根据当前船舶中心更新历史队列、判断稳定状态
    ShipMonitorStatus judgeShipStatus(const ShipMove& cur_pose, int ship_pts);

private:
    // 原有成员
    double dist_thresh_;
    PointCloudPtr bg_cloud_;
    pcl::KdTreeFLANN<PointT> bg_kdtree_;
    bool bg_ready_;

    std::mutex mtx_; // 多线程保护历史缓存
    std::vector<ShipCenterRecord> center_history_; // 历史船舶中心序列

    // 停稳判定参数
    int stable_frame_cnt_ = 10;    // 连续10帧不动判定停稳
    double stable_dist_thresh_ = 0.4; // 位移小于0.4m视为静止
    int min_valid_ship_pts_ = 150;   // 小于该点数判定无船
};

#endif // LIDAR_BG_DIFF_H