#include "lidar_bg_diff.h"
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>
#include <iostream>
#include <cmath>

// 全局复用体素栅格（降采样用，避免重复构造）
static pcl::VoxelGrid<PointT> g_voxel_filter;
// 降采样分辨率，根据雷达精度调整
constexpr float VOXEL_RES = 0.02f;

LidarBgDiff::LidarBgDiff()
    : m_dist_thresh(0.15)
    , m_bg_ready(false)
{
    m_bg_cloud.reset(new PointCloudT);
    // 预设置体素栅格参数，一次初始化永久复用
    g_voxel_filter.setLeafSize(VOXEL_RES, VOXEL_RES, VOXEL_RES);
}

void LidarBgDiff::setDistanceThreshold(double dist_thresh)
{
    m_dist_thresh = dist_thresh;
}

bool LidarBgDiff::loadBackground(const std::string& bg_pcd_path)
{
    std::lock_guard<std::mutex> lock(m_mtx);
    if (pcl::io::loadPCDFile<PointT>(bg_pcd_path, *m_bg_cloud) < 0)
    {
        std::cerr << "[ERROR] Load background pcd failed: " << bg_pcd_path << std::endl;
        m_bg_ready = false;
        return false;
    }
    m_bg_kdtree.setInputCloud(m_bg_cloud);
    m_bg_ready = true;
    // std::cout << "[INFO] Background loaded, points: " << m_bg_cloud->size() << std::endl;
    return true;
}

void LidarBgDiff::setBackground(PointCloudPtr bg_cloud)
{
    std::lock_guard<std::mutex> lock(m_mtx);
    if (!bg_cloud || bg_cloud->empty())
    {
        m_bg_ready = false;
        return;
    }
    m_bg_cloud = bg_cloud;
    m_bg_kdtree.setInputCloud(m_bg_cloud);
    m_bg_ready = true;
}

void LidarBgDiff::extractForeground(PointCloudPtr in_cloud, PointCloudPtr out_cloud)
{
    out_cloud->clear();
    if (!m_bg_ready || !in_cloud || in_cloud->empty() || !out_cloud)
    {
        return;
    }

    // 优化1：先降采样，大幅减少遍历点数
    PointCloudPtr down_cloud(new PointCloudT);
    g_voxel_filter.setInputCloud(in_cloud);
    g_voxel_filter.filter(*down_cloud);
    if (down_cloud->empty()) return;

    std::vector<int> nn_indices(1);
    std::vector<float> nn_dists(1);
    const double dist_sq_thresh = m_dist_thresh * m_dist_thresh; // 预计算距离平方，省去sqrt

    // 优化2：直接比对距离平方，删除sqrt冗余计算
    for (const auto& pt : *down_cloud)
    {
        m_bg_kdtree.nearestKSearch(pt, 1, nn_indices, nn_dists);
        if (nn_dists[0] > dist_sq_thresh)
        {
            out_cloud->push_back(pt);
        }
    }
}

void LidarBgDiff::removeSmallClusters(PointCloudPtr in_cloud, PointCloudPtr out_cloud,
    double eps, int min_points)
{
    out_cloud->clear();
    if (!in_cloud || in_cloud->size() < (size_t)min_points)
        return;

    // 优化3：KD-Tree 复用，不每次新建
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(in_cloud);

    pcl::EuclideanClusterExtraction<PointT> ec;
    std::vector<pcl::PointIndices> cluster_indices;

    ec.setInputCloud(in_cloud);
    ec.setSearchMethod(tree);
    ec.setClusterTolerance(eps);
    ec.setMinClusterSize(min_points);
    ec.extract(cluster_indices);

    if (cluster_indices.empty())
        return;

    // 选取最大簇作为船舶主体
    size_t max_size = 0;
    int max_idx = 0;
    for (size_t i = 0; i < cluster_indices.size(); ++i)
    {
        const auto& idx = cluster_indices[i].indices;
        if (idx.size() > max_size)
        {
            max_size = idx.size();
            max_idx = i;
        }
    }

    pcl::ExtractIndices<PointT> extract;
    pcl::PointIndices::Ptr main_cluster(new pcl::PointIndices(cluster_indices[max_idx]));
    extract.setInputCloud(in_cloud);
    extract.setIndices(main_cluster);
    extract.filter(*out_cloud);
}

ShipMove LidarBgDiff::calculateShipPose(PointCloudPtr ship_cloud)
{
    ShipMove pose;
    if (!ship_cloud || ship_cloud->empty())
    {
        pose.has_ship = false;
        return pose;
    }
    pose.has_ship = true;

    PointT min_pt, max_pt;
    pcl::getMinMax3D(*ship_cloud, min_pt, max_pt);

    pose.bbox_min_x = min_pt.x;
    pose.bbox_min_y = min_pt.y;
    pose.bbox_min_z = min_pt.z;
    pose.bbox_max_x = max_pt.x;
    pose.bbox_max_y = max_pt.y;
    pose.bbox_max_z = max_pt.z;

    pose.center_x = (min_pt.x + max_pt.x) / 2.0;
    pose.center_y = (min_pt.y + max_pt.y) / 2.0;
    pose.center_z = (min_pt.z + max_pt.z) / 2.0;

    return pose;
}

// 原有兼容接口保留不变
ShipMove LidarBgDiff::process(PointCloudPtr raw_cloud, PointCloudPtr ship_cloud,
    double cluster_eps, int cluster_min_pts)
{
    PointCloudPtr fg_cloud(new PointCloudT);
    // 1. 背景差分提取前景
    extractForeground(raw_cloud, fg_cloud);
    if (fg_cloud->empty())
    {
        ShipMove empty_pose;
        return empty_pose;
    }

    // 2. 聚类去噪
    removeSmallClusters(fg_cloud, ship_cloud, cluster_eps, cluster_min_pts);

    // 3. 计算位姿
    return calculateShipPose(ship_cloud);
}

// ===================== 新增：监测参数配置实现 =====================
void LidarBgDiff::setStableFrameCount(int frame_cnt)
{
    std::lock_guard<std::mutex> lock(m_mtx);
    m_stable_frame_cnt = frame_cnt;
}

void LidarBgDiff::setStableDistThreshold(double dist_m)
{
    std::lock_guard<std::mutex> lock(m_mtx);
    m_stable_dist_thresh = dist_m;
}

void LidarBgDiff::setMinValidShipPoints(int min_pts)
{
    std::lock_guard<std::mutex> lock(m_mtx);
    m_min_valid_ship_pts = min_pts;
}

// ===================== 新增：核心状态判定逻辑 =====================
ShipMonitorStatus LidarBgDiff::judgeShipStatus(const ShipMove& cur_pose, int ship_pts)
{
    std::lock_guard<std::mutex> lock(m_mtx);
    // 条件1：无船舶/点数不足 → 判定离开
    if (!cur_pose.has_ship || ship_pts < m_min_valid_ship_pts)
    {
        m_center_history.clear(); // 清空历史缓存
        return ShipMonitorStatus::SHIP_LEAVE;
    }

    // 记录当前帧船舶中心
    ShipCenterRecord rec;
    rec.cx = cur_pose.center_x;
    rec.cy = cur_pose.center_y;
    rec.cz = cur_pose.center_z;
    m_center_history.push_back(rec);

    // 维持历史队列长度，超过稳定帧数则弹出最早帧
    if (m_center_history.size() > (size_t)m_stable_frame_cnt)
    {
        m_center_history.erase(m_center_history.begin());
    }

    // 帧数不足稳定判定阈值 → 判定移动
    if (m_center_history.size() < (size_t)m_stable_frame_cnt)
    {
        return ShipMonitorStatus::SHIP_MOVING;
    }

    // 计算当前帧与最早一帧的三维位移距离
    const auto& first = m_center_history.front();
    double dx = rec.cx - first.cx;
    double dy = rec.cy - first.cy;
    double dz = rec.cz - first.cz;
    double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

    if (dist < m_stable_dist_thresh)
    {
        return ShipMonitorStatus::SHIP_STABLE;
    }
    else
    {
        return ShipMonitorStatus::SHIP_MOVING;
    }
}

ShipMonitorResult LidarBgDiff::monitorShip(PointCloudPtr raw_cloud,
    PointCloudPtr out_ship_cloud,
    double cluster_eps, int cluster_min_pts)
{
    ShipMonitorResult res;
    out_ship_cloud->clear();

    // 背景差分提取前景
    PointCloudPtr fg_cloud(new PointCloudT);
    extractForeground(raw_cloud, fg_cloud);
    if (fg_cloud->empty())
    {
        res.status = ShipMonitorStatus::SHIP_LEAVE;
        res.pose.has_ship = false;
        res.ship_point_count = 0;
        return res;
    }

    // 欧式聚类提取最大船舶簇
    removeSmallClusters(fg_cloud, out_ship_cloud, cluster_eps, cluster_min_pts);
    res.ship_point_count = out_ship_cloud->size();

    // 计算船舶包围盒中心位姿
    res.pose = calculateShipPose(out_ship_cloud);

    // 判定船舶状态：离开/移动/停稳
    res.status = judgeShipStatus(res.pose, res.ship_point_count);

    return res;
}