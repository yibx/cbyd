#include "point_cloud_fuser.h"
#include <chrono>
#include <iostream>
#include "Logger.h" 
#include <pcl/io/pcd_io.h>
#include "system_state_manager.h"
using SM = SystemStateManager;

using namespace std;
using namespace std::chrono;

#define MATCH_NS 500 // 匹配时间差阈值，单位纳秒
#define MAX_CACHE_FRAME 30 // 缓存最大帧数
#define FRAME_TIMEOUT_NS 2000 // 帧超时阈值，超过该时差强制丢弃旧帧
#define FRAME_CACHE 2 // 缓存1s帧数的
#define ENTER_FUSE_TIMEOUT 5 // 融合点云进入队列的超时时间（毫秒）

PointCloudFuser::PointCloudFuser(
    LockFreeRingQueue<RawPointCloud>* queueA,
    LockFreeRingQueue<RawPointCloud>* queueB,
    LockFreeRingQueue<FusedPointCloud>* queueOut
)
    : queueA_(queueA), queueB_(queueB), queueOut_(queueOut)
{}

void PointCloudFuser::start() {
    if (is_running_) return;
    is_running_ = true;

    if (!loadLidarConfigs("../dev_config.yaml", lidarCfg_)){
        std::string err_msg = "雷达配置加载失败，请检查dev_config.yaml文件";
        Logger::instance().info(err_msg);
        return;
    }

    if (!loadMonitorConfig("../monitor_config.yaml", fuse_cfg_)) {
        std::string err_msg = "监测配置加载失败，请检查monitor_config.yaml文件";
        Logger::instance().info(err_msg);
        return;
    }

    // 获取雷达A到码头坐标系的变换矩阵
    Eigen::Affine3f T_A2dock_ = getLidarATrans(fuse_cfg_.lidarA);
    // 获取雷达B到码头坐标系的变换矩阵
    Eigen::Affine3f T_B2dock_ = getLidarBTrans(fuse_cfg_.lidarB);
    T_B2A_ = T_A2dock_.inverse() * T_B2dock_;
    // 用不到，暂时保留
    T_A2B_ = T_B2A_.inverse();


    thread_ = std::thread(&PointCloudFuser::fuseLoop, this);
}

void PointCloudFuser::stop() {
    is_running_ = false;
    if (thread_.joinable()) thread_.join();
}

// ==========================================================
// 融合循环
// ==========================================================
void PointCloudFuser::fuseLoop() {
    RawPointCloud dataA, dataB;

    std::string err_msg;
    while (is_running_) {
        // 1. 从队列A取数据 → 放入缓存A
        if (queueA_->dequeue(dataA)) {
            cacheA_.push_back(dataA);
            // 缓存超限，删除最旧帧
            while (cacheA_.size() > MAX_CACHE_FRAME) {
                cacheA_.erase(cacheA_.begin());
                err_msg = "雷达A数据缓存超限，丢弃旧帧";
                Logger::instance().info(err_msg);
            }
        }

        // 2. 从队列B取数据 → 放入缓存B
        if (queueB_->dequeue(dataB)) {
            cacheB_.push_back(dataB);
            // 缓存超限，删除最旧帧
            while (cacheB_.size() > MAX_CACHE_FRAME) {
                cacheB_.erase(cacheB_.begin());
                err_msg = "雷达B数据缓存超限，丢弃旧帧";
                Logger::instance().info(err_msg);
            }
        }

        if ((cacheA_.size() <= FRAME_CACHE) || (cacheB_.size() <= FRAME_CACHE)) {
            this_thread::sleep_for(milliseconds(5));
            continue;
        }

        bool matched = false;
        while (!cacheA_.empty() && !cacheB_.empty())
        {
            auto& a = cacheA_[0];
            auto& b = cacheB_[0];

            uint64_t diff_ns = (a.timestamp > b.timestamp)
                ? (a.timestamp - b.timestamp)
                : (b.timestamp - a.timestamp);

            // 匹配成功：融合
            if (diff_ns < MATCH_NS)
            {
                matched = true;
                //cout << "[融合] 帧间差值: " << diff_ns << endl;
                // 融合点云
                FusedPointCloud fused = fuse(a, b);

                // 对 fused 做深拷贝，避免共享指针
                FusedPointCloud fused_deep;
                fused_deep.lidar_ip = fused.lidar_ip;
                fused_deep.lidar_id = fused.lidar_id;
                fused_deep.timestamp = fused.timestamp;
                fused_deep.cloud.reset(new PointCloudT(*fused.cloud)); // 深拷贝
                fused_deep.valid_points = fused_deep.cloud->size();

                // 入队给6DoF
                while (!queueOut_->enqueue(fused_deep) && is_running_) {
                    this_thread::sleep_for(milliseconds(ENTER_FUSE_TIMEOUT));
                }

                // 清除已配对帧
                cacheA_.erase(cacheA_.begin());
                cacheB_.erase(cacheB_.begin());
            }   else if (a.timestamp < b.timestamp) {
                cacheA_.erase(cacheA_.begin());
                err_msg = "雷达A数据超时，丢弃旧帧";
                Logger::instance().info(err_msg);
                SM::instance().reportError(
                    ModuleType::FUSER,
                    ErrorLevel::STATUS_WARNING,
                    "雷达点云时间戳失配"
                );
            }   else {
                cacheB_.erase(cacheB_.begin());
                err_msg = "雷达B数据超时，丢弃旧帧";
                Logger::instance().info(err_msg);
                SM::instance().reportError(
                    ModuleType::FUSER,
                    ErrorLevel::STATUS_WARNING,
                    "雷达点云时间戳失配"
                );
            }
        }

        // ======================================================
        // 只有A有数据：直接输出A（深拷贝）
        // ======================================================
        if (!matched && cacheB_.empty() && !cacheA_.empty()) {
            matched = true;
            auto& a = cacheA_[0];
            FusedPointCloud fused;
            fused.lidar_ip = a.lidar_ip;
            fused.lidar_id = a.lidar_id; 
            fused.timestamp = a.timestamp;
            fused.cloud.reset(new PointCloudT(*a.cloud)); // 深拷贝
            fused.valid_points = fused.cloud->size();

            while (!queueOut_->enqueue(fused) && is_running_) {
                this_thread::sleep_for(milliseconds(ENTER_FUSE_TIMEOUT));
            }

            cacheA_.erase(cacheA_.begin());
        }

        // ======================================================
        // 只有B有数据：直接输出B（深拷贝）
        // ======================================================
        if (!matched && cacheA_.empty() && !cacheB_.empty()) {
            matched = true;
            auto& b = cacheB_[0];
            FusedPointCloud fused;
            fused.lidar_ip = b.lidar_ip;
            fused.lidar_id = b.lidar_id; 
            fused.timestamp = b.timestamp;
            fused.cloud.reset(new PointCloudT(*b.cloud)); // 深拷贝
            fused.valid_points = fused.cloud->size();

            while (!queueOut_->enqueue(fused) && is_running_) {
                this_thread::sleep_for(milliseconds(ENTER_FUSE_TIMEOUT));
            }

            cacheB_.erase(cacheB_.begin());
        }

        this_thread::sleep_for(milliseconds(2));
    }
}

// 双雷达点云融合函数：B转换至A坐标系
FusedPointCloud PointCloudFuser::fuse(const RawPointCloud& lidarA,
                                const RawPointCloud& lidarB) {
    FusedPointCloud fused_res;
    // 取两帧最大时间戳作为融合帧时间
    fused_res.timestamp = std::max(lidarA.timestamp, lidarB.timestamp);
    fused_res.cloud.reset(new PointCloudT);

    // 雷达A原始点云直接并入融合结果
    *fused_res.cloud += *lidarA.cloud;

    // 雷达B点云变换到A雷达坐标系
    PointCloudT::Ptr b_aligned_cloud(new PointCloudT);
    pcl::transformPointCloud(*lidarB.cloud, *b_aligned_cloud, T_B2A_);
    *fused_res.cloud += *b_aligned_cloud;

    // 有效总点数
    fused_res.valid_points = fused_res.cloud->size();
    fused_res.lidar_id = lidarA.lidar_id;
    fused_res.lidar_ip = lidarA.lidar_ip + std::string(",") + lidarB.lidar_ip;

    if (lidarCfg_.debug_save) {
        const std::string pcd_dir = "./pcd_debug";
        std::string pcd_name = pcd_dir + "/lidar_fuse_frame_" + std::to_string(fused_res.timestamp) + ".pcd";
        pcl::io::savePCDFileBinary(pcd_name, *fused_res.cloud);
        Logger::instance().info("[INFO] Save fused frame to: " + pcd_name);
    }

    return fused_res;
}