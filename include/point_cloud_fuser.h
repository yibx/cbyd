#ifndef POINT_CLOUD_FUSER_H
#define POINT_CLOUD_FUSER_H

#include "lock_free_queue.h"
#include "point_cloud_acquirer.h"
#include <thread>
#include <atomic>
#include <deque>
#include "config_loader.h"

struct FusedPointCloud {
    std::string lidar_ip;
    std::string lidar_id;
    uint64_t timestamp;
    PointCloudT::Ptr cloud;
    int valid_points;
};

class PointCloudFuser {
public:

    PointCloudFuser(
        LockFreeRingQueue<RawPointCloud>* queueA,
        LockFreeRingQueue<RawPointCloud>* queueB,
        LockFreeRingQueue<FusedPointCloud>* queueOut
    );

    void start();
    void stop();

private:
    void fuseLoop();
    // 输入：A原始点云、B原始点云、B->A变换矩阵
    // 输出：融合后统一在A坐标系的点云结果
    FusedPointCloud fuse(const RawPointCloud& lidarA, const RawPointCloud& lidarB);

private:
    LockFreeRingQueue<RawPointCloud>* queueA_;
    LockFreeRingQueue<RawPointCloud>* queueB_;
    LockFreeRingQueue<FusedPointCloud>* queueOut_;

    std::deque<RawPointCloud> cacheA_;
    std::deque<RawPointCloud> cacheB_;

    std::thread thread_;
    std::atomic<bool> is_running_{false};

    RadarGlobalConfig fuse_cfg_;
    AllLidarConfigs lidarCfg_;
    Eigen::Affine3f T_A2B_;
    Eigen::Affine3f T_B2A_;
};

#endif