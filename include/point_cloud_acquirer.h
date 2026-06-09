#ifndef POINT_CLOUD_ACQUIRER_H
#define POINT_CLOUD_ACQUIRER_H

#include "lock_free_queue.h"
#include <thread>
#include <vector>
#include <atomic>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;

// 真正的雷达点云结构（适配你的工程）
struct RawPointCloud {
    std::string radar_id;     // A 或 B
    uint64_t timestamp;        // 纳秒时间戳
    PointCloudT::Ptr cloud;   // 点云智能指针
};

class PointCloudAcquirer {
public:
    // 传入两个独立队列
    PointCloudAcquirer(
        LockFreeRingQueue<RawPointCloud>* queueA,
        LockFreeRingQueue<RawPointCloud>* queueB,
        int port_dev,
        int port_data
    );

    void start();
    void stop();

private:
    void acquireRadar1Loop();  // 雷达A独立线程
    void acquireRadar2Loop();  // 雷达B独立线程

private:
    LockFreeRingQueue<RawPointCloud>* queueA_;
    LockFreeRingQueue<RawPointCloud>* queueB_;
    int port_dev_;
    int port_data_;

    std::thread thread_radar1_;
    std::thread thread_radar2_;
    std::atomic<bool> is_running_{false};
};

#endif