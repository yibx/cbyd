#ifndef POINT_CLOUD_ACQUIRER_H
#define POINT_CLOUD_ACQUIRER_H

#include "lock_free_queue.h"
#include <thread>
#include <vector>
#include <atomic>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "config_loader.h"
#include "lidar_bg_diff.h"

using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;


struct RawPointCloud {
    std::string lidar_ip;
    std::string lidar_id;     
    uint64_t timestamp;        
    PointCloudT::Ptr cloud;   
};

class PointCloudAcquirer {
public:
    
    PointCloudAcquirer(
        LockFreeRingQueue<RawPointCloud>* queueA,
        LockFreeRingQueue<RawPointCloud>* queueB
    );

    void start();
    void stop();

private:
    void acquireRadar1Loop();  
    void acquireRadar2Loop();  

    bool initLidarBgProcessor(LidarBgDiff& proc, int lidar_sel);
    ShipMonitorResult processOneLidarFrame(LidarBgDiff& proc, const PointCloudT::Ptr raw_cloud, PointCloudT::Ptr ship_out, int lidar_sel);

private:
    LockFreeRingQueue<RawPointCloud>* queueA_;
    LockFreeRingQueue<RawPointCloud>* queueB_;
    int port_dev_;
    int port_data_;

    std::thread thread_radar1_;
    std::thread thread_radar2_;
    std::atomic<bool> is_running_{false};

    AllLidarConfigs lidarCfg_;
    RadarGlobalConfig monitor_cfg_;
};

#endif