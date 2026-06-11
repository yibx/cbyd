#ifndef POINT_CLOUD_FUSER_H
#define POINT_CLOUD_FUSER_H

#include "lock_free_queue.h"
#include "point_cloud_acquirer.h"
#include <thread>
#include <atomic>
#include <deque>

struct FusedPointCloud {
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
    FusedPointCloud fuse(const RawPointCloud& a, const RawPointCloud& b);

private:
    LockFreeRingQueue<RawPointCloud>* queueA_;
    LockFreeRingQueue<RawPointCloud>* queueB_;
    LockFreeRingQueue<FusedPointCloud>* queueOut_;

    std::deque<RawPointCloud> cacheA_;
    std::deque<RawPointCloud> cacheB_;

    std::thread thread_;
    std::atomic<bool> is_running_{false};
};

#endif