#ifndef SIX_DOF_CALCULATOR_H
#define SIX_DOF_CALCULATOR_H

#include <thread>
#include <mutex>
#include <queue>
#include <condition_variable>
#include <vector>
#include <functional>

#include "lock_free_queue.h"
#include "point_cloud_fuser.h"
#include "wharf_dock_checker.h"
#include "config_loader.h"

struct SixDofResult {
    uint64_t timestamp;
    float tx, ty, tz;
    float rx, ry, rz;
    double confidence;
};

class SixDofCalculator {
public:
    SixDofCalculator(LockFreeRingQueue<FusedPointCloud>* rq_fuse,
                     LockFreeRingQueue<SixDofResult>* rq_sixdof);
    void start();
    void stop();

private:
    SixDofResult calculateSixDof(const FusedPointCloud& c);
    void calcLoop();
    void workerThread(int core_id);

    LockFreeRingQueue<FusedPointCloud>* rq_fuse_;
    LockFreeRingQueue<SixDofResult>* rq_sixdof_;
    bool is_running_ = false;
    std::thread thread_;
    FusedPointCloud fuse_pc_base_;

    WharfDockChecker dock_checker_;

    bool b_set_base_;
    // 雷达坐标系 → 船舶坐标系的旋转矩阵
    Eigen::Matrix3d lr_sr_;

    // 线程池
    std::vector<std::thread> workers_;
    std::queue<std::function<void()>> tasks_;
    std::mutex task_mtx_;
    std::condition_variable cv_;
    bool pool_running_;

    RegParam regCfg_;
};

#endif
