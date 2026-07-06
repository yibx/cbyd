#ifndef SIX_DOF_CALCULATOR_H
#define SIX_DOF_CALCULATOR_H

#include <thread>
#include <mutex>
#include <queue>
#include <condition_variable>
#include <vector>
#include <functional>

#include <fstream>
#include <sstream>
#include <chrono>
#include <iomanip>

#include "lock_free_queue.h"
#include "point_cloud_fuser.h"
#include "wharf_dock_checker.h"
#include "config_loader.h"

struct SixDofResult {
    std::string lidar_ip;
    std::string lidar_id;
    uint64_t timestamp;
    float tx, ty, tz;
    float rx, ry, rz;
    double confidence;
    double ship_length;
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
    void initFusionCsv();
    std::string csvWrap(const std::string& val);

    LockFreeRingQueue<FusedPointCloud>* rq_fuse_;
    LockFreeRingQueue<SixDofResult>* rq_sixdof_;
    bool is_running_ = false;
    std::thread thread_;
    FusedPointCloud fuse_pc_base_;

    WharfDockChecker dock_checker_;

    // 雷达坐标系 → 船舶坐标系的旋转矩阵
    Eigen::Matrix3d lr_sr_;

    // 线程池
    std::vector<std::thread> workers_;
    std::queue<std::function<void()>> tasks_;
    std::mutex task_mtx_;
    std::condition_variable cv_;
    bool pool_running_;

    AllLidarConfigs lidarCfg_;
    RegParam regCfg_;
    RadarGlobalConfig monitor_cfg_;
    Eigen::Affine3f T_A2dock_;
    Eigen::Affine3f T_B2dock_;

    std::chrono::steady_clock::time_point last_base_update_tp_;
    std::mutex base_mtx_; // 多线程保护基准帧

    float base_min_x_body_ = 0.0f;
    float base_max_x_body_ = 0.0f;
    bool base_extremum_calc_ = false;

    std::ofstream fusion_csv_;
    std::mutex fusion_csv_mtx_;
    bool fusion_header_wrote_ = false;
};

#endif
