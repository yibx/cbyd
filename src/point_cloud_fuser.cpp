#include "point_cloud_fuser.h"
#include <chrono>
#include <iostream>

#include "system_state_manager.h"
using SM = SystemStateManager;

using namespace std;
using namespace std::chrono;

#define MATCH_NS 5

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
    thread_ = std::thread(&PointCloudFuser::fuseLoop, this);
}

void PointCloudFuser::stop() {
    is_running_ = false;
    if (thread_.joinable()) thread_.join();
}

// ==========================================================
// 融合循环（两路融合 + 单路直接输出 + 全程深拷贝）
// ==========================================================
void PointCloudFuser::fuseLoop() {
    RawPointCloud dataA, dataB;

    while (is_running_) {
        // 1. 从队列A取数据 → 放入缓存A
        if (queueA_->dequeue(dataA)) {
            cacheA_.push_back(dataA);
        }

        // 2. 从队列B取数据 → 放入缓存B
        if (queueB_->dequeue(dataB)) {
            cacheB_.push_back(dataB);
        }

        // ======================================================
        // 两路都存在：时间戳配对 + 融合
        // ======================================================
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
                cout << "[融合] 帧间差值: " << diff_ns << endl;

                // ====================== 融合点云
                FusedPointCloud fused = fuse(a, b);

                // ====================== 【关键】对 fused 做深拷贝，避免共享指针
                FusedPointCloud fused_deep;
                fused_deep.lidar_id = fused.lidar_id;
                fused_deep.timestamp = fused.timestamp;
                fused_deep.cloud.reset(new PointCloudT(*fused.cloud)); // 深拷贝
                fused_deep.valid_points = fused_deep.cloud->size();

                // 入队给6DoF
                while (!queueOut_->enqueue(fused_deep) && is_running_) {
                    this_thread::sleep_for(milliseconds(2));
                }

                // 清除已配对帧
                cacheA_.erase(cacheA_.begin());
                cacheB_.erase(cacheB_.begin());
            }
            // 丢弃旧帧
            else if (a.timestamp < b.timestamp) {
                cacheA_.erase(cacheA_.begin());
                cout << "[丢弃] 旧帧 A" << endl;
                SM::instance().reportError(
                    ModuleType::FUSER,
                    ErrorLevel::STATUS_WARNING,
                    "雷达点云时间戳失配"
                );
            }
            else {
                cacheB_.erase(cacheB_.begin());
                cout << "[丢弃] 旧帧 B" << endl;
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
        if (cacheB_.empty() && !cacheA_.empty()) {
            auto& a = cacheA_[0];
            //cout << "[单路输出] 仅雷达A" << endl;

            FusedPointCloud fused;
            fused.lidar_id = a.lidar_id; // 雷达A帧标识为0
            fused.timestamp = a.timestamp;
            fused.cloud.reset(new PointCloudT(*a.cloud)); // 深拷贝
            fused.valid_points = fused.cloud->size();
	    //std::cout << "fuse ns:" << fused.timestamp << ", fuse size:" << fused.cloud->size() << std::endl;
            while (!queueOut_->enqueue(fused) && is_running_) {
                this_thread::sleep_for(milliseconds(2));
            }

            cacheA_.erase(cacheA_.begin());
        }

        // ======================================================
        // 只有B有数据：直接输出B（深拷贝）
        // ======================================================
        if (cacheA_.empty() && !cacheB_.empty()) {
            auto& b = cacheB_[0];
            cout << "[单路输出] 仅雷达B" << endl;

            FusedPointCloud fused;
            fused.lidar_id = b.lidar_id; // 雷达B帧标识为1
            fused.timestamp = b.timestamp;
            fused.cloud.reset(new PointCloudT(*b.cloud)); // 深拷贝
            fused.valid_points = fused.cloud->size();

            while (!queueOut_->enqueue(fused) && is_running_) {
                this_thread::sleep_for(milliseconds(2));
            }

            cacheB_.erase(cacheB_.begin());
        }

        this_thread::sleep_for(milliseconds(2));
    }
}

// ==========================================================
// 点云融合
// ==========================================================
FusedPointCloud PointCloudFuser::fuse(const RawPointCloud& a, const RawPointCloud& b) {
    FusedPointCloud f;
    f.timestamp = max(a.timestamp, b.timestamp);
    f.cloud = PointCloudT::Ptr(new PointCloudT);

    *f.cloud += *a.cloud;
    *f.cloud += *b.cloud;
    f.valid_points = f.cloud->size();
    f.lidar_id = "127.0.0.1";

    return f;
}

