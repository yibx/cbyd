#include "point_cloud_acquirer.h"
#include "point_cloud_fuser.h"
#include "six_dof_calculator.h"
#include "data_outputter.h"
#include <chrono>
#include <signal.h>
#include <atomic>
#include <Logger.h>

// 全局原子标志，用于信号处理
std::atomic<bool> running{true};

// Ctrl+C 信号处理函数
void sigint_handler(int sig) {
    running = false;
    std::cout << "\nReceived Ctrl+C, stopping threads..." << std::endl;
}

int main() {
    // 注册信号处理
    signal(SIGINT, sigint_handler);

    Logger::instance().info("start service");

    LockFreeRingQueue<RawPointCloud> rq_cloudA(100);
    LockFreeRingQueue<RawPointCloud> rq_cloudB(100);
    LockFreeRingQueue<FusedPointCloud> rq_fuse_cloud(100);
    LockFreeRingQueue<SixDofResult> rq_six_dof(100);

    PointCloudAcquirer acquirer(&rq_cloudA, &rq_cloudB);
    PointCloudFuser fuser(&rq_cloudA, &rq_cloudB, &rq_fuse_cloud);
    SixDofCalculator calc(&rq_fuse_cloud, &rq_six_dof);
    ConcreteDataOutputter output(&rq_six_dof);

    acquirer.start();
    fuser.start();
    calc.start();
    output.start();

    // 持续运行，直到 running 变为 false
    while (running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // 收到信号后，按顺序停止所有线程
    output.stop();
    calc.stop();
    fuser.stop();
    acquirer.stop();

    std::cout << "All threads stopped, exiting." << std::endl;
    return 0;
}
