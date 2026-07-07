#include "point_cloud_acquirer.h"
#include "point_cloud_fuser.h"
#include "six_dof_calculator.h"
#include "data_outputter.h"
#include <chrono>
#include <signal.h>
#include <atomic>
#include <Logger.h>
#include "version.h"
// 全局原子标志，用于信号处理
std::atomic<bool> running{true};

// 信号处理函数
void sigint_handler(int sig) {
    running = false;
    std::cout << "\nReceived Ctrl+C, stopping threads..." << std::endl;
}

int main(int argc, char* argv[]) {
    // 查询版本指令：./cbyd --version
    if (argc >= 2 && std::string(argv[1]) == "--version")
    {
        std::cout << "Software Version: " << SW_VERSION_STR << std::endl;
        std::cout << "Build Time: " << SW_BUILD_TIME << std::endl;
        return 0;
    }
    // 注册信号处理
    signal(SIGINT, sigint_handler);
    // 初始化日志
    IndustrialLogger::Instance()->Init("./logs/app", 10*1024*1024, 30, 8192);
    IndustrialLogger::Instance()->SetLogLevel(spdlog::level::debug);

    std::string ver = std::string("Software Version: ") + SW_VERSION_STR;
    LOG_INFO("MAIN", ver);

    std::string build_time = std::string("Build Time: ") + SW_BUILD_TIME;
    LOG_INFO("MAIN", build_time);

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

    IndustrialLogger::Instance()->Flush();
    IndustrialLogger::Instance()->Shutdown();

    LOG_INFO("MAIN", "All threads stopped, exiting");

    return 0;
}
