#include "data_outputter.h"
#include <iostream>
#include <chrono>
#include <iomanip>
#include "system_state_manager.h"

using SM = SystemStateManager;
using namespace std::chrono_literals;

DataOutputter::DataOutputter(LockFreeRingQueue<SixDofResult>* q3)
    : queue3_(q3)
{
}

void DataOutputter::start() {
    if (is_running_) return;

    // 状态机 -> 开始运行
    SM::instance().setState(SystemState::RUNNING);
    is_running_ = true;
    thread_ = std::thread(&DataOutputter::outputLoop, this);
}

void DataOutputter::stop() {
    is_running_ = false;
    if (thread_.joinable())
        thread_.join();

    // 状态机 -> 空闲
    SM::instance().setState(SystemState::IDLE);
}

void DataOutputter::outputLoop() {
    while (is_running_) {
        // -------------------------- 系统错误检查 --------------------------
        if (SM::instance().getState() == SystemState::FATAL_ERROR) {
            std::cerr << "[Outputter] 系统致命错误，退出输出线程" << std::endl;
            break;
        }

        SixDofResult r;
        if (queue3_->dequeue(r)) {
            bool db_ok = writeToDatabase(r);
            bool rt_ok = pushRealTimeData(r, "ws://127.0.0.1:8080/rt");

            if (!db_ok) {
                SM::instance().reportError(
                    ModuleType::OUTPUTTER,
                    ErrorLevel::STATUS_ERROR,
                    "数据库写入失败"
                );
            }
            if (!rt_ok) {
                SM::instance().reportError(
                    ModuleType::OUTPUTTER,
                    ErrorLevel::STATUS_WARNING,
                    "实时数据推送失败"
                );
            }
        }
        else {
            std::this_thread::sleep_for(100ms);
        }
    }

    SM::instance().setState(SystemState::IDLE);
}

// 显式实现子类构造函数，调用基类构造
ConcreteDataOutputter::ConcreteDataOutputter(LockFreeRingQueue<SixDofResult>* q3)
    : DataOutputter(q3)
{
    // 在这里注册错误推送回调
    SM::instance().setErrorPushCallback([this](const SystemError& err) {
        return pushRealTimeError(err, "ws://127.0.0.1:8080/error");
        });

    SM::instance().setDatabaseWriteCallback([this](const SystemError& err) {
        return writeErrorToDatabase(err);
        });
}

bool ConcreteDataOutputter::writeToDatabase(const SixDofResult& r) {
    try {
        std::cout << "[DB] 写入位姿结果: t=" << std::dec << std::setw(19) << r.timestamp << std::endl;
        std::cout << " tx=" << r.tx << " ty=" << r.ty << " tz=" << r.tz << " rx=" << r.rx << " ry=" << r.ry << " rz=" << r.rz << std::endl;
        return true;
    }
    catch (...) {
        return false;
    }
}

bool ConcreteDataOutputter::pushRealTimeData(const SixDofResult& r, const std::string& url) {
    try {
        std::cout << "[RT] 推送位姿: " << url
            << " rx=" << r.rx << " ry=" << r.ry << " tz=" << r.tz << " rx=" << r.rx << " ry=" << r.ry << " rz=" << r.rz << std::endl;
        return true;
    }
    catch (...) {
        return false;
    }
}

bool ConcreteDataOutputter::writeErrorToDatabase(const SystemError& err) {
    std::cout << "[DB-ERROR] 模块:" << static_cast<int>(err.module)
        << " 等级:" << static_cast<int>(err.level)
        << " 信息:" << err.message << std::endl;
    return true;
}

bool ConcreteDataOutputter::pushRealTimeError(const SystemError& err, const std::string& url) {
    std::cout << "[RT-ERROR] " << url
        << " 错误: " << err.message << std::endl;
    return true;
}
