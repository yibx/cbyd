#ifndef DATA_OUTPUTTER_H
#define DATA_OUTPUTTER_H

#include "lock_free_queue.h"
#include "six_dof_calculator.h"
#include "system_state_manager.h"
#include <thread>
#include <string>

class DataOutputter {
public:
    explicit DataOutputter(LockFreeRingQueue<SixDofResult>* q3);
    virtual ~DataOutputter() = default;

    void start();
    void stop();

    virtual bool writeToDatabase(const SixDofResult& r) = 0;
    virtual bool pushRealTimeData(const SixDofResult& r, const std::string& url) = 0;

    // 错误专用接口
    virtual bool writeErrorToDatabase(const SystemError& err) = 0;
    virtual bool pushRealTimeError(const SystemError& err, const std::string& url) = 0;

private:
    void outputLoop();

    LockFreeRingQueue<SixDofResult>* queue3_;
    std::thread thread_;
    std::atomic<bool> is_running_{false};
};

class ConcreteDataOutputter : public DataOutputter {
public:
    // 移除 using DataOutputter::DataOutputter;
    // 改为显式声明构造函数
    explicit ConcreteDataOutputter(LockFreeRingQueue<SixDofResult>* q3);

    bool writeToDatabase(const SixDofResult& r) override;
    bool pushRealTimeData(const SixDofResult& r, const std::string& url) override;

    bool writeErrorToDatabase(const SystemError& err) override;
    bool pushRealTimeError(const SystemError& err, const std::string& url) override;
};

#endif