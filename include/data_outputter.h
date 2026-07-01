#ifndef DATA_OUTPUTTER_H
#define DATA_OUTPUTTER_H

#include "six_dof_calculator.h"
#include "industrial_mqtt_client.h"
#include "system_state_manager.h"
#include "lock_free_queue.h"
#include "config_loader.h"
#include "Logger.h"

#include <thread>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <Eigen/Core>
#include <chrono>
#include <string>
#include <regex>
#include <stdexcept>

using SM = SystemStateManager;
using namespace std::chrono_literals;

// 船体首尾轨迹存储结构
struct ShipPosRecord {
    uint64_t ts;
    Eigen::Vector3f bow;
    Eigen::Vector3f stern;
};

// 解算后待MQTT推送完整数据包（高优先级）
struct MqttShipData {
    std::string berth_id;
    std::string lidar_ip;
    std::string time_str;

    float surge;
    float sway;
    float heave;
    float roll;
    float pitch;
    float yaw;

    float bX, bY, bZ;
    float sX, sY, sZ;
};

// 待入库数据包
struct DbShipData {
    SixDofResult res;
};

// 设备错误上报包（低优先级，队列溢出优先丢弃）
struct DeviceErrMsg {
    std::string ip;
    std::string time_str;
    int err_code;
};

// 队列阈值配置常量
namespace QueueLimit {
    // 数据库任务队列最大长度
    constexpr size_t DB_MAX_QUEUE = 200;
    // 船体解算任务队列最大长度
    constexpr size_t SOLVE_MAX_QUEUE = 300;
    // MQTT业务数据队列最大长度
    constexpr size_t MQTT_DATA_MAX_QUEUE = 500;
    // MQTT错误告警队列最大长度（更小，更容易丢）
    constexpr size_t MQTT_ERR_MAX_QUEUE = 100;
}

// 基类输出器
class DataOutputter {
public:
    explicit DataOutputter(LockFreeRingQueue<SixDofResult>* q3);
    virtual ~DataOutputter() = default;

    void start();
    void stop();

protected:
    // 改为纯虚，由子类实现，消除父类访问子类私有成员问题
    virtual void outputLoop() = 0;
    LockFreeRingQueue<SixDofResult>* queue3_ = nullptr;
    std::thread thread_;
    std::atomic<bool> is_running_{false};
};

// 具体业务输出实现（解算、数据库、MQTT分离线程）
class ConcreteDataOutputter : public DataOutputter {
public:
    explicit ConcreteDataOutputter(LockFreeRingQueue<SixDofResult>* q3);
    ~ConcreteDataOutputter() override;

    // MQTT连接回调
    void connCallback(bool connected);

    // 船体首尾位置计算 + 存入历史
    void calcShipBowSternPos(const SixDofResult& r);
    // 差分计算首尾加速度 cm/s²
    void getBowSternAcc(float& bX, float& bY, float& bZ, float& sX, float& sY, float& sZ);
    // 欧拉XYZ转旋转矩阵
    Eigen::Matrix3f eulerXYZToRot(float rx, float ry, float rz);
    // 插入历史轨迹缓存
    void pushHistory(uint64_t ts, const Eigen::Vector3f& bow, const Eigen::Vector3f& stern);

    // 原始同步接口（内部改为仅投递任务不阻塞）
    bool writeToDatabase(const SixDofResult& r);
    bool pushRealTimeData(const SixDofResult& r);
    bool pushRealTimeError(const SystemError& err);
    bool writeErrorToDatabase(const SystemError& err);

protected:
    // 子类重写输出主循环
    void outputLoop() override;

private:
    // 三个独立工作线程
    void dbWorkerLoop();
    void solveWorkerLoop();
    void mqttWorkerLoop();

    // 工具：队列溢出丢弃头部旧数据
    template<typename T>
    bool enqueueWithDropOld(LockFreeRingQueue<T>& queue, const T& item, size_t max_size, const std::string& queue_name, bool is_low_prio = false);

    // 错误处理工具函数
    std::string ExtractIpByRegex(const std::string& msg);

private:
    // 配置 & MQTT客户端
    RadarGlobalConfig monitor_cfg_;
    IndustrialMqttClient mqtt_;

    // 轨迹历史缓存
    static constexpr size_t MAX_HIST = 10;
    std::mutex hist_mtx_;
    std::deque<ShipPosRecord> pos_history_;

    // 分离线程无锁任务队列：构造时传入容量，修复无默认构造报错
    LockFreeRingQueue<DbShipData> db_task_queue_;
    LockFreeRingQueue<SixDofResult> solve_task_queue_;
    LockFreeRingQueue<MqttShipData> mqtt_task_queue_;
    LockFreeRingQueue<DeviceErrMsg> err_task_queue_;

    // 工作线程句柄
    std::thread db_thread_;
    std::thread solve_thread_;
    std::thread mqtt_thread_;

    // 线程唤醒条件变量（子类私有，仅子类内部访问）
    std::mutex task_mtx_;
    std::condition_variable task_cv_;
    std::atomic<bool> worker_running_{false};
};

#endif // DATA_OUTPUTTER_H