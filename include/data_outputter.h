#include "industrial_mqtt_client.h"
#include "system_state_manager.h"
#include "lock_free_queue.h"
#include "six_dof_calculator.h"
#include <thread>
#include <atomic>
#include <string>
#include <chrono>

using SM = SystemStateManager;
using namespace std::chrono_literals;

struct SixDofResult;
struct SystemError;

class DataOutputter
{
public:
    explicit DataOutputter(LockFreeRingQueue<SixDofResult>* q3);
    virtual ~DataOutputter() = default;

    void start();
    void stop();

protected:
    virtual bool writeToDatabase(const SixDofResult& r) = 0;
    virtual bool pushRealTimeData(const SixDofResult& r, const std::string& url) = 0;
    std::thread thread_;
    std::atomic<bool> is_running_{false};
    LockFreeRingQueue<SixDofResult>* queue3_ = nullptr;

private:
    void outputLoop();
};

class ConcreteDataOutputter : public DataOutputter
{
public:
    explicit ConcreteDataOutputter(LockFreeRingQueue<SixDofResult>* q3);
    ~ConcreteDataOutputter() override;

protected:
    bool writeToDatabase(const SixDofResult& r) override;
    bool pushRealTimeData(const SixDofResult& r, const std::string& url) override;

private:
    bool writeErrorToDatabase(const SystemError& err);
    bool pushRealTimeError(const SystemError& err, const std::string& url);

    // MQTT 相关回调静态函数
    static void logCallback(MqttLogLevel level, const std::string& msg);
    static void connCallback(bool connected);

    // MQTT 实例
    IndustrialMqttClient mqtt_;
    // 设备/泊位固定配置
    std::string berth_id_ = "2056304240267644930";
    std::string dev_ip_list_ = "192.168.0.200,192.168.0.201,192.168.0.202,192.168.0.203";
    // 设备状态上报计时
    std::chrono::steady_clock::time_point last_dev_status_tp_;
    const std::chrono::seconds dev_status_interval_{3};
};