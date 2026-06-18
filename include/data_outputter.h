#include "industrial_mqtt_client.h"
#include "system_state_manager.h"
#include "lock_free_queue.h"
#include "six_dof_calculator.h"
#include <thread>
#include <atomic>
#include <string>
#include <chrono>
#include "config_loader.h"

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
    virtual bool pushRealTimeData(const SixDofResult& r) = 0;
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
    bool pushRealTimeData(const SixDofResult& r) override;

private:
    bool writeErrorToDatabase(const SystemError& err);
    bool pushRealTimeError(const SystemError& err, const std::string& url);
    
    static void connCallback(bool connected);

    // MQTT 实例
    IndustrialMqttClient mqtt_;
    RadarGlobalConfig monitor_cfg_;
};