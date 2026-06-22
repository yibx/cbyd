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

const int MAX_HIST = 3;

struct ShipPosRecord
{
    uint64_t ts;
    Eigen::Vector3f bow;    // 船艏全局坐标 m
    Eigen::Vector3f stern;  // 船尾全局坐标 m
};

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

    Eigen::Matrix3f eulerXYZToRot(float rx, float ry, float rz);

    void pushHistory(uint64_t ts, const Eigen::Vector3f& bow, const Eigen::Vector3f& stern);

    void calcShipBowSternPos(const SixDofResult& r);

    void getBowSternAcc(float& bX, float& bY, float& bZ, float& sX, float& sY, float& sZ);

    // MQTT 实例
    IndustrialMqttClient mqtt_;
    RadarGlobalConfig monitor_cfg_;

    std::deque<ShipPosRecord> pos_history_;
    std::mutex hist_mtx_;
};