#ifndef INDUSTRIAL_MQTT_CLIENT_H
#define INDUSTRIAL_MQTT_CLIENT_H

#include "hv/hv.h"
#include "hv/mqtt_client.h"
#include "hv/json.hpp"
#include <string>
#include <mutex>
#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <thread>   // 新增：线程头文件

// MQTT 配置结构体
struct MqttConfig {
    std::string host = "127.0.0.1";
    int port = 1883;
    std::string username = "guest";
    std::string password = "guest";
    std::string client_id = "";
    std::string default_topic = "/industrial/data";
    int keepalive = 30;
    int qos = 1;
    bool enable_ssl = false;
    int reconnect_interval = 5;
    int max_reconnect_attempts = -1; // -1 无限重连
};

class IndustrialMqttClient {
public:
    using ConnectionStatusCallback = std::function<void(bool connected)>;

    IndustrialMqttClient();
    ~IndustrialMqttClient();

    bool init(const MqttConfig& config);
    bool start();
    void stop();

    int publish(const std::string& payload, const std::string& topic = "");

    std::string buildJson(const std::string berthId, const std::string& deviceIp, const std::string& timestamp,
		float shipSway, float shipSurge, float shipHeave,
		float shipRoll, float shipPitch, float shipYaw,
		float bowX, float bowY, float bowZ,
		float sternX, float sternY, float sternZ);

    std::string buildDeviceStatusJson(const std::string& deviceIp,
                                      const std::string& timestamp,
                                      int deviceStatus);

    void setConnectionStatusCallback(const ConnectionStatusCallback& callback);
    bool isConnected() const { return is_connected_.load(); }

private:
    static void onMqttEvent(mqtt_client_t* cli, int type);
    int publishInternal(const char* payload, const std::string& topic);
    void reconnect();
    std::string generateClientId();

    // 新增：MQTT 事件循环线程函数
    void mqttWorkThread();

private:
    MqttConfig config_;
    mqtt_client_t* mqtt_cli_ = nullptr;
    std::mutex mqtt_mutex_;

    std::atomic<bool> is_connected_  {false};
    std::atomic<bool> is_running_     {false};
    std::atomic<bool> thread_exit_    {false}; // 线程退出标记
    std::atomic<int>  reconnect_attempts_ {0};

    std::thread mqtt_thread_;  // 新增：独立工作线程

    ConnectionStatusCallback conn_callback_;
};

#endif // INDUSTRIAL_MQTT_CLIENT_H
