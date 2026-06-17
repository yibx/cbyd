#include "industrial_mqtt_client.h"
#include <sstream>
#include <iomanip>
#include <random>
#include <thread>
#include <cstring>

using namespace nlohmann;

static std::string formatFloat(float value, int precision = 3) {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(precision) << value;
    return ss.str();
}

IndustrialMqttClient::IndustrialMqttClient()
{
    srand(static_cast<unsigned int>(time(nullptr)));
}

IndustrialMqttClient::~IndustrialMqttClient()
{
    stop();
    if (mqtt_thread_.joinable())
    {
        mqtt_thread_.join();
    }
}

bool IndustrialMqttClient::init(const MqttConfig& config)
{
    if (is_running_.load())
    {
        log(MqttLogLevel::LOG_WARN, "MQTT client already running");
        return false;
    }
    config_ = config;
    if (config_.client_id.empty())
    {
        config_.client_id = generateClientId();
    }
    log(MqttLogLevel::LOG_INFO, "MQTT init success, client_id: " + config_.client_id);
    return true;
}

// 启动：创建子线程运行 mqtt_client_run，不再阻塞主线程
bool IndustrialMqttClient::start()
{
    std::lock_guard<std::mutex> lock(mqtt_mutex_);
    if (is_running_.load())
    {
        log(MqttLogLevel::LOG_WARN, "MQTT already started");
        return true;
    }

    thread_exit_ = false;
    is_running_  = true;
    reconnect_attempts_ = 0;

    // 启动独立工作线程，事件循环在线程内执行
    if (mqtt_thread_.joinable())
    {
        mqtt_thread_.join();
    }
    mqtt_thread_ = std::thread(&IndustrialMqttClient::mqttWorkThread, this);

    log(MqttLogLevel::LOG_INFO, "MQTT work thread started, no longer block main thread");
    return true;
}

// 停止客户端 + 退出工作线程
void IndustrialMqttClient::stop()
{
    std::lock_guard<std::mutex> lock(mqtt_mutex_);
    if (!is_running_.load())
        return;

    thread_exit_ = true;
    is_running_  = false;

    // 断开 MQTT 连接
    if (mqtt_cli_ != nullptr)
    {
        mqtt_client_disconnect(mqtt_cli_);
        mqtt_client_stop(mqtt_cli_);
        mqtt_client_free(mqtt_cli_);
        mqtt_cli_ = nullptr;
    }

    is_connected_ = false;
    log(MqttLogLevel::LOG_INFO, "MQTT client stopped");

    if (conn_callback_)
    {
        conn_callback_(false);
    }
}

// 【核心】MQTT 独立工作线程：在这里执行阻塞的 mqtt_client_run
void IndustrialMqttClient::mqttWorkThread()
{
    while (!thread_exit_.load())
    {
        {
            std::lock_guard<std::mutex> lock(mqtt_mutex_);
            // 重建客户端
            mqtt_cli_ = mqtt_client_new(nullptr);
            if (!mqtt_cli_)
            {
                log(MqttLogLevel::LOG_ERROR, "Create mqtt client failed");
                std::this_thread::sleep_for(std::chrono::seconds(config_.reconnect_interval));
                continue;
            }

            // 配置客户端参数
            mqtt_client_set_id(mqtt_cli_, config_.client_id.c_str());
            mqtt_cli_->keepalive = config_.keepalive;
            mqtt_client_set_auth(mqtt_cli_, config_.username.c_str(), config_.password.c_str());

            // 遗嘱消息
            mqtt_message_t will_msg{};
            will_msg.topic  = (config_.default_topic + "/will").c_str();
            will_msg.payload = "Client offline";
            will_msg.qos = config_.qos;
            mqtt_client_set_will(mqtt_cli_, &will_msg);

            mqtt_client_set_callback(mqtt_cli_, onMqttEvent);
            mqtt_client_set_userdata(mqtt_cli_, this);
        }

        // 连接 RabbitMQ MQTT
        int ret = mqtt_client_connect(mqtt_cli_, config_.host.c_str(), config_.port, config_.enable_ssl);
        if (ret != 0)
        {
            log(MqttLogLevel::LOG_ERROR, "Connect broker failed, ret=" + std::to_string(ret));
            {
                std::lock_guard<std::mutex> lock(mqtt_mutex_);
                mqtt_client_free(mqtt_cli_);
                mqtt_cli_ = nullptr;
            }
            std::this_thread::sleep_for(std::chrono::seconds(config_.reconnect_interval));
            continue;
        }

        // ========== 此处会阻塞，但现在在子线程，不影响主线程 ==========
        mqtt_client_run(mqtt_cli_);

        // 循环到这里代表连接断开，释放资源，准备重连
        {
            std::lock_guard<std::mutex> lock(mqtt_mutex_);
            if (mqtt_cli_)
            {
                mqtt_client_free(mqtt_cli_);
                mqtt_cli_ = nullptr;
            }
        }

        is_connected_ = false;
        // 检查是否退出线程
        if (thread_exit_.load())
            break;

        // 执行重连等待
        reconnect();
    }
    log(MqttLogLevel::LOG_INFO, "MQTT work thread exit");
}

int IndustrialMqttClient::publish(const std::string& payload, const std::string& topic)
{
    if (!is_connected_.load())
    {
        log(MqttLogLevel::LOG_ERROR, "Publish failed: not connected");
        return -1;
    }
    std::string target_topic = topic.empty() ? config_.default_topic : topic;
    return publishInternal(payload.c_str(), target_topic);
}

int IndustrialMqttClient::publishInternal(const char* payload, const std::string& topic)
{
    std::lock_guard<std::mutex> lock(mqtt_mutex_);
    if (!mqtt_cli_)
    {
        log(MqttLogLevel::LOG_ERROR, "MQTT client handle null");
        return -2;
    }

    mqtt_message_t msg{};
    msg.topic       = topic.c_str();
    msg.topic_len   = topic.size();
    msg.payload     = payload;
    msg.payload_len = strlen(payload);
    msg.qos         = config_.qos;
    msg.retain      = 0;

    int mid = mqtt_client_publish(mqtt_cli_, &msg);
    if (mid <= 0)
    {
        log(MqttLogLevel::LOG_ERROR, "Publish fail, mid=" + std::to_string(mid));
        return mid;
    }
    log(MqttLogLevel::LOG_DEBUG, "Publish ok, topic=" + topic);
    return 0;
}

void IndustrialMqttClient::onMqttEvent(mqtt_client_t* cli, int type)
{
    IndustrialMqttClient* client = static_cast<IndustrialMqttClient*>(mqtt_client_get_userdata(cli));
    if (!client) return;

    switch (type)
    {
        case MQTT_TYPE_CONNACK:
            client->is_connected_ = true;
            client->reconnect_attempts_ = 0;
            client->log(MqttLogLevel::LOG_INFO, "MQTT connected to RabbitMQ");
            if (client->conn_callback_) client->conn_callback_(true);
            break;
        case MQTT_TYPE_DISCONNECT:
            client->is_connected_ = false;
            client->log(MqttLogLevel::LOG_WARN, "MQTT disconnected");
            if (client->conn_callback_) client->conn_callback_(false);
            break;
        case MQTT_TYPE_PUBACK:
            client->log(MqttLogLevel::LOG_DEBUG, "Recv PUBACK");
            break;
        default:
            break;
    }
}

void IndustrialMqttClient::reconnect()
{
    if (config_.max_reconnect_attempts > 0 && reconnect_attempts_ >= config_.max_reconnect_attempts)
    {
        log(MqttLogLevel::LOG_ERROR, "Reach max reconnect times, stop reconnect");
        stop();
        return;
    }
    reconnect_attempts_++;
    log(MqttLogLevel::LOG_WARN, "Reconnect count: " + std::to_string(reconnect_attempts_));
}

std::string IndustrialMqttClient::buildJson(const std::string berthId, const std::string& deviceIp, 
		const std::string& timestamp,
		float shipSway, float shipSurge, float shipHeave,
		float shipRoll, float shipPitch, float shipYaw, 
		float bowX, float bowY, float bowZ,
		float sternX, float sternY, float sternZ)
{
    json j;

    j["berthId"] = berthId;
    j["deviceIp"] = deviceIp;
    j["timestamp"] = timestamp;

    j["shipSway"]  = std::stod(formatFloat(shipSway, 2));
    j["shipSurge"]  = std::stod(formatFloat(shipSurge, 2));
    j["shipHeave"]   = std::stod(formatFloat(shipHeave, 2));
    j["shipRoll"]    = std::stod(formatFloat(shipRoll, 2));
    j["shipPitch"]   = std::stod(formatFloat(shipPitch, 2));
    j["shipYaw"]    = std::stod(formatFloat(shipYaw, 2));

    j["bowAcceleration"]    = {
    	std::stod(formatFloat(bowX, 2)),
	std::stod(formatFloat(bowY, 2)),
	std::stod(formatFloat(bowZ, 2))
    };

    j["sternAcceleration"]    = {
        std::stod(formatFloat(sternX, 2)),
        std::stod(formatFloat(sternY, 2)),
        std::stod(formatFloat(sternZ, 2))
    };

    return j.dump(4);
}

std::string IndustrialMqttClient::buildDeviceStatusJson(const std::string& deviceIp,
                                                        const std::string& timestamp,
                                                        int deviceStatus)
{
    json j;
    j["deviceIp"]     = deviceIp;
    j["deviceStatus"] = deviceStatus;
    j["timestamp"]    = timestamp;

    return j.dump(4);
}

void IndustrialMqttClient::setLogCallback(const LogCallback& callback)
{
    log_callback_ = callback;
}

void IndustrialMqttClient::setConnectionStatusCallback(const ConnectionStatusCallback& callback)
{
    conn_callback_ = callback;
}

void IndustrialMqttClient::log(MqttLogLevel level, const std::string& msg)
{
    if (log_callback_)
    {
        log_callback_(level, msg);
        return;
    }
    const char* lv = "";
    switch (level)
    {
        case MqttLogLevel::LOG_DEBUG: lv = "[DEBUG]"; break;
        case MqttLogLevel::LOG_INFO:  lv = "[INFO]";  break;
        case MqttLogLevel::LOG_WARN:  lv = "[WARN]";  break;
        case MqttLogLevel::LOG_ERROR: lv = "[ERROR]"; break;
    }
    printf("%s %s\n", lv, msg.c_str());
}

std::string IndustrialMqttClient::generateClientId()
{
    std::stringstream ss;
    ss << "mqtt_rabbit_" << hv_getpid() << "_" << (rand() % 1000000);
    return ss.str();
}
