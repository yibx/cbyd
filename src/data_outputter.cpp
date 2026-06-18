#include "data_outputter.h"
#include <iostream>
#include <iomanip>
#include <sstream>
#include "Logger.h"

DataOutputter::DataOutputter(LockFreeRingQueue<SixDofResult>* q3)
    : queue3_(q3) {
}

void DataOutputter::start() {
    if (is_running_) return;
    SM::instance().setState(SystemState::RUNNING);
    is_running_ = true;
    thread_ = std::thread(&DataOutputter::outputLoop, this);
}

void DataOutputter::stop() {
    is_running_ = false;
    if (thread_.joinable())
        thread_.join();
    SM::instance().setState(SystemState::IDLE);
}

void DataOutputter::outputLoop() {
    while (is_running_) {
        if (SM::instance().getState() == SystemState::FATAL_ERROR) {
            Logger::instance().info("[Outputter] 系统致命错误，退出输出线程");
            break;
        }

        SixDofResult r;
        if (queue3_->dequeue(r)) {
            bool db_ok = writeToDatabase(r);
            bool rt_ok = pushRealTimeData(r);

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
        } else {
            std::this_thread::sleep_for(100ms);
        }
    }
    SM::instance().setState(SystemState::IDLE);
}

// MQTT 连接状态回调
void ConcreteDataOutputter::connCallback(bool connected) {
    if (!connected) {
        SM::instance().reportError(
            ModuleType::OUTPUTTER,
            ErrorLevel::STATUS_WARNING,
            "MQTT 连接断开，自动重连中"
        );
        Logger::instance().info("[MQTT CONN] MQTT 已断开，自动重连中");
    } else {
        Logger::instance().info("[MQTT CONN] MQTT 已连接");
    }
}

ConcreteDataOutputter::ConcreteDataOutputter(LockFreeRingQueue<SixDofResult>* q3)
    : DataOutputter(q3) {
    // 注册全局错误回调
    SM::instance().setErrorPushCallback([this](const SystemError& err) {
        return pushRealTimeError(err, "ws://127.0.0.1:8080/error");
    });
    SM::instance().setDatabaseWriteCallback([this](const SystemError& err) {
        return writeErrorToDatabase(err);
    });

    if (!loadMonitorConfig("../monitor_config.yaml", monitor_cfg_)) {
        std::string err_msg = "监测配置加载失败，请检查monitor_config.yaml文件";
        Logger::instance().info(err_msg);
        return;
    }

    // MQTT 配置初始化
    MqttConfig cfg;
    cfg.host         = monitor_cfg_.mqtt_cfg.host;
    cfg.port         = monitor_cfg_.mqtt_cfg.port;
    cfg.username     = monitor_cfg_.mqtt_cfg.username;
    cfg.password     = monitor_cfg_.mqtt_cfg.password;
    cfg.default_topic= monitor_cfg_.mqtt_cfg.default_topic;
    cfg.qos          = monitor_cfg_.mqtt_cfg.qos;
    cfg.reconnect_interval = monitor_cfg_.mqtt_cfg.reconnect_interval;

    mqtt_.setConnectionStatusCallback(connCallback);

    if (!mqtt_.init(cfg)) {
        Logger::instance().error("[Outputter FATAL] MQTT 客户端初始化失败");
        SM::instance().reportError(
            ModuleType::OUTPUTTER,
            ErrorLevel::STATUS_ERROR,
            "MQTT 客户端初始化失败"
        );
    } else {
        mqtt_.start();
    }
}

ConcreteDataOutputter::~ConcreteDataOutputter() {
    mqtt_.stop();
}

bool ConcreteDataOutputter::writeToDatabase(const SixDofResult& r) {
    try
    {
        // 保留接口，实际数据库写入逻辑根据项目需求实现
        return true;
    }
    catch (...)
    {
        return false;
    }
}

bool ConcreteDataOutputter::pushRealTimeData(const SixDofResult& r) {
    try
    {
        // MQTT 上报船舶六自由度数据
        if (!mqtt_.isConnected())
        {
            return false;
        }

        // 构造时间字符串
        std::time_t now_t = r.timestamp / 1000;
        std::tm tm_now = *std::localtime(&now_t);
        std::ostringstream time_ss;
        time_ss << std::put_time(&tm_now, "%Y-%m-%d %H:%M:%S");
        std::string time_str = time_ss.str();

        // 六自由度平移旋转单位说明：r.tx/ty/tz 厘米，内部转米
        float surge = r.tx;
        float sway  = r.ty;
        float heave = r.tz;
        float roll  = r.rx;
        float pitch = r.ry;
        float yaw   = r.rz;

        // 包围盒基准偏移（固定占位，无额外包围盒时填0）
        float bX = 0.0f, bY = 0.0f, bZ = 0.0f;
        float sX = 0.0f, sY = 0.0f, sZ = 0.0f;

        std::string json = mqtt_.buildJson(
            monitor_cfg_.mqtt_cfg.berth_id, r.lidar_ip, time_str,
            sway, surge, heave,
            roll, pitch, yaw,
            bX, bY, bZ,
            sX, sY, sZ
        );
        //std::cout << "[MQTT] publish six-dof json: " << json << std::endl;
        mqtt_.publish(json);

        return true;
    }
    catch (...)
    {
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
    
    // MQTT 上报船舶六自由度数据
    if (!mqtt_.isConnected())
    {
        return false;
    }
    // 获取系统当前本地时间
    auto now = std::chrono::system_clock::now();
    std::time_t now_t = std::chrono::system_clock::to_time_t(now);
    std::tm tm_now = *std::localtime(&now_t);

    std::ostringstream time_ss;
    time_ss << std::put_time(&tm_now, "%Y-%m-%d %H:%M:%S");
    std::string time_str = time_ss.str();
    std::string statusJson = mqtt_.buildDeviceStatusJson(monitor_cfg_.mqtt_cfg.host, time_str, 1);
    std::cout << "[MQTT] publish device status: " << statusJson << std::endl;
    mqtt_.publish(statusJson, monitor_cfg_.mqtt_cfg.dev_status_topic);
    return true;
}