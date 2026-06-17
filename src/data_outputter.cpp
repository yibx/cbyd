#include "data_outputter.h"
#include <iostream>
#include <iomanip>
#include <sstream>

DataOutputter::DataOutputter(LockFreeRingQueue<SixDofResult>* q3)
    : queue3_(q3)
{
}

void DataOutputter::start()
{
    if (is_running_) return;
    SM::instance().setState(SystemState::RUNNING);
    is_running_ = true;
    thread_ = std::thread(&DataOutputter::outputLoop, this);
}

void DataOutputter::stop()
{
    is_running_ = false;
    if (thread_.joinable())
        thread_.join();
    SM::instance().setState(SystemState::IDLE);
}

void DataOutputter::outputLoop()
{
    while (is_running_)
    {
        if (SM::instance().getState() == SystemState::FATAL_ERROR)
        {
            std::cerr << "[Outputter] 系统致命错误，退出输出线程" << std::endl;
            break;
        }

        SixDofResult r;
        if (queue3_->dequeue(r))
        {
            bool db_ok = writeToDatabase(r);
            bool rt_ok = pushRealTimeData(r, "ws://127.0.0.1:8080/rt");

            if (!db_ok)
            {
                SM::instance().reportError(
                    ModuleType::OUTPUTTER,
                    ErrorLevel::STATUS_ERROR,
                    "数据库写入失败"
                );
            }
            if (!rt_ok)
            {
                SM::instance().reportError(
                    ModuleType::OUTPUTTER,
                    ErrorLevel::STATUS_WARNING,
                    "实时数据推送失败"
                );
            }
        }
        else
        {
            std::this_thread::sleep_for(100ms);
        }
    }
    SM::instance().setState(SystemState::IDLE);
}

// MQTT 静态日志回调
void ConcreteDataOutputter::logCallback(MqttLogLevel level, const std::string& msg)
{
    const char* lv = "";
    switch (level)
    {
        case MqttLogLevel::LOG_DEBUG: lv = "[MQTT DEBUG]"; break;
        case MqttLogLevel::LOG_INFO:  lv = "[MQTT INFO]";  break;
        case MqttLogLevel::LOG_WARN:  lv = "[MQTT WARN]";  break;
        case MqttLogLevel::LOG_ERROR: lv = "[MQTT ERROR]"; break;
    }
    std::cout << lv << " " << msg << std::endl;
}

// MQTT 连接状态回调
void ConcreteDataOutputter::connCallback(bool connected)
{
    if (!connected)
    {
        SM::instance().reportError(
            ModuleType::OUTPUTTER,
            ErrorLevel::STATUS_WARNING,
            "MQTT 连接断开，自动重连中"
        );
        std::cout << "[MQTT CONN] MQTT 已断开" << std::endl;
    }
    else
    {
        std::cout << "[MQTT CONN] MQTT 已连接" << std::endl;
    }
}

ConcreteDataOutputter::ConcreteDataOutputter(LockFreeRingQueue<SixDofResult>* q3)
    : DataOutputter(q3)
{
    // 注册全局错误回调
    SM::instance().setErrorPushCallback([this](const SystemError& err) {
        return pushRealTimeError(err, "ws://127.0.0.1:8080/error");
    });
    SM::instance().setDatabaseWriteCallback([this](const SystemError& err) {
        return writeErrorToDatabase(err);
    });

    // MQTT 配置初始化
    MqttConfig cfg;
    cfg.host         = "192.168.0.31";
    cfg.port         = 1883;
    cfg.username     = "rabbitMq";
    cfg.password     = "huazhi2026";
    cfg.default_topic= "/ship/data";
    cfg.qos          = 1;
    cfg.reconnect_interval = 3;

    mqtt_.setLogCallback(logCallback);
    mqtt_.setConnectionStatusCallback(connCallback);

    if (!mqtt_.init(cfg))
    {
        std::cerr << "[Outputter FATAL] MQTT 客户端初始化失败" << std::endl;
        SM::instance().reportError(
            ModuleType::OUTPUTTER,
            ErrorLevel::STATUS_ERROR,
            "MQTT客户端初始化失败，数据无法上报"
        );
    }
    else
    {
        mqtt_.start();
    }

    last_dev_status_tp_ = std::chrono::steady_clock::now();
}

ConcreteDataOutputter::~ConcreteDataOutputter()
{
    mqtt_.stop();
}

bool ConcreteDataOutputter::writeToDatabase(const SixDofResult& r)
{
    try
    {
        std::cout << "[DB] 写入位姿结果: t=" << std::dec << std::setw(19) << r.timestamp << std::endl;
        std::cout << " tx=" << r.tx << " ty=" << r.ty << " tz=" << r.tz
                  << " rx=" << r.rx << " ry=" << r.ry << " rz=" << r.rz << std::endl;
        return true;
    }
    catch (...)
    {
        return false;
    }
}

bool ConcreteDataOutputter::pushRealTimeData(const SixDofResult& r, const std::string& url)
{
    try
    {
        // 原有Websocket打印逻辑保留
        std::cout << "[RT-WS] 推送位姿: " << url
            << " rx=" << r.rx << " ry=" << r.ry << " tz=" << r.tz
            << " rx=" << r.rx << " ry=" << r.ry << " rz=" << r.rz << std::endl;

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
        float surge = r.tx / 100.0f;
        float sway  = r.ty / 100.0f;
        float heave = r.tz / 100.0f;
        float roll  = r.rx;
        float pitch = r.ry;
        float yaw   = r.rz;

        // 包围盒基准偏移（固定占位，无额外包围盒时填0）
        float bX = 0.0f, bY = 0.0f, bZ = 0.0f;
        float sX = 0.0f, sY = 0.0f, sZ = 0.0f;

        std::string json = mqtt_.buildJson(
            berth_id_, dev_ip_list_, time_str,
            sway, surge, heave,
            roll, pitch, yaw,
            bX, bY, bZ,
            sX, sY, sZ
        );
        std::cout << "[MQTT] publish six-dof json: " << json << std::endl;
        mqtt_.publish(json);

        // 定时上报设备状态
        auto now_tp = std::chrono::steady_clock::now();
        auto delta = now_tp - last_dev_status_tp_;
        if (delta >= dev_status_interval_)
        {
            last_dev_status_tp_ = now_tp;
            std::string dev_time = time_str;
            int dev_status = 1;
            std::string statusJson = mqtt_.buildDeviceStatusJson("192.168.0.200", dev_time, dev_status);
            std::cout << "[MQTT] publish device status: " << statusJson << std::endl;
            mqtt_.publish(statusJson, "/dev/status");
        }

        return true;
    }
    catch (...)
    {
        return false;
    }
}

bool ConcreteDataOutputter::writeErrorToDatabase(const SystemError& err)
{
    std::cout << "[DB-ERROR] 模块:" << static_cast<int>(err.module)
        << " 等级:" << static_cast<int>(err.level)
        << " 信息:" << err.message << std::endl;
    return true;
}

bool ConcreteDataOutputter::pushRealTimeError(const SystemError& err, const std::string& url)
{
    std::cout << "[RT-ERROR] " << url
        << " 错误: " << err.message << std::endl;
    return true;
}