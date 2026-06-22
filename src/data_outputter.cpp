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

void ConcreteDataOutputter::pushHistory(uint64_t ts, const Eigen::Vector3f& bow, const Eigen::Vector3f& stern) {
    std::lock_guard<std::mutex> lock(hist_mtx_);
    ShipPosRecord rec;
    rec.ts = ts;
    rec.bow = bow;
    rec.stern = stern;
    pos_history_.push_back(rec);
    while (pos_history_.size() > MAX_HIST)
    {
        pos_history_.pop_front();
    }
}

Eigen::Matrix3f ConcreteDataOutputter::eulerXYZToRot(float rx, float ry, float rz) {
    Eigen::AngleAxisf ax(rx, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf ay(ry, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf az(rz, Eigen::Vector3f::UnitZ());
    return (az * ay * ax).matrix();
}

void ConcreteDataOutputter::calcShipBowSternPos(const SixDofResult& r) {
    std::cout << "run calcShipBowSternPos"  << std::endl;
    // 1. cm 转 m：质心全局平移
    Eigen::Vector3f t_center(r.tx / 100.0f, r.ty / 100.0f, r.tz / 100.0f);
    // 2. 欧拉角构造旋转矩阵
    Eigen::Matrix3f R = eulerXYZToRot(r.rx, r.ry, r.rz);
    // 3. 船体局部艏尾点
    float L = static_cast<float>(r.ship_length);
    Eigen::Vector3f P_bow_body(L / 2.0f, 0.0f, 0.0f);
    Eigen::Vector3f P_stern_body(-L / 2.0f, 0.0f, 0.0f);
    // 4. 刚体变换得到码头全局坐标
    Eigen::Vector3f bow_global = R * P_bow_body + t_center;
    Eigen::Vector3f stern_global = R * P_stern_body + t_center;
    // 5. 存入历史队列
    pushHistory(r.timestamp, bow_global, stern_global);
}

// 输出参数 bX,bY,bZ 船艏加速度; sX,sY,sZ 船尾加速度，单位 cm/s²
void ConcreteDataOutputter::getBowSternAcc(float& bX, float& bY, float& bZ, float& sX, float& sY, float& sZ) {
    std::cout << "run getBowSternAcc"  << std::endl;
    // 默认置0
    bX = bY = bZ = 0.0f;
    sX = sY = sZ = 0.0f;

    std::lock_guard<std::mutex> lock(hist_mtx_);
    // 不足3帧无法二阶差分
    if (pos_history_.size() < 3)
        return;

    // k-2, k-1, k
    const auto& h0 = pos_history_[0];
    const auto& h1 = pos_history_[1];
    const auto& h2 = pos_history_[2];

    // 时间间隔 秒
    double dt_ms = static_cast<double>(h2.ts - h1.ts);
    double dt = dt_ms / 1000.0;
    double dt_sq = dt * dt;

    if (dt_sq < 1e-6)
        return;

    // 二阶差分得到 m/s²
    Eigen::Vector3f a_bow_m = (h2.bow - 2.0f * h1.bow + h0.bow) / dt_sq;
    Eigen::Vector3f a_stern_m = (h2.stern - 2.0f * h1.stern + h0.stern) / dt_sq;

    // m/s² 转 cm/s² × 100
    bX = a_bow_m.x() * 100.0f;
    bY = a_bow_m.y() * 100.0f;
    bZ = a_bow_m.z() * 100.0f;

    sX = a_stern_m.x() * 100.0f;
    sY = a_stern_m.y() * 100.0f;
    sZ = a_stern_m.z() * 100.0f;
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

        // 传入当前六自由度结果 r
        calcShipBowSternPos(r);
        float bX = 0.0f, bY = 0.0f, bZ = 0.0f;
        float sX = 0.0f, sY = 0.0f, sZ = 0.0f;
        getBowSternAcc(bX, bY, bZ, sX, sY, sZ);

        std::string json = mqtt_.buildJson(
            monitor_cfg_.mqtt_cfg.berth_id, r.lidar_ip, time_str,
            sway, surge, heave,
            roll, pitch, yaw,
            bX, bY, bZ,
            sX, sY, sZ
        );
        std::cout << "[MQTT] publish six-dof json: " << json << std::endl;
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