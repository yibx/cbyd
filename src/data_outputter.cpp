#include "data_outputter.h"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <cstring>

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

// 初始化列表给所有无锁队列传入对应容量，修复缺少构造参数报错
ConcreteDataOutputter::ConcreteDataOutputter(LockFreeRingQueue<SixDofResult>* q3)
    : DataOutputter(q3),
      db_task_queue_(QueueLimit::DB_MAX_QUEUE),
      solve_task_queue_(QueueLimit::SOLVE_MAX_QUEUE),
      mqtt_task_queue_(QueueLimit::MQTT_DATA_MAX_QUEUE),
      err_task_queue_(QueueLimit::MQTT_ERR_MAX_QUEUE)
{
    // 注册系统错误回调，仅投递错误任务不阻塞主线程
    SM::instance().setErrorPushCallback([this](const SystemError& err) {
        return pushRealTimeError(err);
    });
    SM::instance().setDatabaseWriteCallback([this](const SystemError& err) {
        return writeErrorToDatabase(err);
    });

    // 加载配置
    if (!loadMonitorConfig("../monitor_config.yaml", monitor_cfg_)) {
        std::string err_msg = "监测配置加载失败，请检查monitor_config.yaml文件";
        Logger::instance().error(err_msg);
        return;
    }

    // MQTT初始化
    MqttConfig cfg;
    cfg.host = monitor_cfg_.mqtt_cfg.host;
    cfg.port = monitor_cfg_.mqtt_cfg.port;
    cfg.username = monitor_cfg_.mqtt_cfg.username;
    cfg.password = monitor_cfg_.mqtt_cfg.password;
    cfg.default_topic = monitor_cfg_.mqtt_cfg.default_topic;
    cfg.qos = monitor_cfg_.mqtt_cfg.qos;
    cfg.reconnect_interval = monitor_cfg_.mqtt_cfg.reconnect_interval;

    mqtt_.setConnectionStatusCallback(std::bind(&ConcreteDataOutputter::connCallback, this, std::placeholders::_1));
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

    // 启动三组独立工作线程
    worker_running_ = true;
    db_thread_ = std::thread(&ConcreteDataOutputter::dbWorkerLoop, this);
    solve_thread_ = std::thread(&ConcreteDataOutputter::solveWorkerLoop, this);
    mqtt_thread_ = std::thread(&ConcreteDataOutputter::mqttWorkerLoop, this);
}

ConcreteDataOutputter::~ConcreteDataOutputter() {
    worker_running_ = false;
    // 唤醒所有等待线程
    task_cv_.notify_all();

    if (db_thread_.joinable()) db_thread_.join();
    if (solve_thread_.joinable()) solve_thread_.join();
    if (mqtt_thread_.joinable()) mqtt_thread_.join();

    mqtt_.stop();
}

void ConcreteDataOutputter::outputLoop() {
    while (is_running_) {
        try {
            if (SM::instance().getState() == SystemState::FATAL_ERROR) {
                Logger::instance().info("[Outputter] 系统致命错误，退出输出主线程");
                break;
            }

            SixDofResult r;
            // 无锁出队，不阻塞循环
            if (queue3_->dequeue(r)) {
                writeToDatabase(r);
                pushRealTimeData(r);
            } else {
                std::unique_lock<std::mutex> lk(task_mtx_);
                task_cv_.wait_for(lk, std::chrono::milliseconds(50));
            }
        } catch (...) {
            Logger::instance().error("[Outputter outputLoop 主线程捕获未知异常，继续运行");
        }
    }
}

void ConcreteDataOutputter::connCallback(bool connected) {
    try {
        if (!connected) {
            SM::instance().reportError(
                ModuleType::OUTPUTTER,
                ErrorLevel::STATUS_WARNING,
                "MQTT 连接断开，自动重连中"
            );
            Logger::instance().warn("[MQTT CONN] MQTT 已断开，自动重连中");
        } else {
            Logger::instance().info("[MQTT CONN] MQTT 已连接");
        }
    } catch (...) {
        Logger::instance().error("[MQTT connCallback 回调异常");
    }
}

Eigen::Matrix3f ConcreteDataOutputter::eulerXYZToRot(float rx, float ry, float rz) {
    Eigen::AngleAxisf ax(rx, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf ay(ry, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf az(rz, Eigen::Vector3f::UnitZ());
    return (az * ay * ax).matrix();
}

void ConcreteDataOutputter::pushHistory(uint64_t ts, const Eigen::Vector3f& bow, const Eigen::Vector3f& stern) {
    std::lock_guard<std::mutex> lock(hist_mtx_);
    ShipPosRecord rec;
    rec.ts = ts;
    rec.bow = bow;
    rec.stern = stern;
    pos_history_.push_back(rec);
    while (pos_history_.size() > MAX_HIST) {
        pos_history_.pop_front();
    }
}

void ConcreteDataOutputter::calcShipBowSternPos(const SixDofResult& r) {
    // cm 转 m：质心全局平移
    Eigen::Vector3f t_center(r.tx / 100.0f, r.ty / 100.0f, r.tz / 100.0f);
    // 欧拉角构造旋转矩阵
    Eigen::Matrix3f R = eulerXYZToRot(r.rx, r.ry, r.rz);
    // 船体局部艏尾点
    float L = static_cast<float>(r.ship_length);
    Eigen::Vector3f P_bow_body(L / 2.0f, 0.0f, 0.0f);
    Eigen::Vector3f P_stern_body(-L / 2.0f, 0.0f, 0.0f);
    // 刚体变换得到码头全局坐标
    Eigen::Vector3f bow_global = R * P_bow_body + t_center;
    Eigen::Vector3f stern_global = R * P_stern_body + t_center;
    // 存入历史队列
    pushHistory(r.timestamp, bow_global, stern_global);
}

void ConcreteDataOutputter::getBowSternAcc(float& bX, float& bY, float& bZ, float& sX, float& sY, float& sZ) {
    bX = bY = bZ = 0.0f;
    sX = sY = sZ = 0.0f;

    std::lock_guard<std::mutex> lock(hist_mtx_);
    if (pos_history_.size() < 3)
        return;

    const auto& h0 = pos_history_[0];
    const auto& h1 = pos_history_[1];
    const auto& h2 = pos_history_[2];

    double dt_ms = static_cast<double>(h2.ts - h1.ts);
    double dt = dt_ms / 1000.0;
    double dt_sq = dt * dt;

    if (dt_sq < 1e-6)
        return;

    Eigen::Vector3f a_bow_m = (h2.bow - 2.0f * h1.bow + h0.bow) / dt_sq;
    Eigen::Vector3f a_stern_m = (h2.stern - 2.0f * h1.stern + h0.stern) / dt_sq;

    bX = a_bow_m.x() * 100.0f;
    bY = a_bow_m.y() * 100.0f;
    bZ = a_bow_m.z() * 100.0f;

    sX = a_stern_m.x() * 100.0f;
    sY = a_stern_m.y() * 100.0f;
    sZ = a_stern_m.z() * 100.0f;
}

template<typename T>
bool ConcreteDataOutputter::enqueueWithDropOld(LockFreeRingQueue<T>& queue, const T& item, size_t max_size, const std::string& queue_name, bool is_low_prio)
{
    // 循环丢弃队首老旧数据，直到长度低于阈值
    size_t drop_cnt = 0;
    while (queue.size() >= max_size)
    {
        T trash;
        queue.dequeue(trash);
        drop_cnt++;
    }

    if (drop_cnt > 0)
    {
        std::string log = "队列[" + queue_name + "]溢出，丢弃老旧数据条数:" + std::to_string(drop_cnt);
        if (is_low_prio)
        {
            Logger::instance().warn(log + "（低优先级告警包）");
        }
        else
        {
            Logger::instance().error(log + "，存在内存溢出风险！");
            SM::instance().reportError(ModuleType::OUTPUTTER, ErrorLevel::STATUS_WARNING, log);
        }
    }

    bool ok = queue.enqueue(item);
    task_cv_.notify_one();
    return ok;
}

bool ConcreteDataOutputter::writeToDatabase(const SixDofResult& r) {
    DbShipData task;
    task.res = r;
    /*
    std::string log_msg = "入库任务：雷达ID=" + r.lidar_id + " 时间戳=" + std::to_string(r.timestamp) +
        " tx=" + std::to_string(r.tx) + " ty=" + std::to_string(r.ty) + " tz=" + std::to_string(r.tz) +
        " rx=" + std::to_string(r.rx) + " ry=" + std::to_string(r.ry) + " rz=" + std::to_string(r.rz) +
        " confidence=" + std::to_string(r.confidence);
    Logger::instance().info("[DB-INFO] " + log_msg);
    */
    //return enqueueWithDropOld(db_task_queue_, task, QueueLimit::DB_MAX_QUEUE, "DB任务队列", false);
    return true;
}

bool ConcreteDataOutputter::pushRealTimeData(const SixDofResult& r) {
    std::string log_msg = "实时数据投递：雷达ID=" + r.lidar_id + " 时间戳=" + std::to_string(r.timestamp) +
        " tx=" + std::to_string(r.tx) + " ty=" + std::to_string(r.ty) + " tz=" + std::to_string(r.tz) +
        " rx=" + std::to_string(r.rx) + " ry=" + std::to_string(r.ry) + " rz=" + std::to_string(r.rz) +
        " confidence=" + std::to_string(r.confidence);
    Logger::instance().info("[MQTT-INFO] " + log_msg);
    return enqueueWithDropOld(solve_task_queue_, r, QueueLimit::SOLVE_MAX_QUEUE, "船体解算队列", false);
}

bool ConcreteDataOutputter::pushRealTimeError(const SystemError& err) {
    DeviceErrMsg msg;
    msg.ip = ExtractIpByRegex(err.message);
    auto now = std::chrono::system_clock::now();
    std::time_t now_t = std::chrono::system_clock::to_time_t(now);
    std::tm tm_now = *std::localtime(&now_t);
    std::ostringstream time_ss;
    time_ss << std::put_time(&tm_now, "%Y-%m-%d %H:%M:%S");
    msg.time_str = time_ss.str();
    msg.err_code = static_cast<int>(err.level);

    // 错误告警为低优先级，溢出优先丢弃
    return enqueueWithDropOld(err_task_queue_, msg, QueueLimit::MQTT_ERR_MAX_QUEUE, "MQTT错误告警队列", true);
}

bool ConcreteDataOutputter::writeErrorToDatabase(const SystemError& err) {
    try {
        Logger::instance().info("[DB-ERROR] 模块:" + std::to_string(static_cast<int>(err.module))
            + " 等级:" + std::to_string(static_cast<int>(err.level))
            + " 信息:" + err.message);
    } catch (...) {}
    return true;
}

std::string ConcreteDataOutputter::ExtractIpByRegex(const std::string& msg) {
    try {
        std::regex ipv4_regex(R"((\d{1,3}\.){3}\d{1,3})");
        std::smatch match;
        if (std::regex_search(msg, match, ipv4_regex)) {
            return match.str();
        }
    } catch (...) {}
    return "";
}

void ConcreteDataOutputter::dbWorkerLoop() {
    Logger::instance().info("[DB Worker] 入库线程启动");
    while (worker_running_) {
        try {
            DbShipData task;
            if (db_task_queue_.dequeue(task)) {
                // 真实数据库写入逻辑，阻塞仅影响本线程
                try {
                    // db.insert(task.res);
                } catch (...) {
                    SM::instance().reportError(ModuleType::OUTPUTTER, ErrorLevel::STATUS_ERROR, "数据库写入异常");
                }
            } else {
                std::unique_lock<std::mutex> lk(task_mtx_);
                task_cv_.wait_for(lk, std::chrono::milliseconds(50));
            }
        } catch (...) {
            // 任意异常捕获，线程不退出
            Logger::instance().error("[DB Worker 线程捕获全局异常，继续循环消费任务]");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    Logger::instance().info("[DB Worker] 入库线程退出");
}

void ConcreteDataOutputter::solveWorkerLoop() {
    Logger::instance().info("[Solve Worker] 船体六自由度计算线程启动");
    while (worker_running_) {
        try {
            SixDofResult r;
            if (solve_task_queue_.dequeue(r)) {
                // 解算首尾坐标存入历史
                calcShipBowSternPos(r);

                // 计算加速度
                float bX = 0, bY = 0, bZ = 0;
                float sX = 0, sY = 0, sZ = 0;
                getBowSternAcc(bX, bY, bZ, sX, sY, sZ);

                // 组装MQTT数据包，投递到MQTT发送线程（高优先级，不标记低优）
                MqttShipData mqtt_pkg;
                mqtt_pkg.berth_id = monitor_cfg_.mqtt_cfg.berth_id;
                mqtt_pkg.lidar_ip = r.lidar_ip;

                std::time_t now_t = r.timestamp / 1000;
                std::tm tm_now = *std::localtime(&now_t);
                std::ostringstream time_ss;
                time_ss << std::put_time(&tm_now, "%Y-%m-%d %H:%M:%S");
                mqtt_pkg.time_str = time_ss.str();

                mqtt_pkg.surge = r.tx;
                mqtt_pkg.sway = r.ty;
                mqtt_pkg.heave = r.tz;
                mqtt_pkg.roll = r.rx;
                mqtt_pkg.pitch = r.ry;
                mqtt_pkg.yaw = r.rz;

                mqtt_pkg.bX = bX;
                mqtt_pkg.bY = bY;
                mqtt_pkg.bZ = bZ;
                mqtt_pkg.sX = sX;
                mqtt_pkg.sY = sY;
                mqtt_pkg.sZ = sZ;

                enqueueWithDropOld(mqtt_task_queue_, mqtt_pkg, QueueLimit::MQTT_DATA_MAX_QUEUE, "MQTT业务数据队列", false);
            } else {
                std::unique_lock<std::mutex> lk(task_mtx_);
                task_cv_.wait_for(lk, std::chrono::milliseconds(50));
            }
        } catch (...) {
            Logger::instance().error("[Solve Worker 线程捕获全局异常，继续循环消费任务]");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    Logger::instance().info("[Solve Worker] 船体解算线程退出");
}

void ConcreteDataOutputter::mqttWorkerLoop() {
    Logger::instance().info("[MQTT Worker] 推送线程启动");
    while (worker_running_) {
        try {
            bool has_task = false;
            // 优先处理正常六自由度数据
            MqttShipData pkg;
            if (mqtt_task_queue_.dequeue(pkg)) {
                has_task = true;
                try {
                    if (mqtt_.isConnected()) {
                        std::string json = mqtt_.buildJson(
                            pkg.berth_id, pkg.lidar_ip, pkg.time_str,
                            pkg.sway, pkg.surge, pkg.heave,
                            pkg.roll, pkg.pitch, pkg.yaw,
                            pkg.bX, pkg.bY, pkg.bZ,
                            pkg.sX, pkg.sY, pkg.sZ
                        );
                        std::cout << "MQTT推送数据包: " << json << std::endl;
                        mqtt_.publish(json, monitor_cfg_.mqtt_cfg.dev_status_topic);
                    }
                } catch (...) {
                    SM::instance().reportError(ModuleType::OUTPUTTER, ErrorLevel::STATUS_WARNING, "MQTT业务数据组装/发送异常");
                }
            }

            // 处理设备错误上报
            DeviceErrMsg err_msg;
            if (err_task_queue_.dequeue(err_msg)) {
                has_task = true;
                try {
                    if (mqtt_.isConnected()) {
                        std::string statusJson = mqtt_.buildDeviceStatusJson(err_msg.ip, err_msg.time_str, err_msg.err_code);
                        std::cout << "MQTT推送错误数据包: " << statusJson << std::endl;
                        mqtt_.publish(statusJson, monitor_cfg_.mqtt_cfg.dev_status_topic);
                    }
                } catch (...) {
                    Logger::instance().warn("MQTT错误消息推送异常");
                }
            }

            if (!has_task) {
                std::unique_lock<std::mutex> lk(task_mtx_);
                task_cv_.wait_for(lk, std::chrono::milliseconds(50));
            }
        } catch (...) {
            Logger::instance().error("[MQTT Worker 线程捕获全局异常，继续循环消费任务]");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    Logger::instance().info("[MQTT Worker] 推送线程退出");
}