#include "easylogging++.h"
INITIALIZE_EASYLOGGINGPP
#include "Logger.h"
#include <dirent.h>
#include <sys/stat.h>
#include <time.h>
#include <string>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <ctime>
#include <iostream>
#include <thread>
#include <mutex>
#include <atomic>

// 文件域静态变量
static std::string g_current_log_name;
static std::mutex g_log_conf_mtx;
static std::atomic<bool> g_log_rotate_run{true};
static std::thread g_log_rotate_thread;

// 线程安全生成当日日志文件名
static std::string getSafeLogFileName() {
    std::time_t ts = std::time(nullptr);
    std::tm tm_buf{};
    localtime_r(&ts, &tm_buf);
    std::ostringstream oss;
    oss << "cbyd_" << std::put_time(&tm_buf, "%Y-%m-%d") << ".log";
    return oss.str();
}

// 获取纯日期字符串，用于跨天对比
static std::string getTodayDateStr() {
    std::time_t ts = std::time(nullptr);
    std::tm tm_buf{};
    localtime_r(&ts, &tm_buf);
    std::ostringstream oss;
    oss << std::put_time(&tm_buf, "%Y-%m-%d");
    return oss.str();
}

// 增加force参数，两段式重配强制重建文件流
static void rotateLogFile(bool force = false) {
    std::lock_guard<std::mutex> lock(g_log_conf_mtx);
    std::string new_log_name = getSafeLogFileName();

    // 不强制且文件名相同，直接返回
    if (!force && new_log_name == g_current_log_name) {
        return;
    }

    el::Logger* defaultLogger = el::Loggers::getLogger("default");
    defaultLogger->flush();

    // 关键修复：先临时关闭文件输出，清除失效文件句柄缓存
    el::Configurations tempCloseConf;
    tempCloseConf.set(el::Level::Global, el::ConfigurationType::ToFile, "false");
    el::Loggers::reconfigureLogger("default", tempCloseConf);

    // 重新完整配置，重新打开日志文件
    el::Configurations conf;
    conf.set(el::Level::Global, el::ConfigurationType::Format, "%datetime %level: %msg");
    conf.set(el::Level::Global, el::ConfigurationType::Filename, new_log_name);
    conf.set(el::Level::Global, el::ConfigurationType::ToFile, "true");
    conf.set(el::Level::Global, el::ConfigurationType::ToStandardOutput, "true");
    el::Loggers::reconfigureLogger("default", conf);

    g_current_log_name = new_log_name;
    LOG(INFO) << "==== 切换日志文件，新文件：" << new_log_name << " ====";
    defaultLogger->flush();
}

// 后台定时检测线程函数
static void logRotateThread() {
    std::string last_date = getTodayDateStr();
    while (g_log_rotate_run.load(std::memory_order_acquire)) {
        std::this_thread::sleep_for(std::chrono::seconds(60));
        std::string now_date = getTodayDateStr();
        if (now_date != last_date) {
            rotateLogFile(false);
            last_date = now_date;
        }
    }
}

// 单例获取
Logger& Logger::instance() {
    static Logger inst; // C++11 线程安全单例
    return inst;
}

// 构造函数自动初始化
Logger::Logger() {
    initConfig();
    g_log_rotate_run.store(true, std::memory_order_release);
    // 启动跨天检测线程
    g_log_rotate_thread = std::thread(logRotateThread);
}

// 析构函数：停止轮换线程，释放资源
Logger::~Logger() {
    g_log_rotate_run.store(false, std::memory_order_release);
    if (g_log_rotate_thread.joinable()) {
        g_log_rotate_thread.join();
    }
    el::Logger* defaultLogger = el::Loggers::getLogger("default");
    defaultLogger->flush();
    LOG(INFO) << "日志轮换线程已安全退出";
    defaultLogger->flush();
    // 微小延时保证IO落盘
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
}

// 日志初始化
void Logger::initConfig() {
#if ELPP_THREAD_SAFE
    std::cout << "===== ELPP_THREAD_SAFE 已开启，线程安全模式生效 =====" << std::endl;
#else
    std::cout << "===== ELPP_THREAD_SAFE 未开启，非线程安全！ =====" << std::endl;
#endif
    std::cout << "easylogging++ Version: " << el::VersionInfo::version() << std::endl;

    g_current_log_name = getSafeLogFileName();
    // 启动强制重建日志文件流
    rotateLogFile(true);
    std::cout << "初始日志文件：" << g_current_log_name << std::endl;

    // 删除旧日志，此处不能用LOG，此时日志刚初始化，避免输出错乱
    DIR* dir = opendir(".");
    if (!dir) return;
    time_t nowTime;
    time(&nowTime);
    struct dirent* entry;
    struct stat fileStat;
    int keepDays = 7;
    while ((entry = readdir(dir)) != nullptr) {
        std::string fileName = entry->d_name;
        if (fileName.find("cbyd_") == 0 && fileName.find(".log") != std::string::npos) {
            if (stat(fileName.c_str(), &fileStat) == -1) continue;
            double seconds = difftime(nowTime, fileStat.st_mtime);
            if (seconds > keepDays * 86400) {
                remove(fileName.c_str());
                std::cout << "自动清理过期日志：" << fileName << std::endl;
            }
        }
    }
    closedir(dir);
}

// 删除N天前日志文件
void Logger::deleteOldLogs(int keepDays) {
    DIR* dir = opendir(".");
    if (!dir) return;

    time_t nowTime;
    time(&nowTime);
    struct dirent* entry;
    struct stat fileStat;

    while ((entry = readdir(dir)) != nullptr) {
        std::string fileName = entry->d_name;
        if (fileName.find("cbyd_") == 0 && fileName.find(".log") != std::string::npos) {
            if (stat(fileName.c_str(), &fileStat) == -1) continue;
            double seconds = difftime(nowTime, fileStat.st_mtime);
            if (seconds > keepDays * 86400) {
                remove(fileName.c_str());
                LOG(INFO) << "自动清理过期日志：" << fileName;
            }
        }
    }
    closedir(dir);
    el::Loggers::getLogger("default")->flush();
}

// 日志等级封装接口
void Logger::info(const std::string& msg) {
    LOG(INFO) << msg;
}

void Logger::warn(const std::string& msg) {
    LOG(WARNING) << msg;
}

void Logger::error(const std::string& msg) {
    LOG(ERROR) << msg;
}

void Logger::debug(const std::string& msg) {
    LOG(DEBUG) << msg;
}

void Logger::fatal(const std::string& msg) {
    LOG(FATAL) << msg;
}