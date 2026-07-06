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

// 全局静态变量
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

// 执行日志文件切换
static void rotateLogFile() {
    std::lock_guard<std::mutex> lock(g_log_conf_mtx);
    std::string new_log_name = getSafeLogFileName();
    if (new_log_name == g_current_log_name) {
        return;
    }

    el::Configurations conf;
    conf.set(el::Level::Global, el::ConfigurationType::Format, "%datetime %level: %msg");
    conf.set(el::Level::Global, el::ConfigurationType::Filename, new_log_name);
    conf.set(el::Level::Global, el::ConfigurationType::ToFile, "true");
    conf.set(el::Level::Global, el::ConfigurationType::ToStandardOutput, "true");
    el::Loggers::reconfigureLogger("default", conf);

    g_current_log_name = new_log_name;
    LOG(INFO) << "==== 跨天自动切换日志文件，新文件：" << new_log_name << " ====";
}

// 后台定时检测线程函数
static void logRotateThread() {
    std::string last_date = getTodayDateStr();
    while (g_log_rotate_run.load()) {
        // 60秒检测一次，降低CPU消耗
        std::this_thread::sleep_for(std::chrono::seconds(60));
        std::string now_date = getTodayDateStr();
        if (now_date != last_date) {
            rotateLogFile();
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
    // 启动跨天检测线程
    g_log_rotate_thread = std::thread(logRotateThread);
}

// 析构函数：停止轮换线程，释放资源
Logger::~Logger() {
    g_log_rotate_run.store(false);
    if (g_log_rotate_thread.joinable()) {
        g_log_rotate_thread.join();
    }
    LOG(INFO) << "日志轮换线程已安全退出";
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
    el::Configurations conf;
    conf.set(el::Level::Global, el::ConfigurationType::Format, "%datetime %level: %msg");
    conf.set(el::Level::Global, el::ConfigurationType::Filename, g_current_log_name);
    conf.set(el::Level::Global, el::ConfigurationType::ToFile, "true");
    conf.set(el::Level::Global, el::ConfigurationType::ToStandardOutput, "true");

    el::Loggers::reconfigureLogger("default", conf);
    std::cout << "初始日志文件：" << g_current_log_name << std::endl;

    deleteOldLogs(7);
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
        // 只匹配业务日志文件
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