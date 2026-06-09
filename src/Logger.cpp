#include "easylogging++.h"

#include "Logger.h"
#include <dirent.h>
#include <sys/stat.h>
#include <time.h>
#include <string>

INITIALIZE_EASYLOGGINGPP

// 单例获取
Logger& Logger::instance() {
    static Logger inst; // C++11 线程安全单例
    return inst;
}

// 构造函数里自动初始化
Logger::Logger() {
    initConfig();
}

// 日志初始化
void Logger::initConfig() {
    el::Configurations conf;

    conf.set(el::Level::Global, el::ConfigurationType::Format, "%datetime %level: %msg");
    conf.set(el::Level::Global, el::ConfigurationType::Filename, "net_monitor_%datetime{%Y-%m-%d}.log");

    conf.set(el::Level::Global, el::ConfigurationType::ToFile, "true");
    conf.set(el::Level::Global, el::ConfigurationType::ToStandardOutput, "true");

    el::Loggers::reconfigureLogger("default", conf);
    deleteOldLogs(7);
}

// 删除7天前日志
void Logger::deleteOldLogs(int keepDays) {
    DIR* dir = opendir(".");
    if (!dir) return;

    time_t nowTime;
    time(&nowTime);

    struct dirent* entry;
    struct stat fileStat;

    while ((entry = readdir(dir)) != nullptr) {
        std::string fileName = entry->d_name;
        if (fileName.find("net_monitor_") == 0 && fileName.find(".log") != std::string::npos) {
            if (stat(fileName.c_str(), &fileStat) == -1) continue;

            double seconds = difftime(nowTime, fileStat.st_mtime);
            if (seconds > keepDays * 86400) {
                remove(fileName.c_str());
            }
        }
    }
    closedir(dir);
}

// 日志接口
void Logger::info(const std::string& msg) {
    LOG(INFO) << msg;
}

void Logger::warn(const std::string& msg) {
    LOG(WARNING) << msg;
}

void Logger::error(const std::string& msg) {
    LOG(ERROR) << msg;
}
