#ifndef LOGGER_H
#define LOGGER_H

#include <string>

class Logger {
public:
    // 获取单例实例
    static Logger& instance();

    // 禁用拷贝和赋值
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    // 日志接口
    void info(const std::string& msg);
    void warn(const std::string& msg);
    void error(const std::string& msg);

private:
    // 私有构造 → 外部无法创建对象（单例核心）
    Logger();
    ~Logger() = default;

    // 初始化配置
    void initConfig();

    // 删除旧日志
    void deleteOldLogs(int keepDays);
};

// 方便调用的宏（可选）
#define LOG_INFO  Logger::instance().info
#define LOG_WARN  Logger::instance().warn
#define LOG_ERROR Logger::instance().error

#endif // LOGGER_H
