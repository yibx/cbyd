#ifndef INDUSTRIAL_LOGGER_H
#define INDUSTRIAL_LOGGER_H

#include <spdlog/spdlog.h>
#include <spdlog/async.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/daily_file_sink.h>

#include <memory>
#include <string>
#include <vector>

class IndustrialLogger
{
public:
    static IndustrialLogger* Instance();

    void Init(const std::string& logPrefix,
              size_t maxFileSize = 10 * 1024 * 1024,
              size_t maxBackup = 30,
              size_t asyncQueueSize = 8192);

    void SetLogLevel(spdlog::level::level_enum lvl);
    void EnableConsole(bool enable);
    void Flush();
    void Shutdown();

    std::shared_ptr<spdlog::logger> GetLogger() { return logger_; }
    static std::string GetShortFileName(const char* fullPath);

    // 模板：格式化字符串 + 可变参数
    template<typename... Args>
    void LogTrace(const std::string& mod, const char* file, int line, const char* fmt, Args&&... args)
    {
        if (!logger_) return;
        std::string f = GetShortFileName(file);
        logger_->trace("[{}] {}:{} | {}", mod, f.c_str(), line, fmt, std::forward<Args>(args)...);
    }
    // 重载：直接传入std::string消息
    void LogTrace(const std::string& mod, const char* file, int line, const std::string& msg)
    {
        if (!logger_) return;
        std::string f = GetShortFileName(file);
        logger_->trace("[{}] {}:{} | {}", mod, f.c_str(), line, msg);
    }

    template<typename... Args>
    void LogDebug(const std::string& mod, const char* file, int line, const char* fmt, Args&&... args)
    {
        if (!logger_) return;
        std::string f = GetShortFileName(file);
        logger_->debug("[{}] {}:{} | {}", mod, f.c_str(), line, fmt, std::forward<Args>(args)...);
    }
    void LogDebug(const std::string& mod, const char* file, int line, const std::string& msg)
    {
        if (!logger_) return;
        std::string f = GetShortFileName(file);
        logger_->debug("[{}] {}:{} | {}", mod, f.c_str(), line, msg);
    }

    template<typename... Args>
    void LogInfo(const std::string& mod, const char* file, int line, const char* fmt, Args&&... args)
    {
        if (!logger_) return;
        std::string f = GetShortFileName(file);
        logger_->info("[{}] {}:{} | {}", mod, f.c_str(), line, fmt, std::forward<Args>(args)...);
    }
    void LogInfo(const std::string& mod, const char* file, int line, const std::string& msg)
    {
        if (!logger_) return;
        std::string f = GetShortFileName(file);
        logger_->info("[{}] {}:{} | {}", mod, f.c_str(), line, msg);
    }

    template<typename... Args>
    void LogWarn(const std::string& mod, const char* file, int line, const char* fmt, Args&&... args)
    {
        if (!logger_) return;
        std::string f = GetShortFileName(file);
        logger_->warn("[{}] {}:{} | {}", mod, f.c_str(), line, fmt, std::forward<Args>(args)...);
    }
    void LogWarn(const std::string& mod, const char* file, int line, const std::string& msg)
    {
        if (!logger_) return;
        std::string f = GetShortFileName(file);
        logger_->warn("[{}] {}:{} | {}", mod, f.c_str(), line, msg);
    }

    template<typename... Args>
    void LogError(const std::string& mod, const char* file, int line, const char* fmt, Args&&... args)
    {
        if (!logger_) return;
        std::string f = GetShortFileName(file);
        logger_->error("[{}] {}:{} | {}", mod, f.c_str(), line, fmt, std::forward<Args>(args)...);
    }
    void LogError(const std::string& mod, const char* file, int line, const std::string& msg)
    {
        if (!logger_) return;
        std::string f = GetShortFileName(file);
        logger_->error("[{}] {}:{} | {}", mod, f.c_str(), line, msg);
    }

    template<typename... Args>
    void LogCrit(const std::string& mod, const char* file, int line, const char* fmt, Args&&... args)
    {
        if (!logger_) return;
        std::string f = GetShortFileName(file);
        logger_->critical("[{}] {}:{} | {}", mod, f.c_str(), line, fmt, std::forward<Args>(args)...);
    }
    void LogCrit(const std::string& mod, const char* file, int line, const std::string& msg)
    {
        if (!logger_) return;
        std::string f = GetShortFileName(file);
        logger_->critical("[{}] {}:{} | {}", mod, f.c_str(), line, msg);
    }

private:
    IndustrialLogger();
    ~IndustrialLogger();
    IndustrialLogger(const IndustrialLogger&) = delete;
    IndustrialLogger& operator=(const IndustrialLogger&) = delete;

    static IndustrialLogger* instance_;
    std::shared_ptr<spdlog::logger> logger_;
    bool consoleEnable_;
};

// 日志宏，自动注入文件、行号，可变参数透传
#define LOG_TRACE(mod, ...) IndustrialLogger::Instance()->LogTrace(mod, __FILE__, __LINE__, ##__VA_ARGS__)
#define LOG_DEBUG(mod, ...) IndustrialLogger::Instance()->LogDebug(mod, __FILE__, __LINE__, ##__VA_ARGS__)
#define LOG_INFO(mod, ...)  IndustrialLogger::Instance()->LogInfo(mod, __FILE__, __LINE__, ##__VA_ARGS__)
#define LOG_WARN(mod, ...)  IndustrialLogger::Instance()->LogWarn(mod, __FILE__, __LINE__, ##__VA_ARGS__)
#define LOG_ERROR(mod, ...) IndustrialLogger::Instance()->LogError(mod, __FILE__, __LINE__, ##__VA_ARGS__)
#define LOG_CRIT(mod, ...)  IndustrialLogger::Instance()->LogCrit(mod, __FILE__, __LINE__, ##__VA_ARGS__)

#endif // INDUSTRIAL_LOGGER_H