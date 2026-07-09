#include "Logger.h"
#include <algorithm>
#include <cstring>
#include <sstream>

IndustrialLogger* IndustrialLogger::instance_ = nullptr;

IndustrialLogger* IndustrialLogger::Instance()
{
    if (nullptr == instance_)
    {
        instance_ = new IndustrialLogger();
    }
    return instance_;
}

IndustrialLogger::IndustrialLogger()
    : consoleEnable_(true)
    , logger_(nullptr)
    , maxFileSize_(0)
    , maxBackup_(0)
    , asyncQueueSize_(0)
{
}

IndustrialLogger::~IndustrialLogger()
{
    Shutdown();
}

std::string IndustrialLogger::GetTodayDate()
{
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm localTm = *std::localtime(&t);

    char buf[32] = {0};
    std::strftime(buf, sizeof(buf), "%Y-%m-%d", &localTm);
    return std::string(buf);
}

void IndustrialLogger::RebuildDailyRotateSink()
{
    todayStr_ = GetTodayDate();
    // 拼接带日期前缀 ./logs/app_2026-07-07
    std::string fullPrefix = baseLogPrefix_ + "_" + todayStr_;
    fileSink_ = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
        fullPrefix,
        maxFileSize_,
        maxBackup_
    );
}

void IndustrialLogger::Init(const std::string& logPrefix,
                            size_t maxFileSize,
                            size_t maxBackup,
                            size_t asyncQueueSize)
{
    // 保存配置
    baseLogPrefix_ = logPrefix;
    maxFileSize_ = maxFileSize;
    maxBackup_ = maxBackup;
    asyncQueueSize_ = asyncQueueSize;

    // 初始化异步线程池
    spdlog::init_thread_pool(asyncQueueSize_, 1);

    // 控制台输出sink
    consoleSink_ = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();

    // 创建当日带日期的滚动文件sink
    RebuildDailyRotateSink();

    std::vector<spdlog::sink_ptr> sinks;
    sinks.emplace_back(fileSink_);
    if (consoleEnable_)
    {
        sinks.emplace_back(consoleSink_);
    }

    // 创建异步logger
    logger_ = std::make_shared<spdlog::async_logger>(
        "industrial_async_log",
        sinks.begin(),
        sinks.end(),
        spdlog::thread_pool(),
        spdlog::async_overflow_policy::block
    );

    // 日志格式不变
    const std::string pattern = "%Y-%m-%d %H:%M:%S.%f [%t] %^%l%$ %v";
    logger_->set_pattern(pattern);
    logger_->flush_on(spdlog::level::err);
    logger_->set_level(spdlog::level::info);
    spdlog::set_default_logger(logger_);
}

void IndustrialLogger::SetLogLevel(spdlog::level::level_enum lvl)
{
    if (logger_)
    {
        logger_->set_level(lvl);
    }
}

void IndustrialLogger::EnableConsole(bool enable)
{
    consoleEnable_ = enable;
}

void IndustrialLogger::Flush()
{
    if (!logger_)
        return;

    std::string curDate = GetTodayDate();
    if (curDate != todayStr_)
    {
        // 刷完旧缓存
        logger_->flush();

        // 释放旧资源
        logger_.reset();
        fileSink_.reset();

        // 生成新日期文件sink
        RebuildDailyRotateSink();

        std::vector<spdlog::sink_ptr> sinks;
        sinks.emplace_back(fileSink_);
        if (consoleEnable_)
        {
            sinks.emplace_back(consoleSink_);
        }

        // 重建异步logger实例
        logger_ = std::make_shared<spdlog::async_logger>(
            "industrial_async_log",
            sinks.begin(),
            sinks.end(),
            spdlog::thread_pool(),
            spdlog::async_overflow_policy::block
        );

        // 复用原有格式与配置
        const std::string pattern = "%Y-%m-%d %H:%M:%S.%f [%t] %^%l%$ %v";
        logger_->set_pattern(pattern);
        logger_->flush_on(spdlog::level::err);
        logger_->set_level(spdlog::level::info);
        spdlog::set_default_logger(logger_);
    }

    logger_->flush();
}

void IndustrialLogger::Shutdown()
{
    if (logger_)
    {
        logger_->flush();
        logger_.reset();
    }
    fileSink_.reset();
    consoleSink_.reset();
    spdlog::shutdown();
}

std::string IndustrialLogger::GetShortFileName(const char* fullPath)
{
    std::string path(fullPath);
    size_t pos = path.find_last_of("/\\");
    if (pos != std::string::npos)
    {
        return path.substr(pos + 1);
    }
    return path;
}