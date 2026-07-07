#include "Logger.h"
#include <algorithm>
#include <cstring>

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
{
}

IndustrialLogger::~IndustrialLogger()
{
    Shutdown();
}

void IndustrialLogger::Init(const std::string& logPrefix,
                            size_t maxFileSize,
                            size_t maxBackup,
                            size_t asyncQueueSize)
{
    spdlog::init_thread_pool(asyncQueueSize, 1);

    auto dailySink = std::make_shared<spdlog::sinks::daily_file_sink_mt>(
        logPrefix,
        0, 0,
        true,
        static_cast<uint16_t>(maxBackup)
    );

    auto consoleSink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();

    std::vector<spdlog::sink_ptr> sinks;
    sinks.emplace_back(dailySink);
    if (consoleEnable_)
    {
        sinks.emplace_back(consoleSink);
    }

    logger_ = std::make_shared<spdlog::async_logger>(
        "industrial_async_log",
        sinks.begin(),
        sinks.end(),
        spdlog::thread_pool(),
        spdlog::async_overflow_policy::block
    );

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
    if (logger_)
    {
        logger_->flush();
    }
}

void IndustrialLogger::Shutdown()
{
    if (logger_)
    {
        logger_->flush();
        logger_.reset();
    }
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