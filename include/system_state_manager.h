#ifndef SYSTEM_STATE_MANAGER_H
#define SYSTEM_STATE_MANAGER_H

#include <atomic>
#include <string>
#include <chrono>
#include <functional>

// 系统状态
enum class SystemState {
    IDLE,           // 空闲
    INITIALIZING,   // 初始化中
    RUNNING,        // 正常运行
    PAUSED,         // 暂停
    RUNERROR,       // 运行错误
    FATAL_ERROR,    // 致命错误，需重启
    EXITING         // 退出中
};

// 系统模块
enum class ModuleType {
    ACQUIRER,
    FUSER,
    SIX_DOF_CALC,
    OUTPUTTER
};

// 错误等级
enum class ErrorLevel {
    STATUS_OK = 0,
    STATUS_WARNING = 1,    // 警告，不影响运行
    STATUS_ERROR = 2,      // 错误，可自动恢复
    STATUS_FATAL = 3       // 致命，必须停止
};

// 错误信息
struct SystemError {
    ModuleType module;
    ErrorLevel level;
    std::string message;
    uint64_t timestamp;
    bool need_push;
};

// 状态机 + 错误推送 
class SystemStateManager {
public:
    static SystemStateManager& instance() {
        static SystemStateManager inst;
        return inst;
    }

    // 状态控制
    void setState(SystemState state);
    SystemState getState() const;

    // 模块错误上报
    void reportError(ModuleType module, ErrorLevel level, const std::string& msg);

    // 注册推送回调
    void setErrorPushCallback(std::function<bool(const SystemError&)> cb);
    void setDatabaseWriteCallback(std::function<bool(const SystemError&)> cb);

    // 获取最近一次错误
    SystemError getLastError() const;
    bool hasError() const;

    // 清除错误
    void clearError();

private:
    SystemStateManager() {
        system_state_ = SystemState::IDLE;
        has_error_ = false;
    }

    std::atomic<SystemState> system_state_;
    std::atomic<bool> has_error_;
    SystemError last_error_;

    std::function<bool(const SystemError&)> push_callback_;
    std::function<bool(const SystemError&)> db_callback_;

    void doPush(const SystemError& err);
    void doWriteDB(const SystemError& err);
};

#endif // SYSTEM_STATE_MANAGER_H

