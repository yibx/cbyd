#include "system_state_manager.h"
#include <stdexcept>

uint64_t getCurrentTimestampMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()
    ).count();
}

// 状态
void SystemStateManager::setState(SystemState state) {
    system_state_ = state;
}

SystemState SystemStateManager::getState() const {
    return system_state_;
}

// 错误上报
void SystemStateManager::reportError(ModuleType module, ErrorLevel level, const std::string& msg) {
    SystemError err{};
    err.module = module;
    err.level = level;
    err.message = msg;
    err.timestamp = getCurrentTimestampMs();
    err.need_push = (level != ErrorLevel::STATUS_OK);

    last_error_ = err;
    has_error_ = (level >= ErrorLevel::STATUS_WARNING);

    // 致命错误自动切换系统状态
    if (level == ErrorLevel::STATUS_FATAL) {
        setState(SystemState::FATAL_ERROR);
    }
    else if (level == ErrorLevel::STATUS_ERROR) {
        setState(SystemState::RUNERROR);
    }

    // 推送 + 存库
    doPush(err);
    doWriteDB(err);
}

// 推送
void SystemStateManager::doPush(const SystemError& err) {
    if (push_callback_ && err.need_push) {
        try { push_callback_(err); }
        catch (...) {}
    }
}

void SystemStateManager::doWriteDB(const SystemError& err) {
    if (db_callback_ && err.need_push) {
        try { db_callback_(err); }
        catch (...) {}
    }
}

// 回调注册
void SystemStateManager::setErrorPushCallback(std::function<bool(const SystemError&)> cb) {
    push_callback_ = std::move(cb);
}

void SystemStateManager::setDatabaseWriteCallback(std::function<bool(const SystemError&)> cb) {
    db_callback_ = std::move(cb);
}


SystemError SystemStateManager::getLastError() const {
    return last_error_;
}

bool SystemStateManager::hasError() const {
    return has_error_;
}

void SystemStateManager::clearError() {
    has_error_ = false;
}
