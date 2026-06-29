#ifndef SYSTEM_STATE_MANAGER_H
#define SYSTEM_STATE_MANAGER_H

#include <atomic>
#include <string>
#include <chrono>
#include <functional>

// ϵͳ״̬
enum class SystemState {
    IDLE,           // ����
    INITIALIZING,   // ��ʼ����
    RUNNING,        // ��������
    PAUSED,         // ��ͣ
    RUNERROR,       // ���д���
    FATAL_ERROR,    // ��������������
    EXITING         // �˳���
};

// ϵͳģ��
enum class ModuleType {
    ACQUIRER,
    FUSER,
    SIX_DOF_CALC,
    OUTPUTTER
};

// ����ȼ�
enum class ErrorLevel {
    STATUS_OK = 0,
    STATUS_WARNING = 1,    // ���棬��Ӱ������
    STATUS_ERROR = 2,      // ���󣬿��Զ��ָ�
    STATUS_FATAL = 3       // ����������ֹͣ
};

// ������Ϣ
struct SystemError {
    ModuleType module;
    ErrorLevel level;
    std::string message;
    uint64_t timestamp;
    bool need_push;
};

// ״̬�� + �������� 
class SystemStateManager {
public:
    static SystemStateManager& instance() {
        static SystemStateManager inst;
        return inst;
    }

    // ״̬����
    void setState(SystemState state);
    SystemState getState() const;

    // ģ������ϱ�
    void reportError(ModuleType module, ErrorLevel level, const std::string& msg);

    // ע�����ͻص�
    void setErrorPushCallback(std::function<bool(const SystemError&)> cb);
    void setDatabaseWriteCallback(std::function<bool(const SystemError&)> cb);

    // ��ȡ���һ�δ���
    SystemError getLastError() const;
    bool hasError() const;

    // �������
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

