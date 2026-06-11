#ifndef LOCK_FREE_QUEUE_H
#define LOCK_FREE_QUEUE_H

#include <atomic>
#include <vector>
#include <stdexcept>

template <typename T>
class LockFreeRingQueue {
public:
    explicit LockFreeRingQueue(size_t capacity)
        : capacity_(capacity), buffer_(capacity), head_(0), tail_(0) {
        if (capacity == 0) throw std::invalid_argument("Queue capacity must be greater than 0");
    }

    bool enqueue(const T& data) {
        size_t current_tail = tail_.load(std::memory_order_relaxed);
        size_t next_tail = (current_tail + 1) % capacity_;
        if (next_tail == head_.load(std::memory_order_acquire)) {
            return false; // 队列满
        }
        buffer_[current_tail] = data;
        tail_.store(next_tail, std::memory_order_release);
        return true;
    }

    bool dequeue(T& data) {
        size_t current_head = head_.load(std::memory_order_relaxed);
        if (current_head == tail_.load(std::memory_order_acquire)) {
            return false; // 队列空
        }
        data = buffer_[current_head];
        head_.store((current_head + 1) % capacity_, std::memory_order_release);
        return true;
    }

    size_t size() const {
        size_t current_head = head_.load(std::memory_order_relaxed);
        size_t current_tail = tail_.load(std::memory_order_relaxed);
        return (current_tail - current_head + capacity_) % capacity_;
    }

    bool empty() const {
        return head_.load(std::memory_order_relaxed) == tail_.load(std::memory_order_relaxed);
    }

    bool full() const {
        size_t next_tail = (tail_.load(std::memory_order_relaxed) + 1) % capacity_;
        return next_tail == head_.load(std::memory_order_relaxed);
    }

private:
    const size_t capacity_;
    std::vector<T> buffer_;
    std::atomic<size_t> head_;
    std::atomic<size_t> tail_;
};

#endif