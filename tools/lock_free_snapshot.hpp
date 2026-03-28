#pragma once

#include <memory>
#include <atomic>

template <typename T>
class LockFreeSnapshot {
public:
    LockFreeSnapshot() : ptr_(std::make_shared<T>()) {}

    // 存储新状态（无锁写入，生成一个新副本替换）
    void store(const T& value) {
        std::shared_ptr<T> new_ptr = std::make_shared<T>(value);
        std::atomic_store(&ptr_, new_ptr);
    }

    // 获取当前状态快照（无锁读取）
    T load() const {
        std::shared_ptr<T> current_ptr = std::atomic_load(&ptr_);
        return *current_ptr;
    }

private:
    std::shared_ptr<T> ptr_;
};