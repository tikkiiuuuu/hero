#ifndef TOOLS__THREAD_SAFE_QUEUE_HPP
#define TOOLS__THREAD_SAFE_QUEUE_HPP

#include <condition_variable>
#include <functional>
#include <iostream>
#include <mutex>
#include <queue>

namespace tools {
template<typename T, bool PopWhenFull = false>
class ThreadSafeQueue {
public:
    ThreadSafeQueue(
        size_t max_size,
        std::function<void(void)> full_handler = [] {}
    ):
        max_size_(max_size),
        full_handler_(full_handler) {}

    void push(const T& value) {
        std::unique_lock<std::mutex> lock(mutex_);

        if (queue_.size() >= max_size_) {
            if (PopWhenFull) {
                queue_.pop();
            } else {
                full_handler_();
                return;
            }
        }

        queue_.push(value);
        not_empty_condition_.notify_all();
    }

    void pop(T& value) {
        std::unique_lock<std::mutex> lock(mutex_);

        not_empty_condition_.wait(lock, [this] { return !queue_.empty(); });

        if (queue_.empty()) {
            std::cerr << "Error: Attempt to pop from an empty queue." << std::endl;
            return;
        }

        value = queue_.front();
        queue_.pop();
    }

    T pop() {
        std::unique_lock<std::mutex> lock(mutex_);

        not_empty_condition_.wait(lock, [this] { return !queue_.empty(); });

        T value = std::move(queue_.front());
        queue_.pop();
        return std::move(value);
    }

    T front() {
        std::unique_lock<std::mutex> lock(mutex_);

        not_empty_condition_.wait(lock, [this] { return !queue_.empty(); });

        return queue_.front();
    }

    void back(T& value) {
        std::unique_lock<std::mutex> lock(mutex_);

        if (queue_.empty()) {
            std::cerr << "Error: Attempt to access the back of an empty queue." << std::endl;
            return;
        }

        value = queue_.back();
    }

    bool empty() {
        std::unique_lock<std::mutex> lock(mutex_);
        return queue_.empty();
    }

    void clear() {
        std::unique_lock<std::mutex> lock(mutex_);
        while (!queue_.empty()) {
            queue_.pop();
        }
        not_empty_condition_.notify_all(); // 如果其他线程正在等待队列不为空，这样可以唤醒它们
    }

private:
    std::queue<T> queue_;
    size_t max_size_;
    mutable std::mutex mutex_;
    std::condition_variable not_empty_condition_;
    std::function<void(void)> full_handler_;
};

//* 默认超出后覆盖，但据设计一般不会超
template<class T,int max_size>
class ThreadSafeRingBuffer{
    public:
        ThreadSafeRingBuffer(): max_size_(max_size),buffer_(max_size){}
        void push(const T& value){
            std::unique_lock<std::mutex> lock(mutex_);
            if(size_==max_size_){
                front_index_ = (front_index_ + 1) % max_size_;
                size_--;
            }
            buffer_[(back_index_+1)%max_size_] = value;
            back_index_ = (back_index_ + 1) % max_size_;
            size_++;
            mutex_condition_.notify_all();
        }
        T pop(int num = 1){
            std::unique_lock<std::mutex> lock(mutex_);
            mutex_condition_.wait(lock, [this] { return size_>0; });

            if (size_<num) {
                std::cerr << "Error: Attempt to pop from an empty queue." << std::endl;
                return T();
            }
            T value = buffer_[(front_index_+num-1)%max_size_];//* 从一数第几个
            // buffer_[front_index_] = T();
            front_index_ = (front_index_ + num) % max_size_;
            size_-=num;
            return value;
        }
        T front(int num = 1){
            std::unique_lock<std::mutex> lock(mutex_);
            mutex_condition_.wait(lock, [this] { return size_>0; });
            if(num>size_){
                std::cerr << "Error: Attempt to access the back of an empty queue." << std::endl;
                return T();
            }
            return buffer_[(front_index_+num-1)%max_size_];
        }
        T back(int num = 1){
            std::unique_lock<std::mutex> lock(mutex_);
            mutex_condition_.wait(lock, [this] { return size_>0; });
            if(num>size_){
                std::cerr << "Error: Attempt to access the back of an empty queue." << std::endl;
                return T();
            }
            return buffer_[(back_index_-num+1)%max_size_];
        }
        void clear(){
            std::unique_lock<std::mutex> lock(mutex_);
            size_ = 0;
            front_index_ = 0;
            back_index_ = 0;
            buffer_.clear();
            mutex_condition_.notify_all();
        }
        bool empty(){
            std::unique_lock<std::mutex> lock(mutex_);
            return size_ == 0;
            mutex_condition_.notify_all();
        }
    private:
        std::vector<T> buffer_;
        mutable std::mutex mutex_;
        size_t size_ = 0;
        std::condition_variable mutex_condition_;
        size_t max_size_;
        size_t front_index_ = 0;
        size_t back_index_ = 0;
};

} // namespace tools

#endif // TOOLS__THREAD_SAFE_QUEUE_HPP