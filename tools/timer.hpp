#pragma once
#include <chrono>
#include <string>
#include <functional>
#include <optional>

namespace tools {

enum class TimeUnit {
    NANOSECONDS,
    MICROSECONDS,
    MILLISECONDS,
    SECONDS
};

class ScopedTimer {
public:
    using Callback = std::function<void(const std::string&, double)>;
    
    // 构造函数：使用回调函数
    explicit ScopedTimer(const std::string& name,
                         Callback callback,
                         TimeUnit unit = TimeUnit::MILLISECONDS)
        : name_(name), unit_(unit), callback_(std::move(callback)),
          start_(std::chrono::steady_clock::now()) {}
    
    // 构造函数：直接写入到变量
    ScopedTimer(const std::string& name,
                double& output_var,
                TimeUnit unit = TimeUnit::MILLISECONDS)
        : name_(name), unit_(unit), output_var_(&output_var),
          start_(std::chrono::steady_clock::now()) {}
    
    ~ScopedTimer() {
        auto end = std::chrono::steady_clock::now();
        auto duration = end - start_;
        double elapsed = get_elapsed(duration);
        
        if (callback_) {
            (*callback_)(name_, elapsed);
        } else if (output_var_) {
            *output_var_ = elapsed;
        }
    }
    
    // 手动获取已耗时（不等待析构）
    double elapsed_ms() const {
        auto now = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(now - start_);
        return duration.count() / 1000.0;
    }
    
    // 获取原始耗时值（秒）
    double elapsed_seconds() const {
        auto now = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(now - start_);
        return duration.count() / 1e9;
    }

private:
    double get_elapsed(const std::chrono::steady_clock::duration& duration) const {
        switch (unit_) {
            case TimeUnit::NANOSECONDS:
                return std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
            case TimeUnit::MICROSECONDS:
                return std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
            case TimeUnit::MILLISECONDS:
                return std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
            case TimeUnit::SECONDS:
                return std::chrono::duration_cast<std::chrono::seconds>(duration).count();
        }
        return 0.0;
    }
    
    std::string name_;
    TimeUnit unit_;
    std::optional<Callback> callback_;
    double* output_var_ = nullptr;
    std::chrono::steady_clock::time_point start_;
};

} // namespace tools