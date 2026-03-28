// #include "trajectory.hpp"

// #include <cmath>

// namespace tools
// {
// constexpr double g = 9.7833;

// Trajectory::Trajectory(const double v0, const double d, const double h)
// {
//   auto a = g * d * d / (2 * v0 * v0);
//   auto b = -d;
//   auto c = a + h;
//   auto delta = b * b - 4 * a * c;

//   if (delta < 0) {
//     unsolvable = true;
//     return;
//   }

//   unsolvable = false;
//   auto tan_pitch_1 = (-b + std::sqrt(delta)) / (2 * a);
//   auto tan_pitch_2 = (-b - std::sqrt(delta)) / (2 * a);
//   auto pitch_1 = std::atan(tan_pitch_1);
//   auto pitch_2 = std::atan(tan_pitch_2);
//   auto t_1 = d / (v0 * std::cos(pitch_1));
//   auto t_2 = d / (v0 * std::cos(pitch_2));

//   pitch = (t_1 < t_2) ? pitch_1 : pitch_2;
//   fly_time = (t_1 < t_2) ? t_1 : t_2;
// }

// }  // namespace tools

#include "tools/trajectory.hpp" // 修正引用路径
#include <cmath>
#include <ceres/ceres.h>
#include <ceres/jet.h>
#include <glog/logging.h>

namespace tools
{
constexpr double GRAVITY = 9.7947; 
constexpr double K = 0.019; // 空气阻力系数，根据实际情况调整 0.049

// 1. 定义代价函数结构体
struct ResistanceFuncLinear {
    // 成员变量：存储目标位置和子弹速度
    const double target_d_; // 水平距离
    const double target_h_; // 垂直高度
    const double v0_;       // 初速度

    // 构造函数：初始化这些常数
    ResistanceFuncLinear(double d, double h, double v0) 
        : target_d_(d), target_h_(h), v0_(v0) {}

    // 2. 核心计算逻辑：Ceres 会不断调用这个函数来评估当前角度好不好
    template <typename T>
    bool operator()(const T* const angle, T* residual) const {
        T theta = angle[0];
        
        // --- 物理模型 (带空气阻力) ---
        // 速度分量
        T v_x = T(v0_) * ceres::cos(theta);
        T v_y = T(v0_) * ceres::sin(theta);
        
        // 计算到达目标水平距离 d 所需的时间 t
        // x(t) = (vx0/k) * (1 - e^(-kt))  =>  t = -ln(1 - x*k/vx0) / k
        T k = T(K);
        T limit = T(1.0) - T(target_d_) * k / v_x;
        
        // 检查是否可能到达（如果在射程外，给一个大惩罚）
        if (limit <= T(0.0)) {
            residual[0] = T(100.0); 
            return true;
        }

        T t = -ceres::log(limit) / k;

        // 计算 t 时刻的垂直位置 yActual
        // y(t) = (vy0/k + g/k^2) * (1 - e^(-kt)) - g*t/k
        T y_actual = (v_y/k + T(GRAVITY)/(k*k)) * (1.0 - ceres::exp(-k * t)) - (T(GRAVITY)/k) * t;

        // 残差 = 计算高度 - 实际目标高度
        residual[0] = y_actual - T(target_h_);
        
        return true;
    }
};

Trajectory::Trajectory(const double v0, const double d, const double h)
{
    // 3. 构建优化问题
    double shoot_angle = std::atan2(h, d); // 初始猜测：直瞄角度
    
    ceres::Problem problem;
    // 使用 new ResistanceFuncLinear(d, h, v0) 创建代价函数对象
    // 注意这里传入的是 v0, d, h，而不是之前的 bs, target_xy
    problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<ResistanceFuncLinear, 1, 1>(
            new ResistanceFuncLinear(d, h, v0) 
        ),
        nullptr,
        &shoot_angle
    );

    ceres::Solver::Options options;
    options.max_num_iterations = 25;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false;
    
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // 4. 保存结果
    if (summary.termination_type == ceres::CONVERGENCE || summary.final_cost < 1e-3) {
        this->unsolvable = false;
        this->pitch = shoot_angle;
        
        // 重新计算 flight time
        double cos_theta = std::cos(shoot_angle);
        double limit = 1.0 - d * K / (v0 * cos_theta);
        if(limit > 0)
             this->fly_time = -std::log(limit) / K;
        else
             this->fly_time = 0; // Should not happen if converged
             
    } else {
        this->unsolvable = true;
        this->pitch = 0;
        this->fly_time = 0;
    }}
}//namespace tools