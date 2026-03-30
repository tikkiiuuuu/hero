#include "planner.hpp"

#include <vector>

#include "tools/math_tools.hpp"
#include "tools/trajectory.hpp"
#include "tools/yaml.hpp"

#include <ruckig/ruckig.hpp>
#include <ruckig/trajectory.hpp>
#include <algorithm>

using namespace std::chrono_literals;

namespace auto_aim
{
Planner::Planner(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  yaw_offset_ = tools::read<double>(yaml, "yaw_offset") / 57.3;
  pitch_offset_ = tools::read<double>(yaml, "pitch_offset") / 57.3;
  fire_thresh_ = tools::read<double>(yaml, "fire_thresh");

  yaw_fire_thresh_=tools::read<double>(yaml, "yaw_fire_thresh");
  pitch_fire_thresh_=tools::read<double>(yaml, "pitch_fire_thresh");

  decision_speed_ = tools::read<double>(yaml, "decision_speed");
  high_speed_delay_time_ = tools::read<double>(yaml, "high_speed_delay_time");
  low_speed_delay_time_ = tools::read<double>(yaml, "low_speed_delay_time");
  comm_delay_=tools::read<double>(yaml,"comm_delay");

  setup_yaw_solver(config_path);
  setup_pitch_solver(config_path);

  auto max_yaw_acc = tools::read<double>(yaml, "max_yaw_acc");
  auto max_pitch_acc = tools::read<double>(yaml, "max_pitch_acc");
  auto max_yaw_vel = tools::read<double>(yaml, "max_yaw_vel");
  auto max_pitch_vel = tools::read<double>(yaml, "max_pitch_vel");
  auto max_yaw_jerk = tools::read<double>(yaml, "max_yaw_jerk");
  auto max_pitch_jerk = tools::read<double>(yaml, "max_pitch_jerk");
  
  // 设置约束
  ruckig_input_.max_velocity[0] = max_yaw_vel;       
  ruckig_input_.max_acceleration[0] = max_yaw_acc;
  ruckig_input_.max_jerk[0] = max_yaw_jerk; 
  ruckig_input_.max_velocity[1] = max_pitch_vel;       
  ruckig_input_.max_acceleration[1] = max_pitch_acc;
  ruckig_input_.max_jerk[1] = max_pitch_jerk; 
  ruckig_input_.duration_discretization = ruckig::DurationDiscretization::Continuous;
  ruckig_input_.synchronization = ruckig::Synchronization::Time; 

  shoot_offset_ = tools::read<int>(yaml, "shoot_offset");
}

Planner::Planner(const std::string & config_path,const int ifdouble)
{
  auto yaml = tools::load(config_path);
  yaw_offset_ = tools::read<double>(yaml, "yaw_offset") / 57.3;
  pitch_offset_ = tools::read<double>(yaml, "pitch_offset") / 57.3;
  fire_thresh_ = tools::read<double>(yaml, "fire_thresh");
  decision_speed_ = tools::read<double>(yaml, "decision_speed");
  high_speed_delay_time_ = tools::read<double>(yaml, "high_speed_delay_time");
  low_speed_delay_time_ = tools::read<double>(yaml, "low_speed_delay_time");
  rho_ = tools::read<double>(yaml, "rho");
  iter_time_ = tools::read<int>(yaml, "iter_time");
  setup_doubleyaw_solver(config_path);
  setup_pitch_solver(config_path);
}

// Plan Planner::plan(Target target, double bullet_speed)
// {
//   // 0. Check bullet speed
//   if (bullet_speed < 10 || bullet_speed > 25) {
//     bullet_speed = 22;
//   }

//   // 1. Predict fly_time
//   Eigen::Vector3d xyz;
//   auto min_dist = 1e10;
//   for (auto & xyza : target.armor_xyza_list()) {
//     auto dist = xyza.head<2>().norm();
//     if (dist < min_dist) {
//       min_dist = dist;
//       xyz = xyza.head<3>();
//     }
//   }
//   auto bullet_traj = tools::Trajectory(bullet_speed, min_dist, xyz.z());
//   target.predict(bullet_traj.fly_time);
//   // target.predict(10e-6);

//   // 2. Get trajectory
//   double yaw0;
//   Trajectory traj;
//   try {
//     yaw0 = aim(target, bullet_speed)(0);
//     traj = get_trajectory(target, yaw0, bullet_speed);
//   } catch (const std::exception & e) {
//     tools::logger()->warn("Unsolvable target {:.2f}", bullet_speed);
//     return {false};
//   }

//   // 3. Solve yaw
//   Eigen::VectorXd x0(2);
//   x0 << traj(0, 0), traj(1, 0);
//   tiny_set_x0(yaw_solver_, x0);

//   yaw_solver_->work->Xref = traj.block(0, 0, 2, HORIZON);
//   tiny_solve(yaw_solver_);

//   // 4. Solve pitch
//   x0 << traj(2, 0), traj(3, 0);
//   tiny_set_x0(pitch_solver_, x0);

//   pitch_solver_->work->Xref = traj.block(2, 0, 2, HORIZON);
//   tiny_solve(pitch_solver_);

//   Plan plan;
//   plan.control = true;

//   plan.target_yaw = tools::limit_rad(traj(0, HALF_HORIZON) + yaw0);
//   plan.target_pitch = traj(2, HALF_HORIZON);

//   plan.yaw = tools::limit_rad(yaw_solver_->work->x(0, HALF_HORIZON) + yaw0);
//   plan.yaw_vel = yaw_solver_->work->x(1, HALF_HORIZON);
//   plan.yaw_acc = yaw_solver_->work->u(0, HALF_HORIZON);

//   plan.pitch = pitch_solver_->work->x(0, HALF_HORIZON);
//   plan.pitch_vel = pitch_solver_->work->x(1, HALF_HORIZON);
//   plan.pitch_acc = pitch_solver_->work->u(0, HALF_HORIZON);

//   auto shoot_offset_ = 2;
//   plan.fire =
//     std::hypot(
//       traj(0, HALF_HORIZON + shoot_offset_) - yaw_solver_->work->x(0, HALF_HORIZON + shoot_offset_),
//       traj(2, HALF_HORIZON + shoot_offset_) -
//         pitch_solver_->work->x(0, HALF_HORIZON + shoot_offset_)) < fire_thresh_;
//   return plan;
// }

// Plan Planner::plan(std::optional<Target> target, double bullet_speed)
// {
//   if (!target.has_value()) return {false};

//   double delay_time =
//     std::abs(target->get_state()[Target::state::w]) > decision_speed_ ? high_speed_delay_time_ : low_speed_delay_time_;

//   auto future = std::chrono::steady_clock::now() + std::chrono::microseconds(int(delay_time * 1e6));

//   target->predict(future);

//   return plan(*target, bullet_speed);
// }

/************************************************************************************************************************
 * @brief 之前的plan分开,很屎，现在合并了。同时把predict(dt)也换为了future,绝对时间
 * 
 * @param target 
 * @param bullet_speed 
 * @return Plan 
***********************************************************************************************************************/

Plan Planner::plan(RWTracker::ShowTargetInterface & target, double bullet_speed, double send_time)
{


  double delay_time =
    std::abs(target.get_state()[Target::state::w]) > decision_speed_ ? high_speed_delay_time_ : low_speed_delay_time_;   //delay_time是高速小陀螺下的电控机械的各种延时，包括拨弹延时
  // auto future = std::chrono::steady_clock::now() + std::chrono::microseconds(int(delay_time * 1e6));
    //非高速小陀螺状态下 不考虑拨弹延时 固定电控通信延时

  bool is_spinning = std::abs(target.get_state()[Target::state::w]) > decision_speed_;
  

  // return plan(*target, bullet_speed);

  // 0. Check bullet speed
  if (bullet_speed < 10 || bullet_speed > 25) {
    bullet_speed = 11.5;
  }

  // 1. Predict fly_time
  Eigen::Vector3d xyz;
  auto min_dist = 1e10;
  for (auto & xyza : target.armor_xyza_list()) {
    auto dist = xyza.head<2>().norm();
    if (dist < min_dist) {
      min_dist = dist;
      xyz = xyza.head<3>();
    }
  }
  auto bullet_traj = tools::Trajectory(bullet_speed, min_dist, xyz.z());
  // target.predict(future + std::chrono::microseconds(int(bullet_traj.fly_time * 1e6)));
  //? 这里不知道为什么之前写成了 target.tracker_.dt_ = send_time + low_speed_delay_time_ + bullet_traj.fly_time;
  //TODO 现在改为这样，待验证

  if(is_spinning) { target.tracker_.dt_ = bullet_traj.fly_time + delay_time;}
  else { target.tracker_.dt_ = bullet_traj.fly_time + comm_delay_;}

  // target.predict(delay_time + bullet_traj.fly_time);
  target.state_ = target.ekf_.predict();
  // target.predict(10e-6);

  // 2. Get trajectory
  double yaw0;
  Trajectory traj;
  try {
    yaw0 = aim(target, bullet_speed)(0);
    traj = get_trajectory(target, yaw0, bullet_speed);
  } catch (const std::exception & e) {
    tools::logger()->warn("Unsolvable target {:.2f}", bullet_speed);
    return {false};
  }

  // 3. Solve yaw
  Eigen::VectorXd x0(2);
  x0 << traj(0, 0), traj(1, 0);
  tiny_set_x0(yaw_solver_, x0);

  yaw_solver_->work->Xref = traj.block(0, 0, 2, HORIZON);
  tiny_solve(yaw_solver_);

  // 4. Solve pitch
  x0 << traj(2, 0), traj(3, 0);
  tiny_set_x0(pitch_solver_, x0);

  pitch_solver_->work->Xref = traj.block(2, 0, 2, HORIZON);
  tiny_solve(pitch_solver_);

  Plan plan;
  plan.control = true;

  plan.target_yaw = tools::limit_rad(traj(0, HALF_HORIZON) + yaw0);
  plan.target_pitch = traj(2, HALF_HORIZON);

  plan.yaw = tools::limit_rad(yaw_solver_->work->x(0, HALF_HORIZON) + yaw0);
  plan.yaw_vel = yaw_solver_->work->x(1, HALF_HORIZON);
  plan.yaw_acc = yaw_solver_->work->u(0, HALF_HORIZON);

  plan.pitch = pitch_solver_->work->x(0, HALF_HORIZON);
  plan.pitch_vel = pitch_solver_->work->x(1, HALF_HORIZON);
  plan.pitch_acc = pitch_solver_->work->u(0, HALF_HORIZON);

  //auto shoot_offset_ = 2;
  // plan.fire =
  //   std::hypot(
  //     traj(0, HALF_HORIZON + shoot_offset_) - yaw_solver_->work->x(0, HALF_HORIZON + shoot_offset_),
  //     traj(2, HALF_HORIZON + shoot_offset_) -
  //       pitch_solver_->work->x(0, HALF_HORIZON + shoot_offset_)) < fire_thresh_;

  int min_shoot_offset = 0; 
  int max_shoot_offset = 10; 
  
  if(is_spinning) { max_shoot_offset = 2;}  //高速小陀螺 窄窗口
  

  bool can_fire = true;
  double max_window_yaw_err = 0.0;
  double max_window_pitch_err = 0.0;

  for (int offset = min_shoot_offset; offset <= max_shoot_offset; ++offset) {
      // 确保不越界
      if (HALF_HORIZON + offset >= HORIZON) break; 

      double yaw_err = std::abs(traj(0, HALF_HORIZON + offset) - yaw_solver_->work->x(0, HALF_HORIZON + offset));
      double pitch_err = std::abs(traj(2, HALF_HORIZON + offset) - pitch_solver_->work->x(0, HALF_HORIZON + offset));
      
      // 记录窗口内的最大误差
      max_window_yaw_err = std::max(max_window_yaw_err, yaw_err);
      max_window_pitch_err = std::max(max_window_pitch_err, pitch_err);

      plan.max_window_pitch_err=max_window_pitch_err;
      plan.max_window_yaw_err=max_window_yaw_err;
      if (yaw_err > yaw_fire_thresh_ || pitch_err > pitch_fire_thresh_) {
          can_fire = false;
          break; // 只要在这个时间窗口内有任何一刻误差超标，就不开火
      }
  }
  plan.fire = can_fire;

  // plan.fire= (traj(0, HALF_HORIZON + shoot_offset_) - yaw_solver_->work->x(0, HALF_HORIZON + shoot_offset_)<yaw_fire_thresh_) &&
  //            (traj(2, HALF_HORIZON + shoot_offset_) - pitch_solver_->work->x(0, HALF_HORIZON + shoot_offset_)<pitch_fire_thresh_);

  return plan;
}

Eigen::VectorXd Planner::predict_state(const Eigen::VectorXd & current_state, double dt) const
{

  Eigen::VectorXd x_prior = current_state;
  if(x_prior.size() < 11) return x_prior;

  // 将 [xc,vx,yc,vy,za_1,vz,theta,omega,r1,za_2,r2] 按线性运动学向前更新 
  x_prior[0] += x_prior[1] * dt; // xc += vx * dt
  x_prior[2] += x_prior[3] * dt; // yc += vy * dt
  x_prior[4] += x_prior[5] * dt; // za_1 += vz * dt
  x_prior[6] += x_prior[7] * dt; // theta += omega * dt
  x_prior[9] += x_prior[5] * dt; // za_2 += vz * dt (za2与za1共享z轴速度)

  // 角度限幅归一化
  x_prior[6] = tools::limit_rad(x_prior[6]);

  return x_prior;
}

// std::vector<Eigen::Vector4d> Planner::compute_armor_xyza(const Eigen::VectorXd & state_x, int armor_num) const
// {
//   std::vector<Eigen::Vector4d> res;
//     // ... 这里将目标车辆中心点 + 半径 r 和 theta 推算出每个装甲板的三维坐标
//     // (逻辑与原生 target.armor_xyza_list() 完全一致，但从传入参数 state_x 中读取)
//       std::vector<Eigen::Vector4d> _armor_xyza_list;

//   for (int i = 0; i < armor_num_; i++) {
//     auto angle = tools::limit_rad(get_state()[state::a] + i * 2 * CV_PI / armor_num_);
//     Eigen::Vector3d xyz = h_armor_xyz(i);
//     _armor_xyza_list.push_back({xyz[0], xyz[1], xyz[2], angle});
//   }
//   return res;
// }

// // 计算出装甲板中心的坐标（考虑长短轴）
// Eigen::Vector3d Target::h_armor_xyz( int id) const
// {
//   const auto& filter_state = get_state();
//   auto angle = tools::limit_rad(filter_state[state::a] + id * 2 * CV_PI / armor_num_);
//   auto use_l_h = (armor_num_ == 4) && (id == 1 || id == 3);

//   auto r = (use_l_h) ? filter_state[state::r] + filter_state[state::l] : filter_state[state::r];
//   auto armor_x = filter_state[state::x] - r * std::cos(angle);
//   auto armor_y = filter_state[state::y] - r * std::sin(angle);
//   auto armor_z = (use_l_h) ? filter_state[state::z] + filter_state[state::h] : filter_state[state::z];

//   return {armor_x, armor_y, armor_z};
// }

std::vector<Eigen::Vector4d> Planner::compute_armor_xyza(const Eigen::VectorXd & state_x, int armor_num, int circle_index) const
{
    std::vector<Eigen::Vector4d> res;
    double xc = state_x[0];
    double yc = state_x[2];
    double theta = state_x[6];

    for (int i = 0; i < armor_num; i++) {
        double armor_theta = tools::limit_rad(theta + i * 2 * CV_PI / armor_num);
        double r, z;
        // 如果是4块装甲板且是侧板
        if (armor_num == 4 && i % 2 == 1) {
            r = state_x[10]; // r2
            z = state_x[9];  // za2
        } else if (armor_num == 3) {
            r = state_x[8];
            z = state_x[4];
            int circle_type[3] = {1,0,-1};
            z += 0.1 * circle_type[(circle_index + i) % 3];
        } else {
            r = state_x[8];  // r1
            z = state_x[4];  // za1
        }
        
        // 参考 rw_tracker.cpp 的转换逻辑：xc = xa - r * cos(yaw)  => xa = xc + r * cos(yaw)
        double armor_x = xc + r * std::cos(armor_theta);
        double armor_y = yc + r * std::sin(armor_theta);

        res.push_back({armor_x, armor_y, z, armor_theta});
    }
    return res;
}

// Eigen::Matrix<double, 2, 1> Planner::aim_from_state(const Eigen::VectorXd & state_x, int armor_num, double bullet_speed)
// {
//     // 利用算出的装甲板，得到最优 yaw / pitch
//     auto armors = compute_armor_xyza(state_x, armor_num);
//     Eigen::Vector3d xyz;
//     double yaw;
//     double min_dist = 1e10;

//     for (auto & a : armors) {
//         if (a.head<2>().norm() < min_dist) {
//             min_dist = a.head<2>().norm();
//             xyz = a.head<3>();
//             yaw = a[3];
//         }
//     }
//     debug_xyza = Eigen::Vector4d(xyz.x(), xyz.y(), xyz.z(), yaw);

//     auto azim = std::atan2(xyz.y(), xyz.x());
//     auto bullet_traj = tools::Trajectory(bullet_speed, min_dist, xyz.z());
//     if (bullet_traj.unsolvable) throw std::runtime_error("Unsolvable bullet trajectory!");

//     return {tools::limit_rad(azim + yaw_offset_), -bullet_traj.pitch - pitch_offset_};

// }

Eigen::Matrix<double, 2, 1> Planner::aim_from_state(
  const Eigen::VectorXd & state_x,
  int armor_num,
  double bullet_speed)
{
  auto armor_list = compute_armor_xyza(state_x, armor_num);

  // -------------------------------
  // 点积法：选出最正对摄像头的装甲板
  // -------------------------------
  double best_score = -1e9;
  int best_id = -1;

  for (int i = 0; i < armor_num; i++) {
      const auto &xyza = armor_list[i];

      double ax = xyza[0];
      double ay = xyza[1];
      double az = xyza[2];
      double yaw = xyza[3];

      // 摄像头在世界系原点
      Eigen::Vector2d v(ax, ay);
      double dist = v.norm();
      if (dist < 1e-6) continue;
      Eigen::Vector2d v_hat = v / dist;

      // 装甲板前向向量
      Eigen::Vector2d f(std::cos(yaw), std::sin(yaw));

      // 点积
      double score = f.dot(-v_hat);

      if (score > best_score) {
          best_score = score;
          best_id = i;
      }
  }

  if (best_id < 0)
      throw std::runtime_error("No armor selected!");

  // -------------------------------
  // 选中的装甲板（世界系）
  // -------------------------------
  const auto &best_xyza = armor_list[best_id];
  Eigen::Vector3d xyz(best_xyza[0], best_xyza[1], best_xyza[2]);
  double yaw = best_xyza[3];

  // 供外部绘图使用（世界系）
  debug_xyza = Eigen::Vector4d(xyz.x(), xyz.y(), xyz.z(), yaw);

  // -------------------------------
  // 计算瞄准角
  // -------------------------------
  double min_dist = xyz.head<2>().norm();
  double azim = std::atan2(xyz.y(), xyz.x());

  auto bullet_traj = tools::Trajectory(bullet_speed, min_dist, xyz.z());
  if (bullet_traj.unsolvable)
      throw std::runtime_error("Unsolvable bullet trajectory!");

  return {
      tools::limit_rad(azim + yaw_offset_),
      -bullet_traj.pitch - pitch_offset_
  };
}


Trajectory Planner::get_trajectory_from_state(const Eigen::VectorXd & target_state, int armor_num, double yaw0, double bullet_speed)
{
    Trajectory traj;
    // 核心改变：用副本变量 state_copy 替代 EKF
    Eigen::VectorXd state_copy = predict_state(target_state, -DT * (HALF_HORIZON + 1));
    auto yaw_pitch_last = aim_from_state(state_copy, armor_num, bullet_speed);

    state_copy = predict_state(state_copy, 2*DT); // [0] = -HALF_HORIZON * DT
    auto yaw_pitch = aim_from_state(state_copy, armor_num, bullet_speed);

    for (int i = 0; i < HORIZON; i++) {
        // 利用自定义推演函数，干干净净向前滚时间
        state_copy = predict_state(state_copy, DT);
        auto yaw_pitch_next = aim_from_state(state_copy, armor_num, bullet_speed);

        auto yaw_vel = tools::limit_rad(yaw_pitch_next(0) - yaw_pitch_last(0)) / (2 * DT);
        auto pitch_vel = (yaw_pitch_next(1) - yaw_pitch_last(1)) / (2 * DT);

        traj.col(i) << tools::limit_rad(yaw_pitch(0) - yaw0), yaw_vel, yaw_pitch(1), pitch_vel;

        yaw_pitch_last = yaw_pitch;
        yaw_pitch = yaw_pitch_next;
    }
    return traj;
}

Plan Planner::plan_1(const Eigen::VectorXd& tracker_state, int armors_num, double bullet_speed, double send_time, int circle_index){

  bool is_spinning = std::abs(tracker_state[7]) > decision_speed_;
  double delay_time = is_spinning ? high_speed_delay_time_ : low_speed_delay_time_;   //delay_time是高速小陀螺下的电控机械的各种延时，包括拨弹延时
  // auto future = std::chrono::steady_clock::now() + std::chrono::microseconds(int(delay_time * 1e6));
    //非高速小陀螺状态下 不考虑拨弹延时 固定电控通信延时


    bool is_converged = true;
      //旋转半径明显不符合物理实际(假设步兵常理半径在0.15~0.5m之间)，说明未收敛
  if (tracker_state.size() > 10 && armors_num == 4) {
    double r1 = tracker_state[8];
    double r2 = tracker_state[10];
    if (r1 < 0.15 || r1 > 0.50 || r2 < 0.15 || r2 > 0.50) {
        is_converged = false;
    }
}

   // 1. 完全基于副本推演：先补充系统延迟时间
  double initial_dt = send_time + comm_delay_+delay_time;
  Eigen::VectorXd current_state = predict_state(tracker_state, initial_dt);

  ////获得整车中心并发送角度
  Plan plan{}; // 初始化全部置零防止野指针
  plan.control = true;
  plan.yaw = tools::limit_rad(aim_from_state_2(current_state, armors_num, bullet_speed, circle_index)(0));
  plan.pitch = aim_from_state_2(current_state, armors_num, bullet_speed, circle_index)(1);
  plan.fire = false;


   //判断开火时机
  //考虑子弹飞行，机械电控延时，

  auto armors = compute_armor_xyza(current_state, armors_num, circle_index);
  double min_dist = 1e10;  double target_z = 0;
  for(auto &a : armors) { if(a.head<2>().norm() < min_dist) { min_dist = a.head<2>().norm(); target_z = a.z(); } }

  auto bullet_traj = tools::Trajectory(bullet_speed, min_dist, target_z);

  current_state = predict_state(current_state, bullet_traj.fly_time);
  auto hit_armors = compute_armor_xyza(current_state, armors_num, circle_index);


  //遍历四块装甲板的状态，其其恰好到达对应的yaw，pitch
for(int i= 0;i<armors_num;i++){

        // 直接根据击中时的该块装甲板坐标计算其所需的 pitch 和 yaw
      double a_x = hit_armors[i][0];
      double a_y = hit_armors[i][1];
      double a_z = hit_armors[i][2];

          //避免强行打切角大于60度的装甲板产生跳弹
      Eigen::Vector2d v(a_x, a_y);
      Eigen::Vector2d f(std::cos(hit_armors[i][3]), std::sin(hit_armors[i][3]));
      if (f.dot(-v / v.norm()) < 0.5) continue; // 约cos(60度)=0.5, 切角太差直接过滤

      double a_dist = std::hypot(a_x, a_y);
      double yaw_i = tools::limit_rad(std::atan2(a_y, a_x) + yaw_offset_);

      auto armor_traj = tools::Trajectory(bullet_speed, a_dist, a_z);
      if (armor_traj.unsolvable) continue;

      double pitch_i = -armor_traj.pitch - pitch_offset_;

      if (is_converged && 
          std::abs(plan.yaw - yaw_i) < yaw_fire_thresh_ && 
          std::abs(plan.pitch - pitch_i) < pitch_fire_thresh_)
      { plan.fire=true; }

    }
    return plan;
  
  }


Plan Planner::plan_2(const Eigen::VectorXd& tracker_state, int armors_num, double bullet_speed, double send_time, int circle_index)
{
  bool is_spinning = std::abs(tracker_state[7]) > decision_speed_;
  double delay_time = is_spinning ? high_speed_delay_time_ : low_speed_delay_time_;   //delay_time是高速小陀螺下的电控机械的各种延时，包括拨弹延时

  // 1. 基于副本推演：先补充系统硬件与通讯延迟时间，推演子弹即将出膛时的状态
  double initial_dt = send_time + comm_delay_ + delay_time;
  Eigen::VectorXd current_state = predict_state(tracker_state, initial_dt);

  // 2. 拿到此时的装甲板坐标，估算目标距离从而获得子弹飞行时间
  auto armors_now = compute_armor_xyza(current_state, armors_num, circle_index);
  double min_dist = 1e10;
  double target_z = 0;
  for(auto &a : armors_now) {
      if(a.head<2>().norm() < min_dist) {
          min_dist = a.head<2>().norm();
          target_z = a.z();
      }
  }

  auto bullet_traj = tools::Trajectory(bullet_speed, min_dist, target_z);

  // 从现在到子弹命中的总时间 = 发送与系统延时 + 子弹飞行时间
  auto time_all = bullet_traj.fly_time + initial_dt;

  // 3. 彻底推演出子弹命中时的车辆状态（添加了平移提前量）
  Eigen::VectorXd hit_state = predict_state(tracker_state, time_all);

  Plan plan{}; // 初始化置零
  plan.control = true;
  plan.fire = false; // 默认不开火

  // 4. 调用 aim_from_state_2 瞄准整车中心（遵循整车中心策略）
  try {
      Eigen::Matrix<double, 2, 1> aim_result = aim_from_state_2(hit_state, armors_num, bullet_speed, circle_index);
      plan.yaw = aim_result(0);
      plan.pitch = aim_result(1);

      // 在平移模式下，如果弹道规划成功且没有报出 unsolvable，就可以直接开火
      // 因为我们是在持续追瞄目标预测的落点。
      plan.fire = true;
  } catch (const std::exception & e) {
      tools::logger()->warn("Translation Plan Unsolvable trajectory at hit stage!");
  }

  return plan;
}

Eigen::Matrix<double, 2, 1> Planner::aim_from_state_2(
  const Eigen::VectorXd & state_x,
  int armor_num,
  double bullet_speed,
  int circle_index)
{
  // -------------------------------
  // 从整车中心计算
  // -------------------------------
  double xc = state_x[0];
  double yc = state_x[2];

  // Pitch取得平均高度值
  double avg_z = 0.0;
  if (armor_num == 4 && state_x.size() > 9) {
      avg_z = (state_x[4] + state_x[9]) / 2.0; // za_1 和 za_2 的平均
  } else if (armor_num == 3) {
      // 前哨站：三块装甲板高度分别为 z+0.1, z, z-0.1
      // 平均高度 = (z+0.1 + z + z-0.1) / 3 = z，正好是中心高度
      avg_z = state_x[4];
  } else {
      avg_z = state_x[4]; // 只有 za_1
  }

  // 供外部绘图使用（世界系，整车中心态）
  debug_xyza = Eigen::Vector4d(xc, yc, avg_z, state_x[6]);

  // -------------------------------
  // 计算瞄准角（Yaw正对整车中心）
  // -------------------------------
  double center_dist = std::hypot(xc, yc);
  double azim = std::atan2(yc, xc);

  auto bullet_traj = tools::Trajectory(bullet_speed, center_dist, avg_z);
  if (bullet_traj.unsolvable)
      throw std::runtime_error("Unsolvable bullet trajectory!");

  return {
      tools::limit_rad(azim + yaw_offset_),
      -bullet_traj.pitch - pitch_offset_
  };
}

Plan Planner::plan(const Eigen::VectorXd& tracker_state, int tracked_armors_num, double bullet_speed, double send_time)
{

  bool is_spinning = std::abs(tracker_state[7]) > decision_speed_;
  double delay_time = is_spinning ? high_speed_delay_time_ : low_speed_delay_time_;   //delay_time是高速小陀螺下的电控机械的各种延时，包括拨弹延时
  // auto future = std::chrono::steady_clock::now() + std::chrono::microseconds(int(delay_time * 1e6));
    //非高速小陀螺状态下 不考虑拨弹延时 固定电控通信延时

   // 1. 完全基于副本推演：先补充系统延迟时间
  double initial_dt = send_time + comm_delay_+delay_time;
  Eigen::VectorXd current_state = predict_state(tracker_state, initial_dt);

  // 2. 拿到目前的装甲板坐标并估算飞行时间
  auto armors = compute_armor_xyza(current_state, tracked_armors_num);
  double min_dist = 1e10;  double target_z = 0;
  for(auto &a : armors) { if(a.head<2>().norm() < min_dist) { min_dist = a.head<2>().norm(); target_z = a.z(); } }

  auto bullet_traj = tools::Trajectory(bullet_speed, min_dist, target_z);

  // if(is_spinning) { target.tracker_.dt_ = bullet_traj.fly_time + delay_time;}
  // else { target.tracker_.dt_ = bullet_traj.fly_time + comm_delay_;
  // target.predict(delay_time + bullet_traj.fly_time);
  // 3. 补充子弹飞行时间
  current_state = predict_state(current_state, bullet_traj.fly_time);


  // 2. Get trajectory
  double yaw0;
  Trajectory traj;
  try {
    yaw0 = aim_from_state(current_state, tracked_armors_num, bullet_speed)(0);
      traj = get_trajectory_from_state(current_state, tracked_armors_num, yaw0, bullet_speed);
  } catch (const std::exception & e) {
    tools::logger()->warn("Unsolvable target {:.2f}", bullet_speed);
    return {false};
  }

  // 3. Solve yaw
  Eigen::VectorXd x0(2);
  x0 << traj(0, 0), traj(1, 0);
  tiny_set_x0(yaw_solver_, x0);

  yaw_solver_->work->Xref = traj.block(0, 0, 2, HORIZON);
  tiny_solve(yaw_solver_);

  // 4. Solve pitch
  x0 << traj(2, 0), traj(3, 0);
  tiny_set_x0(pitch_solver_, x0);

  pitch_solver_->work->Xref = traj.block(2, 0, 2, HORIZON);
  tiny_solve(pitch_solver_);

  Plan plan;
  plan.control = true;

  plan.dist=min_dist;
  plan.target_yaw = tools::limit_rad(traj(0, HALF_HORIZON) + yaw0);
  plan.target_pitch = traj(2, HALF_HORIZON);

  plan.yaw = tools::limit_rad(yaw_solver_->work->x(0, HALF_HORIZON) + yaw0);
  plan.yaw_vel = yaw_solver_->work->x(1, HALF_HORIZON);
  plan.yaw_acc = yaw_solver_->work->u(0, HALF_HORIZON);

  plan.pitch = pitch_solver_->work->x(0, HALF_HORIZON);
  plan.pitch_vel = pitch_solver_->work->x(1, HALF_HORIZON);
  plan.pitch_acc = pitch_solver_->work->u(0, HALF_HORIZON);

  //auto shoot_offset_ = 2;
  // plan.fire =
  //   std::hypot(
  //     traj(0, HALF_HORIZON + shoot_offset_) - yaw_solver_->work->x(0, HALF_HORIZON + shoot_offset_),
  //     traj(2, HALF_HORIZON + shoot_offset_) -
  //       pitch_solver_->work->x(0, HALF_HORIZON + shoot_offset_)) < fire_thresh_;

  int min_shoot_offset = 0; 
  int max_shoot_offset = 10; 
  
  if(is_spinning) { max_shoot_offset = 2;}  //高速小陀螺 窄窗口
  

  bool can_fire = true;
  double max_window_yaw_err = 0.0;
  double max_window_pitch_err = 0.0;

  for (int offset = min_shoot_offset; offset <= max_shoot_offset; ++offset) {
      // 确保不越界
      if (HALF_HORIZON + offset >= HORIZON) break; 

      double yaw_err = std::abs(traj(0, HALF_HORIZON + offset) - yaw_solver_->work->x(0, HALF_HORIZON + offset));
      double pitch_err = std::abs(traj(2, HALF_HORIZON + offset) - pitch_solver_->work->x(0, HALF_HORIZON + offset));
      
      // 记录窗口内的最大误差
      max_window_yaw_err = std::max(max_window_yaw_err, yaw_err);
      max_window_pitch_err = std::max(max_window_pitch_err, pitch_err);

      plan.max_window_pitch_err=max_window_pitch_err;
      plan.max_window_yaw_err=max_window_yaw_err;
      if (yaw_err > yaw_fire_thresh_ || pitch_err > pitch_fire_thresh_) {
          can_fire = false;
          break; // 只要在这个时间窗口内有任何一刻误差超标，就不开火
      }
  }
  plan.fire = can_fire;

  // plan.fire= (traj(0, HALF_HORIZON + shoot_offset_) - yaw_solver_->work->x(0, HALF_HORIZON + shoot_offset_)<yaw_fire_thresh_) &&
  //            (traj(2, HALF_HORIZON + shoot_offset_) - pitch_solver_->work->x(0, HALF_HORIZON + shoot_offset_)<pitch_fire_thresh_);

  return plan;
}

Plan Planner::plan(std::optional<Target> target, double bullet_speed)
{
  if (!target.has_value()) return {false};

  double delay_time =
    std::abs(target->get_state()[Target::state::w]) > decision_speed_ ? high_speed_delay_time_ : low_speed_delay_time_;

  auto future = std::chrono::steady_clock::now() + std::chrono::microseconds(int(delay_time * 1e6));

  target->predict(future);

  // return plan(*target, bullet_speed);

  // 0. Check bullet speed
  if (bullet_speed < 10 || bullet_speed > 25) {
    bullet_speed = 12;
  }

  // 1. Predict fly_time
  Eigen::Vector3d xyz;
  auto min_dist = 1e10;
  for (auto & xyza : target->armor_xyza_list()) {
    auto dist = xyza.head<2>().norm();
    if (dist < min_dist) {
      min_dist = dist;
      xyz = xyza.head<3>();
    }
  }
  auto bullet_traj = tools::Trajectory(bullet_speed, min_dist, xyz.z());
  target->predict(future + std::chrono::microseconds(int(bullet_traj.fly_time * 1e6)));
  // target.predict(10e-6);

  // 2. Get trajectory
  double yaw0;
  Trajectory traj;
  try {
    yaw0 = aim(*target, bullet_speed)(0);
    traj = get_trajectory(* target, yaw0, bullet_speed);
  } catch (const std::exception & e) {
    tools::logger()->warn("Unsolvable target {:.2f}", bullet_speed);
    return {false};
  }

  // 3. Solve yaw
  Eigen::VectorXd x0(2);
  x0 << traj(0, 0), traj(1, 0);
  tiny_set_x0(yaw_solver_, x0);

  yaw_solver_->work->Xref = traj.block(0, 0, 2, HORIZON);
  tiny_solve(yaw_solver_);

  // 4. Solve pitch
  x0 << traj(2, 0), traj(3, 0);
  tiny_set_x0(pitch_solver_, x0);

  pitch_solver_->work->Xref = traj.block(2, 0, 2, HORIZON);
  tiny_solve(pitch_solver_);

  Plan plan;
  plan.control = true;

  plan.target_yaw = tools::limit_rad(traj(0, HALF_HORIZON) + yaw0);
  plan.target_pitch = traj(2, HALF_HORIZON);

  plan.yaw = tools::limit_rad(yaw_solver_->work->x(0, HALF_HORIZON) + yaw0);
  plan.yaw_vel = yaw_solver_->work->x(1, HALF_HORIZON);
  plan.yaw_acc = yaw_solver_->work->u(0, HALF_HORIZON);

  plan.pitch = pitch_solver_->work->x(0, HALF_HORIZON);
  plan.pitch_vel = pitch_solver_->work->x(1, HALF_HORIZON);
  plan.pitch_acc = pitch_solver_->work->u(0, HALF_HORIZON);

  auto shoot_offset_ = 2;
  plan.fire =
    std::hypot(
      traj(0, HALF_HORIZON + shoot_offset_) - yaw_solver_->work->x(0, HALF_HORIZON + shoot_offset_),
      traj(2, HALF_HORIZON + shoot_offset_) -
        pitch_solver_->work->x(0, HALF_HORIZON + shoot_offset_)) < fire_thresh_;
  return plan;
}

void Planner::plan_double(std::optional<Target> target, double bullet_speed)
{
  if (!target.has_value()) plan_double_.control = false;

  double delay_time =
    std::abs(target->get_state()[Target::state::w]) > decision_speed_ ? high_speed_delay_time_ : low_speed_delay_time_;

  auto future = std::chrono::steady_clock::now() + std::chrono::microseconds(int(delay_time * 1e6));

  target->predict(future);
  plan_double(*target, bullet_speed);
  // return plan_double(*target, bullet_speed);
}

void Planner::plan_double(Target target, double bullet_speed)
{
  // 0. Check bullet speed
  if (bullet_speed < 10 || bullet_speed > 25) {
    bullet_speed = 22;
  }

  // 1. Predict fly_time
  Eigen::Vector3d xyz;
  auto min_dist = 1e10;
  for (auto & xyza : target.armor_xyza_list()) {
    auto dist = xyza.head<2>().norm();
    if (dist < min_dist) {
      min_dist = dist;
      xyz = xyza.head<3>();
    }
  }
  auto bullet_traj = tools::Trajectory(bullet_speed, min_dist, xyz.z());
  target.predict(bullet_traj.fly_time);

  // 2. Get trajectory
  double yaw0;
  Trajectory traj;
  try {
    yaw0 = aim(target, bullet_speed)(0);
    traj = get_trajectory(target, yaw0, bullet_speed);
  } catch (const std::exception & e) {
    tools::logger()->warn("Unsolvable target {:.2f}", bullet_speed);
    plan_double_.control = false;
  }

  // 3. Solve yaw
  Eigen::VectorXd x0(3);
  x0 << traj(0, 0), traj(0, 0)*0.5, traj(0, 0)*0.5;
  tiny_set_x0(yaw_solver_, x0);

  yaw_solver_->work->Xref = Eigen::MatrixXd::Zero(3,HORIZON);
  yaw_solver_->work->Xref.block(0,0,1, HORIZON) = traj.block(0, 0, 1, HORIZON);
  yaw_solver_->work->Xref.block(0,0,1, HORIZON) = traj.block(0, 0, 1, HORIZON);
  // yaw_solver_->work->Xref.block(0,0,1, HORIZON) = traj.block(0, 0, 1, HORIZON)*0.7;
  // yaw_solver_->work->Xref.block(0,0,1, HORIZON) = traj.block(0, 0, 1, HORIZON)*0.3;
  tiny_solve(yaw_solver_);

  // 4. Solve pitch
  Eigen::VectorXd x01(2);
  x01 << traj(2, 0), traj(3, 0);
  tiny_set_x0(pitch_solver_, x01);

  pitch_solver_->work->Xref = traj.block(2, 0, 2, HORIZON);
  tiny_solve(pitch_solver_);

  plan_double_.control = true;

  plan_double_.target_yaw = tools::limit_rad(traj(0, HALF_HORIZON) + yaw0);
  plan_double_.target_pitch = traj(2, HALF_HORIZON);

  plan_double_.yaw = tools::limit_rad(yaw_solver_->work->x(0, HALF_HORIZON) + yaw0);
  plan_double_.yaw_vel = yaw_solver_->work->u(0, HALF_HORIZON)+yaw_solver_->work->u(1, HALF_HORIZON);
  // plan_double.yaw_acc = yaw_solver_->work->u(0, HALF_HORIZON);
  plan_double_.yaw_acc = yaw_solver_->work->u(2, HALF_HORIZON)+yaw_solver_->work->u(3, HALF_HORIZON);

  plan_double_.yaw_big = tools::limit_rad(yaw_solver_->work->x(1, HALF_HORIZON) + yaw0);
  plan_double_.yaw_vel_big = yaw_solver_->work->u(0, HALF_HORIZON);
  plan_double_.yaw_acc_big = yaw_solver_->work->u(2, HALF_HORIZON);

  plan_double_.yaw_small = tools::limit_rad(yaw_solver_->work->x(2, HALF_HORIZON) + yaw0);
  plan_double_.yaw_vel_small = yaw_solver_->work->u(1, HALF_HORIZON);
  plan_double_.yaw_acc_small = yaw_solver_->work->u(3, HALF_HORIZON);

  plan_double_.pitch = pitch_solver_->work->x(0, HALF_HORIZON);
  plan_double_.pitch_vel = pitch_solver_->work->x(1, HALF_HORIZON);
  plan_double_.pitch_acc = pitch_solver_->work->u(0, HALF_HORIZON);

  auto shoot_offset_ = 2;
  plan_double_.fire =
    std::hypot(
      traj(0, HALF_HORIZON + shoot_offset_) - yaw_solver_->work->x(0, HALF_HORIZON + shoot_offset_),
      traj(2, HALF_HORIZON + shoot_offset_) -
        pitch_solver_->work->x(0, HALF_HORIZON + shoot_offset_)) < fire_thresh_;

  // // 3. Solve yaw
  // Eigen::VectorXd x0(6);
  // // if(plan_double_.control){
  // //   x0 << traj(0, 0), plan_double_.yaw_big, plan_double_.yaw_small, traj(1,0), plan_double_.yaw_vel_big, plan_double_.yaw_vel_small;
  // // }else{
  // //   x0 << traj(0, 0), 0, 0, traj(1,0), 0, 0;
  // // }
  // x0 << traj(0, 0), 0, 0, traj(1,0), 0, 0;
  // tiny_set_x0(yaw_solver_, x0);

  // yaw_solver_->work->Xref = Eigen::MatrixXd::Zero(6,HORIZON);
  // yaw_solver_->work->Xref.block(0,0,1, HORIZON) = traj.block(0, 0, 1, HORIZON);
  // yaw_solver_->work->Xref.block(3,0,1, HORIZON) = traj.block(1, 0, 1, HORIZON);
  // // yaw_solver_->work->Xref.block(0,0,1, HORIZON) = traj.block(0, 0, 1, HORIZON)*0.7;
  // // yaw_solver_->work->Xref.block(0,0,1, HORIZON) = traj.block(0, 0, 1, HORIZON)*0.3;
  // tiny_solve(yaw_solver_);

  // // 4. Solve pitch
  // Eigen::VectorXd x01(2);
  // x01 << traj(2, 0), traj(3, 0);
  // tiny_set_x0(pitch_solver_, x01);

  // pitch_solver_->work->Xref = traj.block(2, 0, 2, HORIZON);
  // tiny_solve(pitch_solver_);

  // plan_double_.control = true;

  // plan_double_.target_yaw = tools::limit_rad(traj(0, HALF_HORIZON) + yaw0);
  // plan_double_.target_pitch = traj(2, HALF_HORIZON);
  // plan_double_.target_yaw_vel = traj(1, HALF_HORIZON);

  // plan_double_.yaw = tools::limit_rad(yaw_solver_->work->x(0, HALF_HORIZON) + yaw0);
  // plan_double_.yaw_vel = yaw_solver_->work->x(3, HALF_HORIZON);
  // // plan_double.yaw_acc = yaw_solver_->work->u(0, HALF_HORIZON);
  // plan_double_.yaw_acc = yaw_solver_->work->u(0, HALF_HORIZON)+yaw_solver_->work->u(1, HALF_HORIZON);

  // plan_double_.yaw_big = tools::limit_rad(yaw_solver_->work->x(1, HALF_HORIZON) + yaw0);
  // plan_double_.yaw_vel_big = yaw_solver_->work->x(4, HALF_HORIZON);
  // plan_double_.yaw_acc_big = yaw_solver_->work->u(0, HALF_HORIZON);

  // plan_double_.yaw_small = tools::limit_rad(yaw_solver_->work->x(2, HALF_HORIZON) + yaw0);
  // plan_double_.yaw_vel_small = yaw_solver_->work->x(5, HALF_HORIZON);
  // plan_double_.yaw_acc_small = yaw_solver_->work->u(1, HALF_HORIZON);

  // plan_double_.pitch = pitch_solver_->work->x(0, HALF_HORIZON);
  // plan_double_.pitch_vel = pitch_solver_->work->x(1, HALF_HORIZON);
  // plan_double_.pitch_acc = pitch_solver_->work->u(0, HALF_HORIZON);

  // auto shoot_offset_ = 2;
  // plan_double_.fire =
  //   std::hypot(
  //     traj(0, HALF_HORIZON + shoot_offset_) - yaw_solver_->work->x(0, HALF_HORIZON + shoot_offset_),
  //     traj(2, HALF_HORIZON + shoot_offset_) -
  //       pitch_solver_->work->x(0, HALF_HORIZON + shoot_offset_)) < fire_thresh_;
  // // return &plan_double_;
}

void Planner::setup_yaw_solver(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto max_yaw_acc = tools::read<double>(yaml, "max_yaw_acc");
  auto Q_yaw = tools::read<std::vector<double>>(yaml, "Q_yaw");
  auto R_yaw = tools::read<std::vector<double>>(yaml, "R_yaw");

  Eigen::MatrixXd A{{1, DT}, {0, 1}};
  Eigen::MatrixXd B{{0}, {DT}};
  Eigen::VectorXd f{{0, 0}};
  Eigen::Matrix<double, 2, 1> Q(Q_yaw.data());
  Eigen::Matrix<double, 1, 1> R(R_yaw.data());
  tiny_setup(&yaw_solver_, A, B, f, Q.asDiagonal(), R.asDiagonal(), 1.0, 2, 1, HORIZON, 0);

  Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(2, HORIZON, -1e17);
  Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(2, HORIZON, 1e17);
  Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(1, HORIZON - 1, -max_yaw_acc);
  Eigen::MatrixXd u_max = Eigen::MatrixXd::Constant(1, HORIZON - 1, max_yaw_acc);
  tiny_set_bound_constraints(yaw_solver_, x_min, x_max, u_min, u_max);

  yaw_solver_->settings->max_iter = 10;
}

void Planner::setup_doubleyaw_solver(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto max_big_yaw_acc = tools::read<double>(yaml, "max_big_yaw_acc");
  auto max_small_yaw_acc = tools::read<double>(yaml, "max_small_yaw_acc");
  auto Q_yaw = tools::read<std::vector<double>>(yaml, "Q_yaw");
  auto R_yaw = tools::read<std::vector<double>>(yaml, "R_yaw");
  //clang-format off
  Eigen::MatrixXd A{{0, 1, 1}, \
                    {0, 1, 0}, \
                    {0, 0, 1}  };
  
  Eigen::MatrixXd B{{ 0, 0,  0,  0}, \
                    { 1, 0, DT,  0}, \
                    { 0, 1,  0, DT}};
  //clang-format on
  Eigen::VectorXd f{{0, 0, 0}};
  Eigen::Matrix<double, 3, 1> Q(Q_yaw.data());
  Eigen::Matrix<double, 4, 1> R(R_yaw.data());
  tiny_setup(&yaw_solver_, A, B, f, Q.asDiagonal(), R.asDiagonal(), 1.0, 3, 4, HORIZON, 0);

  Eigen::MatrixXd x_max(3, HORIZON);
  x_max.row(0).setConstant(1e17); // No limit on angle
  x_max.row(1).setConstant(1e17);  // No limit on angle1
  x_max.row(2).setConstant(M_PI/6);  // limit on angle2

  Eigen::MatrixXd x_min(3, HORIZON);
  x_min.row(0).setConstant(-1e17); // No limit on angle
  x_min.row(1).setConstant(-1e17); // Min velocity 1
  x_min.row(2).setConstant(-M_PI/6); // Min velocity 2

  Eigen::MatrixXd u_max(4, HORIZON - 1);
  u_max.row(0).setConstant(0.01); // Max velocity 1  
  u_max.row(1).setConstant(1.0);  // Max velocity 2
  u_max.row(2).setConstant(max_big_yaw_acc);  // Limit for u(0)
  u_max.row(3).setConstant(max_small_yaw_acc); // Limit for u(1)

  Eigen::MatrixXd u_min(4, HORIZON - 1);
  u_min.row(0).setConstant(-0.01); // Max velocity 1  
  u_min.row(1).setConstant(-1.0);  // Max velocity 2
  u_min.row(2).setConstant(-max_big_yaw_acc); // Limit for u(0)
  u_min.row(3).setConstant(-max_small_yaw_acc); // Limit for u(1)
  tiny_set_bound_constraints(yaw_solver_, x_min, x_max, u_min, u_max);

  // 添加线性等式约束：yaw - yaw_big - yaw_small = 0
  // 转换为两个不等式：
  // yaw - yaw_big - yaw_small <= 0  和  yaw - yaw_big - yaw_small >= 0
  
  Eigen::MatrixXd Alin_x(2, 3);  // 2个约束，3个状态变量
  Alin_x << 1, -1, -1,   // yaw - yaw_big - yaw_small <= 0
           -1,  1,  1;   // -yaw + yaw_big + yaw_small <= 0
  
  Eigen::VectorXd blin_x(2);
  blin_x << 0, 0;  // 右侧都是 0
  
  // 没有控制输入的线性约束
  Eigen::MatrixXd Alin_u(0, 2);  // 0个约束
  Eigen::VectorXd blin_u(0);
  
  tiny_set_linear_constraints(yaw_solver_, Alin_x, blin_x, Alin_u, blin_u);

  // 启用线性约束
  yaw_solver_->settings->en_state_linear = 1;
  yaw_solver_->settings->en_input_linear = 0;
  yaw_solver_->settings->en_state_bound = 1;
  yaw_solver_->settings->max_iter = iter_time_;
  yaw_solver_->settings->abs_pri_tol = 1e-6;



  // //clang-format off
  // Eigen::MatrixXd A{{1, 0, 0, 0, DT, DT}, 
  //                   {0, 1, 0, 0, DT, 0}, 
  //                   {0, 0, 1, 0, 0, DT},
  //                   {0, 0, 0, 0, 1, 1},
  //                   {0, 0, 0, 0, 1, 0},
  //                   {0, 0, 0, 0, 0, 1}};
  
  // Eigen::MatrixXd B{{ 0, 0},
  //                   { 0, 0},
  //                   { 0, 0},
  //                   { 0, 0},
  //                   { DT, 0},
  //                   { 0, DT}};
  // //clang-format on
  // Eigen::VectorXd f{{0, 0, 0, 0, 0, 0}};
  // Eigen::Matrix<double, 6, 1> Q(Q_yaw.data());
  // Eigen::Matrix<double, 2, 1> R(R_yaw.data());
  // tiny_setup(&yaw_solver_, A, B, f, Q.asDiagonal(), R.asDiagonal(), rho_, 6, 2, HORIZON, 1);

  // Eigen::MatrixXd x_max(6, HORIZON);
  // x_max.row(0).setConstant(1e5); 
  // x_max.row(1).setConstant(1e5); 
  // x_max.row(2).setConstant(M_PI/6);
  // x_max.row(3).setConstant(1e5);
  // x_max.row(4).setConstant(1e5);
  // x_max.row(5).setConstant(1e5);

  // Eigen::MatrixXd x_min(6, HORIZON);
  // x_min.row(0).setConstant(-1e5);
  // x_min.row(1).setConstant(-1e5); 
  // x_min.row(2).setConstant(-M_PI/6);
  // x_min.row(3).setConstant(-1e5);
  // x_min.row(4).setConstant(-1e5);
  // x_min.row(5).setConstant(-1e5);

  // Eigen::MatrixXd u_max(2, HORIZON - 1);
  // u_max.row(0).setConstant(max_big_yaw_acc); 
  // u_max.row(1).setConstant(max_small_yaw_acc); 

  // Eigen::MatrixXd u_min(2, HORIZON - 1);
  // u_min.row(0).setConstant(-max_big_yaw_acc); 
  // u_min.row(1).setConstant(-max_small_yaw_acc); 
  // tiny_set_bound_constraints(yaw_solver_, x_min, x_max, u_min, u_max);

  // // // 添加线性等式约束：yaw - yaw_big - yaw_small = 0
  // // // 转换为两个不等式：
  // // // yaw - yaw_big - yaw_small <= 0  和  yaw - yaw_big - yaw_small >= 0
  
  // // Eigen::MatrixXd Alin_x(2, 3);  // 2个约束，3个状态变量
  // // Alin_x << 1, -1, -1,   // yaw - yaw_big - yaw_small <= 0
  // //          -1,  1,  1;   // -yaw + yaw_big + yaw_small <= 0
  
  // // Eigen::VectorXd blin_x(2);
  // // blin_x << 0, 0;  // 右侧都是 0
  
  // // // 没有控制输入的线性约束
  // // Eigen::MatrixXd Alin_u(0, 2);  // 0个约束
  // // Eigen::VectorXd blin_u(0);
  
  // // tiny_set_linear_constraints(yaw_solver_, Alin_x, blin_x, Alin_u, blin_u);

  // Eigen::MatrixXd Alin_x(4, 6);  // 改为3个约束
  // Alin_x << 1, -1, -1, 0, 0, 0,    // x[0] - x[1] - x[2] <= 1e-17  (上界)
  //         -1,  1,  1, 0, 0, 0,    // -x[0] + x[1] + x[2] <= 1e-17 (下界，即 x[0] - x[1] - x[2] >= -1e-17)
  //           0, 0, 0, -1, 1, 1,     // -x[3] + x[4] + x[5] <= 1e-17 (速度约束)
  //           0, 0, 0, 1, -1, -1;   
  // Eigen::VectorXd blin_x(4);
  // blin_x << 1e-5, 1e-5, 1e-5, 1e-5;
  // tiny_set_linear_constraints(yaw_solver_, Alin_x, blin_x, Eigen::MatrixXd::Zero(0, 6), Eigen::VectorXd::Zero(0));

  // yaw_solver_->settings->abs_pri_tol = 1e-4;
  // yaw_solver_->settings->en_state_linear = 1;
  // yaw_solver_->settings->en_input_linear = 1;
  // yaw_solver_->settings->en_state_bound = 1;
  // yaw_solver_->settings->en_input_bound = 1;
  // yaw_solver_->settings->max_iter = iter_time_; 
  // // yaw_solver_->settings->adaptive_rho = 1;
}

void Planner::setup_pitch_solver(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto max_pitch_acc = tools::read<double>(yaml, "max_pitch_acc");
  auto Q_pitch = tools::read<std::vector<double>>(yaml, "Q_pitch");
  auto R_pitch = tools::read<std::vector<double>>(yaml, "R_pitch");

  Eigen::MatrixXd A{{1, DT}, {0, 1}};
  Eigen::MatrixXd B{{0}, {DT}};
  Eigen::VectorXd f{{0, 0}};
  Eigen::Matrix<double, 2, 1> Q(Q_pitch.data());
  Eigen::Matrix<double, 1, 1> R(R_pitch.data());
  tiny_setup(&pitch_solver_, A, B, f, Q.asDiagonal(), R.asDiagonal(), 1.0, 2, 1, HORIZON, 0);

  Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(2, HORIZON, -1e17);
  Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(2, HORIZON, 1e17);
  Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(1, HORIZON - 1, -max_pitch_acc);
  Eigen::MatrixXd u_max = Eigen::MatrixXd::Constant(1, HORIZON - 1, max_pitch_acc);
  tiny_set_bound_constraints(pitch_solver_, x_min, x_max, u_min, u_max);

  pitch_solver_->settings->max_iter = 10;
}

Eigen::Matrix<double, 2, 1> Planner::aim(const Target & target, double bullet_speed)
{
  Eigen::Vector3d xyz;
  double yaw;
  auto min_dist = 1e10;

  for (auto & xyza : target.armor_xyza_list()) {
    auto dist = xyza.head<2>().norm();
    if (dist < min_dist) {
      min_dist = dist;
      xyz = xyza.head<3>();
      yaw = xyza[3];
    }
  }
  debug_xyza = Eigen::Vector4d(xyz.x(), xyz.y(), xyz.z(), yaw);

  auto azim = std::atan2(xyz.y(), xyz.x());
  auto bullet_traj = tools::Trajectory(bullet_speed, min_dist, xyz.z());
  if (bullet_traj.unsolvable) throw std::runtime_error("Unsolvable bullet trajectory!");

  return {tools::limit_rad(azim + yaw_offset_), -bullet_traj.pitch - pitch_offset_};
}

Trajectory Planner::get_trajectory(Target & target, double yaw0, double bullet_speed)
{
  Trajectory traj;

  target.predict(-DT * (HALF_HORIZON + 1));
  auto yaw_pitch_last = aim(target, bullet_speed);

  target.predict(DT);  // [0] = -HALF_HORIZON * DT -> [HHALF_HORIZON] = 0
  auto yaw_pitch = aim(target, bullet_speed);

  for (int i = 0; i < HORIZON; i++) {
    target.predict(DT);
    auto yaw_pitch_next = aim(target, bullet_speed);

    auto yaw_vel = tools::limit_rad(yaw_pitch_next(0) - yaw_pitch_last(0)) / (2 * DT);
    auto pitch_vel = (yaw_pitch_next(1) - yaw_pitch_last(1)) / (2 * DT);

    traj.col(i) << tools::limit_rad(yaw_pitch(0) - yaw0), yaw_vel, yaw_pitch(1), pitch_vel;

    yaw_pitch_last = yaw_pitch;
    yaw_pitch = yaw_pitch_next;
  }

  return traj;
}

/************************************************************************************************************************
 * @brief 新观测器关于target接口的重构
 * 
***********************************************************************************************************************/
Plan Planner::plan(RWTracker::ShowTargetInterface & target, double bullet_speed)
{
  target.init_ekf();
  double original_dt = target.tracker_.dt_;
  //! 这里太难崩了，直接给private注释了
  if (!target.tracker_.matched && target.tracker_.tracker_state != RWTracker::TrackState::TEMP_LOST) return {false};

  double delay_time =
    std::abs(target.get_state()[Target::state::w]) > decision_speed_ ? high_speed_delay_time_ : low_speed_delay_time_;
  //? 不知道为什么高速的delay甚至更小，感觉同济也没搞懂，可能是高速的响应范围小？
  // auto future = std::chrono::steady_clock::now() + std::chrono::microseconds(int(delay_time * 1e6));

  target.tracker_.dt_ = delay_time;
  target.state_ = target.ekf_.predict();

  // return plan(*target, bullet_speed);

  // 0. Check bullet speed
  if (bullet_speed < 10 || bullet_speed > 25) {
    bullet_speed = 11.5;
  }

  // 1. Predict fly_time
  Eigen::Vector3d xyz;
  auto min_dist = 1e10;
  for (auto & xyza : target.armor_xyza_list()) {
    auto dist = xyza.head<2>().norm();
    if (dist < min_dist) {
      min_dist = dist;
      xyz = xyza.head<3>();
    }
  }
  auto bullet_traj = tools::Trajectory(bullet_speed, min_dist, xyz.z());
  // target.predict(future + std::chrono::microseconds(int(bullet_traj.fly_time * 1e6)));
  target.tracker_.dt_ = delay_time + bullet_traj.fly_time;
  // target.predict(delay_time + bullet_traj.fly_time);
  target.state_ = target.ekf_.predict();
  // target.predict(10e-6);

  // 2. Get trajectory
  double yaw0;
  Trajectory traj;
  try {
    yaw0 = aim(target, bullet_speed)(0);
    traj = get_trajectory(target, yaw0, bullet_speed);
  } catch (const std::exception & e) {
    tools::logger()->warn("Unsolvable target {:.2f}", bullet_speed);
    return {false};
  }

  // 3. Solve yaw
  Eigen::VectorXd x0(2);
  x0 << traj(0, 0), traj(1, 0);
  tiny_set_x0(yaw_solver_, x0);

  yaw_solver_->work->Xref = traj.block(0, 0, 2, HORIZON);
  tiny_solve(yaw_solver_);

  // 4. Solve pitch
  x0 << traj(2, 0), traj(3, 0);
  tiny_set_x0(pitch_solver_, x0);

  pitch_solver_->work->Xref = traj.block(2, 0, 2, HORIZON);
  tiny_solve(pitch_solver_);

  Plan plan;
  plan.control = true;

  plan.target_yaw = tools::limit_rad(traj(0, HALF_HORIZON) + yaw0);
  plan.target_pitch = traj(2, HALF_HORIZON);

  plan.yaw = tools::limit_rad(yaw_solver_->work->x(0, HALF_HORIZON) + yaw0);
  plan.yaw_vel = yaw_solver_->work->x(1, HALF_HORIZON);
  plan.yaw_acc = yaw_solver_->work->u(0, HALF_HORIZON);

  plan.pitch = pitch_solver_->work->x(0, HALF_HORIZON);
  plan.pitch_vel = pitch_solver_->work->x(1, HALF_HORIZON);
  plan.pitch_acc = pitch_solver_->work->u(0, HALF_HORIZON);

  auto shoot_offset_ = 2;
  plan.fire =
    std::hypot(
      traj(0, HALF_HORIZON + shoot_offset_) - yaw_solver_->work->x(0, HALF_HORIZON + shoot_offset_),
      traj(2, HALF_HORIZON + shoot_offset_) -
        pitch_solver_->work->x(0, HALF_HORIZON + shoot_offset_)) < fire_thresh_;
  target.tracker_.dt_ = original_dt;
  return plan;
}

Eigen::Matrix<double, 2, 1>
Planner::aim(const RWTracker::ShowTargetInterface& target, double bullet_speed) {
    Eigen::Vector3d xyz;
    double yaw;
    auto min_dist = 1e10;

    for (auto& xyza: target.armor_xyza_list()) {
        auto dist = xyza.head<2>().norm();
        if (dist < min_dist) {
            min_dist = dist;
            xyz = xyza.head<3>();
            yaw = xyza[3];
        }
    }
    debug_xyza = Eigen::Vector4d(xyz.x(), xyz.y(), xyz.z(), yaw);

    auto azim = std::atan2(xyz.y(), xyz.x());
    auto bullet_traj = tools::Trajectory(bullet_speed, min_dist, xyz.z());
    if (bullet_traj.unsolvable)
        throw std::runtime_error("Unsolvable bullet trajectory!");

    return { tools::limit_rad(azim + yaw_offset_), -bullet_traj.pitch - pitch_offset_ };
}

[[deprecated]]Trajectory Planner::get_trajectory(RWTracker::ShowTargetInterface & target, double yaw0, double bullet_speed)
{
  Trajectory traj;
  target.tracker_.dt_ = -DT * (HALF_HORIZON + 1);
  target.state_ = target.ekf_.predict();
  auto yaw_pitch_last = aim(target, bullet_speed);

  target.tracker_.dt_ = DT;  // [0] = -HALF_HORIZON * DT -> [HHALF_HORIZON] = 0
  target.state_ = target.ekf_.predict();
  auto yaw_pitch = aim(target, bullet_speed);

  for (int i = 0; i < HORIZON; i++) {
    //*注意 target.tracker_.dt_ = DT;
    target.state_ = target.ekf_.predict();
    auto yaw_pitch_next = aim(target, bullet_speed);

    auto yaw_vel = tools::limit_rad(yaw_pitch_next(0) - yaw_pitch_last(0)) / (2 * DT);
    auto pitch_vel = (yaw_pitch_next(1) - yaw_pitch_last(1)) / (2 * DT);

    traj.col(i) << tools::limit_rad(yaw_pitch(0) - yaw0), yaw_vel, yaw_pitch(1), pitch_vel;

    yaw_pitch_last = yaw_pitch;
    yaw_pitch = yaw_pitch_next;
  }

  return traj;
}
/************************************************************************************************************************
 * @brief 重构结束
 * 
***********************************************************************************************************************/


// 1. 检测阶跃点的轨迹生成函数
[[deprecated]]std::pair<Trajectory, int> Planner::get_trajectory_with_jump_detection(
  RWTracker::ShowTargetInterface & target, double yaw0, double bullet_speed)
{
Trajectory traj;
int jump_idx = -1;  // 阶跃点索引，-1表示无阶跃点

target.tracker_.dt_ = -DT * (HALF_HORIZON + 1);
target.state_ = target.ekf_.predict();
auto yaw_pitch_last = aim(target, bullet_speed);

target.tracker_.dt_ = DT;
target.state_ = target.ekf_.predict();
auto yaw_pitch = aim(target, bullet_speed);

// 用于检测阶跃点的阈值
const double jump_threshold_yaw = 0.3;  // 弧度，可根据实际情况调整
const double jump_threshold_vel = 5.0;  // rad/s，可根据实际情况调整

for (int i = 0; i < HORIZON; i++) {
  target.tracker_.dt_ = DT;
  target.state_ = target.ekf_.predict();
  auto yaw_pitch_next = aim(target, bullet_speed);

  auto yaw_vel = tools::limit_rad(yaw_pitch_next(0) - yaw_pitch_last(0)) / (2 * DT);
  auto pitch_vel = (yaw_pitch_next(1) - yaw_pitch_last(1)) / (2 * DT);

  traj.col(i) << tools::limit_rad(yaw_pitch(0) - yaw0), yaw_vel, 
                  yaw_pitch(1), pitch_vel;

  // 检测阶跃点：检查位置或速度的突变
  if (i > 0 && jump_idx == -1) {
    double yaw_diff = std::abs(traj(0, i) - traj(0, i-1));
    double vel_diff = std::abs(traj(1, i) - traj(1, i-1));
    
    // 如果位置或速度变化超过阈值，认为是阶跃点
    if (yaw_diff > jump_threshold_yaw || vel_diff > jump_threshold_vel) {
      jump_idx = i;  // 记录阶跃点位置
    }
  }

  yaw_pitch_last = yaw_pitch;
  yaw_pitch = yaw_pitch_next;
}

return {traj, jump_idx};
}

// 2. 在阶跃点两侧搜索起点和终点，生成ruckig轨迹
std::pair<bool, int>
Planner::ruckig_search_transition(const Trajectory& traj, int jump_idx, int search_window) {
    // search_window: 搜索窗口大小，表示在阶跃点两侧各搜索多少个点

    // 确定搜索范围
    std::cout << "jump_idx: " << jump_idx << std::endl;
    int start_search_begin = std::max(0, jump_idx - search_window);
    int start_search_end = jump_idx - 1;
    int end_search_begin = jump_idx + 1;
    int end_search_end = std::min(HORIZON - 1, jump_idx + search_window);

    // 如果阶跃点在窗口边界，无法搜索
    if (jump_idx < 0 || jump_idx >= HORIZON) {
        return { false, -1 };
    }

    // 从阶跃点向前搜索起点，向后搜索终点
    // 策略：找到满足加速度约束的最大过渡段
    int best_start_idx = jump_idx - 1;
    int best_end_idx = jump_idx + 1;
    bool found_valid = false;

    // 尝试不同的起点和终点组合
    //? 这里先遍历搜索，验证之后再改为二分搜索
    for (int offset = 0; offset <= search_window && jump_idx - offset >= 0; offset++) {
        int start_idx = jump_idx - offset;
        int end_idx = jump_idx + offset;

        if (start_idx >= end_idx)
            continue;

        // 设置ruckig输入
        ruckig::InputParameter<2> current_input = ruckig_input_;

        // 起点状态
        current_input.current_position[0] = traj(0, start_idx);
        current_input.current_velocity[0] = traj(1, start_idx);
        current_input.current_position[1] = traj(2, start_idx);
        current_input.current_velocity[1] = traj(3, start_idx);

        // 终点状态
        current_input.target_position[0] = traj(0, end_idx);
        current_input.target_velocity[0] = traj(1, end_idx);
        current_input.target_position[1] = traj(2, end_idx);
        current_input.target_velocity[1] = traj(3, end_idx);

        // 最小过渡时间
        double min_duration = (end_idx - start_idx) * DT;
        current_input.minimum_duration = min_duration;

        // 验证输入
        if (!ruckig_yaw_pitch_.validate_input(current_input)) {
            continue;
        }

        // 计算轨迹
        std::cout << "start_idx" << start_idx << std::endl;
        std::cout << "end_idx" << end_idx << std::endl;
        ruckig::Trajectory<2> test_trajectory;
        auto result = ruckig_yaw_pitch_.calculate(current_input, test_trajectory);

        if (result != ruckig::Result::Working && result != ruckig::Result::Finished) {
            continue;
        }

        // 检查轨迹是否满足加速度约束
        double max_acc_yaw = 0.0;
        double max_acc_pitch = 0.0;
        const int sample_points = 50;
        double duration = test_trajectory.get_duration();

        bool valid = true;
        for (int i = 0; i <= sample_points; ++i) {
            double t = (duration * i) / sample_points;
            std::array<double, 2> pos, vel, acc;
            test_trajectory.at_time(t, pos, vel, acc);

            max_acc_yaw = std::max(max_acc_yaw, std::abs(acc[0]));
            max_acc_pitch = std::max(max_acc_pitch, std::abs(acc[1]));

            // 检查是否超过最大加速度（留一点余量）
            if (max_acc_yaw > ruckig_input_.max_acceleration[0] + 1e-3
                || max_acc_pitch > ruckig_input_.max_acceleration[1] + 1e-3)
            {
                valid = false;
                continue;
            }
        }

        if (valid) {
            // 找到满足约束的轨迹，优先选择更大的过渡段
            if (!found_valid || (end_idx - start_idx) > (best_end_idx - best_start_idx)) {
                best_start_idx = start_idx;
                best_end_idx = end_idx;
                ruckig_trajectory_ = test_trajectory;
                ruckig_input_ = current_input;
                found_valid = true;
            }
        }
    }

    if (found_valid) {
        return { true, best_start_idx }; // 返回成功标志和起点索引
    }

    return { false, -1 };
}

// 3. 主规划函数
[[deprecated]]Plan Planner::plan_with_ruckig(
  RWTracker::ShowTargetInterface & target, 
  double bullet_speed, 
  double send_time)
{
target.init_ekf();
double original_dt = target.tracker_.dt_;

if (!target.tracker_.matched && 
    target.tracker_.tracker_state != RWTracker::TrackState::TEMP_LOST) {
  return {false};
}

// 预测到发送时间
target.tracker_.dt_ = send_time + low_speed_delay_time_;
target.state_ = target.ekf_.predict();

// 检查子弹速度
if (bullet_speed < 10 || bullet_speed > 25) {
  bullet_speed = 22;
}

// 计算飞行时间并预测目标
Eigen::Vector3d xyz;
auto min_dist = 1e10;
for (auto & xyza : target.armor_xyza_list()) {
  auto dist = xyza.head<2>().norm();
  if (dist < min_dist) {
    min_dist = dist;
    xyz = xyza.head<3>();
  }
}
auto bullet_traj = tools::Trajectory(bullet_speed, min_dist, xyz.z());
target.tracker_.dt_ = bullet_traj.fly_time;
target.state_ = target.ekf_.predict();

// 获取轨迹并检测阶跃点
double yaw0;
Trajectory traj;
int jump_idx = -1;
try {
  yaw0 = aim(target, bullet_speed)(0);
  std::tie(traj, jump_idx) = get_trajectory_with_jump_detection(target, yaw0, bullet_speed);
} catch (const std::exception & e) {
  tools::logger()->warn("Unsolvable target {:.2f}", bullet_speed);
  target.tracker_.dt_ = original_dt;
  return {false};
}

Plan plan;
plan.control = true;

std::array<double, 2> new_position, new_velocity, new_acceleration;

// 判断当前窗口内是否有阶跃点
// 窗口范围：[0, HORIZON-1]，当前时刻对应 HALF_HORIZON
bool use_ruckig = false;
int transition_start_idx = -1;

if (jump_idx >= 0 && jump_idx < HORIZON) {
  // 检查阶跃点是否在当前窗口内（可以定义窗口范围）
  // 这里假设窗口是 [HALF_HORIZON - window_size, HALF_HORIZON + window_size]
  const int window_size = HALF_HORIZON / 2;  // 窗口大小
  int window_start = std::max(0, HALF_HORIZON - window_size);
  int window_end = std::min(HORIZON - 1, HALF_HORIZON + window_size);
  
  if (jump_idx >= window_start && jump_idx <= window_end) {
    // 在阶跃点两侧搜索起点和终点
    const int search_window = std::min(20, HALF_HORIZON / 2);  // 搜索窗口大小
    // const int search_window = HALF_HORIZON;
    auto [success, start_idx] = ruckig_search_transition(traj, jump_idx, search_window);
    
    if (success) {
      use_ruckig = true;
      transition_start_idx = start_idx;
    }
  }
}

if (use_ruckig) {
  // 使用ruckig轨迹，取窗口中点（HALF_HORIZON对应的时间点）
  // 计算相对于轨迹起点的时间
  double relative_time = (HALF_HORIZON - transition_start_idx) * DT;
  double trajectory_duration = ruckig_trajectory_.get_duration();
  
  // 确保时间在轨迹范围内
  relative_time = std::clamp(relative_time, 0.0, trajectory_duration);
  
  ruckig_trajectory_.at_time(relative_time, new_position, new_velocity, new_acceleration);
  
  plan.yaw = tools::limit_rad(new_position[0] + yaw0);
  plan.yaw_vel = new_velocity[0];
  plan.yaw_acc = new_acceleration[0];
  
  plan.pitch = new_position[1];
  plan.pitch_vel = new_velocity[1];
  plan.pitch_acc = new_acceleration[1];
} else {
  // 没有阶跃点或ruckig失败，直接使用原始轨迹
  new_position[0] = traj(0, HALF_HORIZON);
  new_position[1] = traj(2, HALF_HORIZON);
  new_velocity[0] = traj(1, HALF_HORIZON);
  new_velocity[1] = traj(3, HALF_HORIZON);
  new_acceleration[0] = 0.0;
  new_acceleration[1] = 0.0;
  
  plan.yaw = tools::limit_rad(new_position[0] + yaw0);
  plan.yaw_vel = new_velocity[0];
  plan.yaw_acc = new_acceleration[0];
  
  plan.pitch = new_position[1];
  plan.pitch_vel = new_velocity[1];
  plan.pitch_acc = new_acceleration[1];
}

// 设置目标值（射击轨迹上的值）
plan.target_yaw = tools::limit_rad(traj(0, HALF_HORIZON) + yaw0);
plan.target_pitch = traj(2, HALF_HORIZON);

// 开火判断
if (use_ruckig) {
  // 计算过渡段终点与射击轨迹的差异
  double trajectory_duration = ruckig_trajectory_.get_duration();
  std::array<double, 2> end_pos, end_vel, end_acc;
  ruckig_trajectory_.at_time(trajectory_duration, end_pos, end_vel, end_acc);
  
  // 找到对应的终点索引
  int end_idx = transition_start_idx + static_cast<int>(trajectory_duration / DT);
  end_idx = std::clamp(end_idx, 0, HORIZON - 1);
  
  double yaw_diff = std::abs(traj(0, end_idx) - end_pos[0]);
  double pitch_diff = std::abs(traj(2, end_idx) - end_pos[1]);
  plan.fire = std::hypot(yaw_diff, pitch_diff) < fire_thresh_;
} else {
  // 直接跟随射击轨迹，可以开火
  plan.fire = true;
}

target.tracker_.dt_ = original_dt;
return plan;
}

// 在 planner.cpp 中实现

// 1. 适用于 Target 的阶跃点检测轨迹生成函数
//? python+gdb测试这里的阶越检测没有问题
//! 第三个是可搜索区间长度
std::tuple<Trajectory, int,int> Planner::get_trajectory_with_jump_detection(
  Target & target, double yaw0, double bullet_speed)
{
Trajectory traj;
int jump_idx = -1;  // 阶跃点索引，-1表示无阶跃点

target.predict(-DT * (HALF_HORIZON + 1));
auto yaw_pitch_last = aim(target, bullet_speed);

target.predict(DT);  // [0] = -HALF_HORIZON * DT -> [HALF_HORIZON] = 0
auto yaw_pitch = aim(target, bullet_speed);

std::vector<int> rank;

for (int i = 0; i < HORIZON; i++) {
  target.predict(DT);
  auto yaw_pitch_next = aim(target, bullet_speed);

  auto yaw_vel = tools::limit_rad(yaw_pitch_next(0) - yaw_pitch_last(0)) / (2 * DT);
  auto pitch_vel = (yaw_pitch_next(1) - yaw_pitch_last(1)) / (2 * DT);

  traj.col(i) << tools::limit_rad(yaw_pitch(0) - yaw0), yaw_vel, 
                  yaw_pitch(1), pitch_vel;
  if(((yaw_pitch_next(0) - yaw_pitch(0)) *
                  (yaw_pitch(0) - yaw_pitch_last(0))) < 0 ){
                    //* 这个特征不赖，26rad/s也稳定，可能idx会差+-1,但仍然是小量
    rank.push_back(i);
  }
  yaw_pitch_last = yaw_pitch;
  yaw_pitch = yaw_pitch_next;
  }
  int smaller;
  if (rank.empty()) {
    return {traj,-1,-1};
  } else {
    auto it = std::min_element(rank.begin(), rank.end(),
      [](int a, int b){ return std::abs(HALF_HORIZON - a) < std::abs(HALF_HORIZON - b); });
    jump_idx = *it;
    if(it==rank.begin()) {smaller = std::min(jump_idx,*(it+1)-jump_idx);}
    else if(it==rank.end()-1) {smaller = std::min(jump_idx-*(it-1),HORIZON-jump_idx);}
    else {smaller = std::min(jump_idx-*(it-1),*(it+1)-jump_idx);}
    return {traj, jump_idx, smaller};
  }
}

// 2. 适用于 Target 的主规划函数
Plan Planner::plan_with_ruckig(Target& target, double bullet_speed, double send_time) {
    // 计算延迟时间
    double delay_time = std::abs(target.get_state()[Target::state::w]) > decision_speed_
        ? high_speed_delay_time_
        : low_speed_delay_time_;

    // 预测到发送时间
    target.predict(send_time + delay_time);

    // 检查子弹速度
    if (bullet_speed < 10 || bullet_speed > 25) {
        bullet_speed = 22;
    }

    // 计算飞行时间并预测目标
    Eigen::Vector3d xyz;
    auto min_dist = 1e10;
    for (auto& xyza: target.armor_xyza_list()) {
        auto dist = xyza.head<2>().norm();
        if (dist < min_dist) {
            min_dist = dist;
            xyz = xyza.head<3>();
        }
    }
    auto bullet_traj = tools::Trajectory(bullet_speed, min_dist, xyz.z());
    target.predict(bullet_traj.fly_time);

    // 获取轨迹并检测阶跃点
    double yaw0;
    Trajectory traj;
    int jump_idx = -1;
    int length = -1;
    try {
        yaw0 = aim(target, bullet_speed)(0);
        std::tie(traj, jump_idx, length) =
            get_trajectory_with_jump_detection(target, yaw0, bullet_speed);
    } catch (const std::exception& e) {
        tools::logger()->warn("Unsolvable target {:.2f}", bullet_speed);
        return { false };
    }

    Plan plan;
    plan.control = true;

    std::array<double, 2> new_position, new_velocity, new_acceleration;

    // 判断当前窗口内是否有阶跃点
    // 窗口范围：[0, HORIZON-1]，当前时刻对应 HALF_HORIZON
    bool use_ruckig = false;
    int transition_start_idx = -1;

    if (jump_idx >= 0 && jump_idx < HORIZON) {
        // 检查阶跃点是否在当前窗口内（可以定义窗口范围）
        // 这里假设窗口是 [HALF_HORIZON - window_size, HALF_HORIZON + window_size]
        const int window_size = HALF_HORIZON / 2; // 窗口大小
        int window_start = std::max(0, HALF_HORIZON - window_size);
        int window_end = std::min(HORIZON - 1, HALF_HORIZON + window_size);

        if (jump_idx >= window_start && jump_idx <= window_end) {
            // 在阶跃点两侧搜索起点和终点
            auto [success, best_start_idx] = ruckig_search_transition(traj, jump_idx, length);

            if (success) {
                use_ruckig = true;
                transition_start_idx = best_start_idx;
            }
        }
    }

    Eigen::MatrixXd ruckig_vis_traj = Eigen::MatrixXd::Zero(4, HORIZON);
    ruckig_vis_traj.setZero();
    if (use_ruckig) {
        double r_duration = ruckig_trajectory_.get_duration();
        std::array<double, 2> p, v, a;
        
        // 获取起点状态 (t=0)
        std::array<double, 2> p_start, v_start, a_start;
        ruckig_trajectory_.at_time(0.0, p_start, v_start, a_start);

        // 获取终点状态 (t=duration)
        std::array<double, 2> p_end, v_end, a_end;
        ruckig_trajectory_.at_time(r_duration, p_end, v_end, a_end);

        for (int i = 0; i < HORIZON; ++i) {
            // 计算相对于 Ruckig 起点的时间
            double rel_t = (i - transition_start_idx) * DT;

            if (rel_t < 0) {
                // 在起点之前，保持起点状态（看起来是一条直线）
                p = p_start;
                v = v_start; 
                // 或者设为 0，取决于你想看静止还是保持初速度，通常保持位置不变速度为0更符合直觉，
                // 但如果想看完全连续，应该用 start 状态
            } 
            else if (rel_t > r_duration) {
                // 在终点之后，保持终点状态（看起来是一条直线）
                p = p_end;
                v = v_end;
            } 
            else {
                // 在 Ruckig 规划的时间范围内
                ruckig_trajectory_.at_time(rel_t, p, v, a);
            }
            ruckig_vis_traj(0, i) = p[0]; // Yaw 位置
            ruckig_vis_traj(1, i) = v[0]; // Yaw 速度
            ruckig_vis_traj(2, i) = p[1]; // Pitch 位置
            ruckig_vis_traj(3, i) = v[1]; // Pitch 速度
            if (ruckig_vis_traj(0, 0) > 1e10) std::cout<<"Prevent optimization\n";
        }
    }

    if (use_ruckig) {
        // 使用ruckig轨迹，取窗口中点（HALF_HORIZON对应的时间点）
        // 计算相对于轨迹起点的时间
        double relative_time = (HALF_HORIZON - transition_start_idx) * DT;
        double trajectory_duration = ruckig_trajectory_.get_duration();

        // 确保时间在轨迹范围内
        relative_time = std::clamp(relative_time, 0.0, trajectory_duration);

        ruckig_trajectory_.at_time(relative_time, new_position, new_velocity, new_acceleration);

        plan.yaw = tools::limit_rad(new_position[0] + yaw0);
        plan.yaw_vel = new_velocity[0];
        plan.yaw_acc = new_acceleration[0];

        plan.pitch = new_position[1];
        plan.pitch_vel = new_velocity[1];
        plan.pitch_acc = new_acceleration[1];
    } else {
        // 没有阶跃点或ruckig失败，直接使用原始轨迹
        new_position[0] = traj(0, HALF_HORIZON);
        new_position[1] = traj(2, HALF_HORIZON);
        new_velocity[0] = traj(1, HALF_HORIZON);
        new_velocity[1] = traj(3, HALF_HORIZON);
        new_acceleration[0] = 0.0;
        new_acceleration[1] = 0.0;

        plan.yaw = tools::limit_rad(new_position[0] + yaw0);
        plan.yaw_vel = new_velocity[0];
        plan.yaw_acc = new_acceleration[0];

        plan.pitch = new_position[1];
        plan.pitch_vel = new_velocity[1];
        plan.pitch_acc = new_acceleration[1];
    }

    // 设置目标值（射击轨迹上的值）
    plan.target_yaw = tools::limit_rad(traj(0, HALF_HORIZON) + yaw0);
    plan.target_pitch = traj(2, HALF_HORIZON);

    // 开火判断
    if (use_ruckig) {
        // 计算过渡段终点与射击轨迹的差异
        double trajectory_duration = ruckig_trajectory_.get_duration();
        std::array<double, 2> end_pos, end_vel, end_acc;
        ruckig_trajectory_.at_time(trajectory_duration, end_pos, end_vel, end_acc);

        // 找到对应的终点索引
        int end_idx = transition_start_idx + static_cast<int>(trajectory_duration / DT);
        end_idx = std::clamp(end_idx, 0, HORIZON - 1);

        double yaw_diff = std::abs(traj(0, end_idx) - end_pos[0]);
        double pitch_diff = std::abs(traj(2, end_idx) - end_pos[1]);
        plan.fire = std::hypot(yaw_diff, pitch_diff) < fire_thresh_;
    } else {
        // 直接跟随射击轨迹，可以开火
        plan.fire = true;
    }

    return plan;
}

}  // namespace auto_aim