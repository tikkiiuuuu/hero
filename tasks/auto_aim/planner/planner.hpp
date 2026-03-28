#ifndef AUTO_AIM__PLANNER_HPP
#define AUTO_AIM__PLANNER_HPP

#include <Eigen/Dense>
#include <list>
#include <optional>

#include "tasks/auto_aim/target.hpp"
#include "tinympc/tiny_api.hpp"

#include "tasks/auto_aim/rw_tracker.hpp"
#include <ruckig/ruckig.hpp>
// #include <ruckig/trackig.hpp> pro版本才有得用
namespace auto_aim
{
constexpr double DT = 0.01;
constexpr double DT1 = 0.001;
constexpr int HALF_HORIZON = 50;
constexpr int HORIZON = HALF_HORIZON * 2;

using  Trajectory = Eigen::Matrix<double, 4, HORIZON>;  // yaw, yaw_vel, pitch, pitch_vel

struct Plan
{
  bool control;
  bool fire;
  float target_yaw;
  float target_pitch;
  float yaw;
  float yaw_vel;
  float yaw_acc;
  float pitch;
  float pitch_vel;
  float pitch_acc;
  double max_window_yaw_err ;
  double max_window_pitch_err;
  double dist;
};

struct Plan_double
{
  bool control;
  bool fire;
  float target_yaw;
  float target_pitch;
  float target_yaw_vel;
  float yaw;
  float yaw_vel;
  float yaw_acc;
  float yaw_big;
  float yaw_vel_big;
  float yaw_acc_big;
  float yaw_small;
  float yaw_vel_small;
  float yaw_acc_small;
  float pitch;
  float pitch_vel;
  float pitch_acc;
};

class Planner
{
public:
  Eigen::Vector4d debug_xyza;
  explicit Planner(const std::string & config_path);
  Planner(const std::string & config_path,const int ifdouble);

  // Plan plan(Target target, double bullet_speed);
  Plan plan(std::optional<Target> target, double bullet_speed);
  Plan plan(RWTracker::ShowTargetInterface& target, double bullet_speed);
  Plan plan(RWTracker::ShowTargetInterface& target, double bullet_speed, double send_time);
  // Plan plan_with_ruckig(RWTracker::ShowTargetInterface& target, 
  //                                   double bullet_speed, 
  //                                   double send_time);
  // Plan plan_with_ruckig(Target & target, double bullet_speed, double send_time);
  void plan_double(std::optional<Target> target, double bullet_speed);
  void plan_double(Target target, double bullet_speed);

  // 在 planner.hpp 中添加以下函数声明
  std::pair<Trajectory, int> get_trajectory_with_jump_detection(
      RWTracker::ShowTargetInterface& target,
      double yaw0,
      double bullet_speed
  );
  std::pair<bool, int>
  ruckig_search_transition(const Trajectory& traj, int jump_idx, int search_window);
  Plan
  plan_with_ruckig(RWTracker::ShowTargetInterface& target, double bullet_speed, double send_time);
  Plan plan(const Eigen::VectorXd& tracker_state, int tracked_armors_num, double bullet_speed, double send_time);

  Plan plan_1(const Eigen::VectorXd& tracker_state, int armors_num, double bullet_speed, double send_time);
  Plan plan_2(const Eigen::VectorXd& tracker_state, int armors_num, double bullet_speed, double send_time);

  std::tuple<Trajectory, int,int>
  get_trajectory_with_jump_detection(Target& target, double yaw0, double bullet_speed);
  Plan plan_with_ruckig(Target& target, double bullet_speed, double send_time);

  Plan_double plan_double_;
  float rho_;
  int iter_time_;


  private:
  double yaw_offset_;
  double pitch_offset_;
  double fire_thresh_;
  double yaw_fire_thresh_,pitch_fire_thresh_;
  double low_speed_delay_time_, high_speed_delay_time_, decision_speed_;
  
  double max_window_yaw_err ;
  double max_window_pitch_err;
  double min_dist;
  double comm_delay_;

  TinySolver * yaw_solver_;
  TinySolver * pitch_solver_;

  void setup_yaw_solver(const std::string & config_path);
  void setup_doubleyaw_solver(const std::string & config_path);
  void setup_pitch_solver(const std::string & config_path);

  Eigen::Matrix<double, 2, 1> aim(const Target & target, double bullet_speed);
  Eigen::Matrix<double, 2, 1> aim(const RWTracker::ShowTargetInterface & target, double bullet_speed);
  Trajectory get_trajectory(Target & target, double yaw0, double bullet_speed);
  Trajectory get_trajectory(RWTracker::ShowTargetInterface & target, double yaw0, double bullet_speed);
  // std::pair<Trajectory, int> get_trajectory_with_rank(RWTracker::ShowTargetInterface & target, double yaw0, double bullet_speed);
  // std::pair<Trajectory, int> get_trajectory_with_rank(Target & target, double yaw0, double bullet_speed);
  
  //Plan plan(const Eigen::VectorXd& tracker_state, int tracked_armors_num, double bullet_speed, double send_time);
  //Plan plan(const Eigen::VectorXd& tracker_state, int tracked_armors_num, double bullet_speed, double send_time);
  Trajectory get_trajectory_from_state(const Eigen::VectorXd & target_state, int armor_num, double yaw0, double bullet_speed);
  Eigen::Matrix<double, 2, 1> aim_from_state(const Eigen::VectorXd & state_x, int armor_num, double bullet_speed);

  Eigen::Matrix<double, 2, 1> aim_from_state_2(const Eigen::VectorXd & state_x, int armor_num, double bullet_speed);

  std::vector<Eigen::Vector4d> compute_armor_xyza(const Eigen::VectorXd & state_x, int armor_num) const;
  Eigen::VectorXd predict_state(const Eigen::VectorXd & current_state, double dt) const;

  ruckig::Ruckig<2> ruckig_yaw_pitch_ {DT};
  // ruckig::Trackig<2> trackig_yaw_pitch_ {0.001};
  ruckig::InputParameter<2> ruckig_input_;
  ruckig::Trajectory<2> ruckig_trajectory_;
  // ruckig::OutputParameter<2> ruckig_output_;
  // std::pair<bool,int> ruckig_search(Trajectory & traj, int power);
  int shoot_offset_ = 2;//* 以轨迹的步长进行偏移
  double time_ruckig;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__PLANNER_HPP