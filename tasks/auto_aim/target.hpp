#ifndef AUTO_AIM__TARGET_HPP
#define AUTO_AIM__TARGET_HPP

#include <Eigen/Dense>
#include <chrono>
#include <optional>
#include <queue>
#include <string>
#include <vector>

#include "armor.hpp"
#include "tools/extended_kalman_filter.hpp"

namespace auto_aim
{

class FilterStrategy;
class EKFStrategy;

class Target
{
public:
  enum state{
    x,vx,y,vy,z,vz,a,w,r,l,h
  };
  // a: angle
  // w: angular velocity
  // r: r1
  // l: r2 - r1
  // h: z2 - z1
  ArmorName name;
  ArmorType armor_type;
  ArmorPriority priority;
  bool jumped;
  int last_id;  // debug only

  Target() = default;
  Target(const Target& other);
  Target(
    const Armor & armor, 
    int armor_num,
    std::unique_ptr<FilterStrategy> filter);

  Target(double x, double vyaw, double radius, double h);//* 只是为了创建虚拟对象，还用原来的
  Target(double x,double direction, double v, double vyaw, double radius, double h);

  void update(const Armor & armor);
  void predict(const double dt);
  void predict(std::chrono::steady_clock::time_point t);
  const Eigen::VectorXd& get_state() const;
  std::vector<Eigen::Vector4d> armor_xyza_list() const;
  bool diverged() const;
  bool convergened();
  bool has_bad_convergence() const;
  bool isinit = false;
  bool checkinit();
  Eigen::Vector3d h_armor_xyz( int id) const;

  // friend class FilterStrategy;
  friend class EKFStrategy;

  Target& operator=(const Target& other);

  Eigen::Vector4d R_dig;

private:
  int armor_num_;
  int switch_count_;
  int update_count_;

  bool is_switch_, is_converged_;

  std::unique_ptr<FilterStrategy> filter_;
};

class FilterStrategy
{
public:
  virtual ~FilterStrategy() = default;
  virtual void update_ypda(const Armor & armor, int id, Target& target) = 0;
  virtual void predict(const double dt, Target& target) = 0;
  virtual void predict(std::chrono::steady_clock::time_point t, Target& target) = 0;
  virtual const Eigen::VectorXd& get_state() const = 0;
  virtual bool has_bad_convergence() const = 0;
  virtual std::unique_ptr<FilterStrategy> clone() const = 0;
  // virtual Eigen::Vector3d h_armor_xyz(const Eigen::VectorXd & x, int id) const = 0;
};

class EKFStrategy : public FilterStrategy
{
public:

  EKFStrategy(
    std::chrono::steady_clock::time_point t,
    const Eigen::VectorXd & x0, 
    const Eigen::MatrixXd & P0,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add);

  EKFStrategy(
    const Eigen::VectorXd & x0, 
    const Eigen::MatrixXd & P0,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add);

  void update_ypda(const Armor & armor, int id, Target& target) override;  // yaw pitch distance angle
  void predict(double dt, Target& target) override;
  void predict(std::chrono::steady_clock::time_point t, Target& target) override;//* 自动计算和上次预测的时间差，对齐时间
  

  const Eigen::VectorXd& get_state() const override;//* 原项目中用来访问状态量和只读的整个对象的引用
  const tools::ExtendedKalmanFilter & ekf() const;

  bool has_bad_convergence() const override;

  std::unique_ptr<FilterStrategy> clone() const override;
  // Eigen::Vector3d h_armor_xyz(const Eigen::VectorXd & x, int id) const override;

private:
  
  tools::ExtendedKalmanFilter ekf_;
  std::chrono::steady_clock::time_point t_;
  
  Eigen::MatrixXd h_jacobian(Target&, int id) const;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TARGET_HPP