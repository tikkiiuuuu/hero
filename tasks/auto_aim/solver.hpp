#ifndef AUTO_AIM__SOLVER_HPP
#define AUTO_AIM__SOLVER_HPP

#include <Eigen/Dense>  // 必须在opencv2/core/eigen.hpp上面
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>

#include "armor.hpp"
#include "tools/trajectory.hpp"

namespace auto_aim
{
class Solver
{
public:
  explicit Solver(const std::string & config_path);

  Eigen::Matrix3d R_gimbal2world() const;

  void set_R_gimbal2world(const Eigen::Quaterniond & q);

  void solve(Armor & armor) const;

  std::vector<cv::Point2f> reproject_armor(
    const Eigen::Vector3d & xyz_in_world, double yaw, ArmorType type, ArmorName name) const;

  double outpost_reprojection_error(Armor armor, const double & picth);

  std::vector<cv::Point2f> world2pixel(const std::vector<cv::Point3f> & worldPoints) const;

  void draw_trajectory(
    cv::Mat & img, const tools::Trajectory & traj, double aim_yaw, double v0) const;

  float calculateDistanceToCenter(const cv::Point2f & image_point) const;

  Eigen::Vector3d odom2camera_tvec() const; 
  Eigen::Matrix4d odom_camera_matrix() const;

private:
  cv::Mat camera_matrix_;
  cv::Mat distort_coeffs_;
  Eigen::Matrix3d R_gimbal2imubody_;
  Eigen::Matrix3d R_camera2gimbal_;
  Eigen::Vector3d t_camera2gimbal_;
  Eigen::Matrix3d R_gimbal2world_;
  Eigen::Vector3d t_odom2camera_;
  Eigen::Matrix3d R_odom2camera_;

  void optimize_yaw(Armor & armor) const;
  void optimize_yaw_mylights(Armor & armor);

  double armor_reprojection_error(const Armor & armor, double yaw, const double & inclined) const;
  double SJTU_cost(
    const std::vector<cv::Point2f> & cv_refs, const std::vector<cv::Point2f> & cv_pts,
    const double & inclined) const;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__SOLVER_HPP