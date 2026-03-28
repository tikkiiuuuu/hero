#ifndef IO__PBLISH2NAV_HPP
#define IO__PBLISH2NAV_HPP

#include <Eigen/Dense>  // For Eigen::Vector3d

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace io
{
class Publish2Nav : public rclcpp::Node
{
public:
  Publish2Nav();

  ~Publish2Nav();

  void start();

  void send_data(const Eigen::Vector3d & target_pos);

private:
  // ROS2 发布者
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
};

}  // namespace io

#endif  // Publish2Nav_HPP_
