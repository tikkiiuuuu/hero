#include "ros2.hpp"
#include "geometry_msgs/msg/twist.hpp"
namespace io
{
ROS2::ROS2()
{
  // rclcpp::init(0, nullptr);
  rclcpp::init(0, nullptr, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);

  publish2nav_ = std::make_shared<Publish2Nav>();

  subscribe2nav_ = std::make_shared<Subscribe2Nav>();

  publish_spin_thread_ = std::make_unique<std::thread>([this]() { publish2nav_->start(); });

  subscribe_spin_thread_ = std::make_unique<std::thread>([this]() { subscribe2nav_->start(); });
}

ROS2::~ROS2()
{
  rclcpp::shutdown();
  publish_spin_thread_->join();
  subscribe_spin_thread_->join();
}

void ROS2::publish(const Eigen::Vector3d & target_pos) { publish2nav_->send_data(target_pos); }

}  // namespace io
