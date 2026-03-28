#include "publish2nav.hpp"

#include <Eigen/Dense>
#include <chrono>
#include <memory>
#include <thread>

#include "tools/logger.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace io
{

Publish2Nav::Publish2Nav() : Node("auto_aim_target_pos_publisher")
{
  publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("auto_aim_target_pos", 10);

  RCLCPP_INFO(this->get_logger(), "auto_aim_target_pos_publisher node initialized.");
}

Publish2Nav::~Publish2Nav()
{
  RCLCPP_INFO(this->get_logger(), "auto_aim_target_pos_publisher node shutting down.");
}

void Publish2Nav::send_data(const Eigen::Vector3d & target_pos)
{
  // 创建消息
  auto message = std::make_shared<geometry_msgs::msg::PoseStamped>();

  // 将 Eigen::Vector3d 数据转换为字符串并存储在消息中
  message->pose.position.x = target_pos[0];
  message->pose.position.y = target_pos[1];
  message->pose.position.z = target_pos[2];

  // 发布消息
  publisher_->publish(*message);

  // RCLCPP_INFO(
  //   this->get_logger(), "auto_aim_target_pos_publisher node sent message: '%s'",
  //   message->data.c_str());
}

void Publish2Nav::start()
{
  RCLCPP_INFO(this->get_logger(), "auto_aim_target_pos_publisher node starting to spin...");
  rclcpp::spin(this->shared_from_this());
}

}  // namespace io
