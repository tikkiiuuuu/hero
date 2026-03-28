#include "subscribe2nav.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32.hpp"

#include <sstream>
#include <vector>

namespace io
{

Subscribe2Nav::Subscribe2Nav()
: Node("nav_subscriber")
{
  nav_ul_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10,
    std::bind(&Subscribe2Nav::nav_ul_callback, this, std::placeholders::_1));
  angle_subscription_ = this->create_subscription<std_msgs::msg::Float32>(
    "/cmd_angle", 10,
    std::bind(&Subscribe2Nav::angle_callback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "nav_subscriber node initialized.");
}

Subscribe2Nav::~Subscribe2Nav()
{
  RCLCPP_INFO(this->get_logger(), "nav_subscriber node shutting down.");
}

void Subscribe2Nav::angle_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(mtx_);
  angle_ = msg->data;
}
void Subscribe2Nav::nav_ul_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  float angle;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    angle = angle_;
  }
  ros2_msg_.push(std::make_tuple(msg->linear.x, msg->linear.y, angle));
  RCLCPP_INFO(this->get_logger(), "nav_subscriber node received message: '%f', '%f', '%f'", msg->linear.x, msg->linear.y, msg->linear.z);
}

void Subscribe2Nav::start()
{
  RCLCPP_INFO(this->get_logger(), "nav_subscriber node Starting to spin...");
  rclcpp::spin(this->shared_from_this());
}

}  // namespace io