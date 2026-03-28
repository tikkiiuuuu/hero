#ifndef IO__SUBSCRIBE2NAV_HPP
#define IO__SUBSCRIBE2NAV_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>

#include <vector>

#include "tools/thread_safe_queue.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32.hpp"
namespace io
{
class Subscribe2Nav : public rclcpp::Node
{
public:
  [[deprecated]] Subscribe2Nav();

  ~Subscribe2Nav();

  void start();
  //* 这里给拓展发送用
  tools::ThreadSafeRingBuffer<std::tuple<float,float,float>, 10> ros2_msg_;

private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_ul_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr angle_subscription_;
  void nav_ul_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void angle_callback(const std_msgs::msg::Float32::SharedPtr msg);
  float angle_;
  std::mutex mtx_;
  
};
}  // namespace io

#endif  // IO__SUBSCRIBE2NAV_HPP
