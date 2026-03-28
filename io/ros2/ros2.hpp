#ifndef IO__ROS2_HPP
#define IO__ROS2_HPP

#include "publish2nav.hpp"
#include "subscribe2nav.hpp"
// #include "geometry_msgs/msg/twist.hpp"

namespace io
{
class ROS2
{
public:
  ROS2();

  ~ROS2();

  void publish(const Eigen::Vector3d & target_pos);

  std::tuple<float,float,float> get_nav_ul_subscribe(){
    while(subscribe2nav_->ros2_msg_.empty()){
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return subscribe2nav_->ros2_msg_.back();
  }

  template <typename T>
  std::shared_ptr<rclcpp::Publisher<T>> create_publisher(
    const std::string & node_name, const std::string & topic_name, size_t queue_size)
  {
    auto node = std::make_shared<rclcpp::Node>(node_name);

    auto publisher = node->create_publisher<T>(topic_name, queue_size);

    // 运行一个单独的线程来 spin 这个节点，确保消息可以被正确发布
    std::thread([node]() { rclcpp::spin(node); }).detach();

    return publisher;
  }

  std::shared_ptr<Subscribe2Nav> subscribe2nav_;

private:
  std::shared_ptr<Publish2Nav> publish2nav_;
  

  std::unique_ptr<std::thread> publish_spin_thread_;
  std::unique_ptr<std::thread> subscribe_spin_thread_;
};

}  // namespace io
#endif