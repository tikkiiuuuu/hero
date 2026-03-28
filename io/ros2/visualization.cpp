#include "visualization.hpp"
#include "tools/logger.hpp"

namespace io
{

//! 只是ai写了代码，什么都没有实现

VisualizationPublisher::VisualizationPublisher() 
: Node("visualization_publisher"), marker_id_counter_(0)
{
    // 初始化发布器
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
    marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 10);
    point_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("target_point", 10);
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("gimbal_pose", 10);
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    robot_description_pub_ = this->create_publisher<std_msgs::msg::String>("robot_description", 10);

    // 初始化TF广播器
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);

    // 设置默认参数
    frame_id_ = "world";
    marker_lifetime_ = rclcpp::Duration::from_seconds(1.0);

    RCLCPP_INFO(this->get_logger(), "Visualization publisher initialized");
}

VisualizationPublisher::~VisualizationPublisher()
{
    RCLCPP_INFO(this->get_logger(), "Visualization publisher shutting down");
}

void VisualizationPublisher::publish_target_marker(const Eigen::Vector3d& position, const std::string& target_id)
{
    auto marker = create_marker("targets", marker_id_counter_++, visualization_msgs::msg::Marker::SPHERE);
    
    marker.pose.position.x = position[0];
    marker.pose.position.y = position[1];
    marker.pose.position.z = position[2];
    
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    
    marker.text = target_id;
    
    marker_pub_->publish(marker);
}

void VisualizationPublisher::publish_target_trajectory(const std::vector<Eigen::Vector3d>& trajectory)
{
    if (trajectory.size() < 2) return;

    auto marker = create_marker("trajectories", marker_id_counter_++, visualization_msgs::msg::Marker::LINE_STRIP);
    
    marker.scale.x = 0.02;  // 线宽
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.8;

    for (const auto& point : trajectory) {
        geometry_msgs::msg::Point p;
        p.x = point[0];
        p.y = point[1];
        p.z = point[2];
        marker.points.push_back(p);
    }

    marker_pub_->publish(marker);
}

void VisualizationPublisher::publish_target_velocity_arrow(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity)
{
    if (velocity.norm() < 1e-6) return;

    auto marker = create_marker("velocity_arrows", marker_id_counter_++, visualization_msgs::msg::Marker::ARROW);
    
    // 设置箭头起点
    marker.pose.position.x = position[0];
    marker.pose.position.y = position[1];
    marker.pose.position.z = position[2];
    
    // 设置箭头方向
    Eigen::Vector3d normalized_vel = velocity.normalized();
    Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(), normalized_vel);
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();
    
    // 设置箭头大小
    double scale = std::min(velocity.norm() * 0.1, 2.0);  // 限制最大长度
    marker.scale.x = scale;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 0.8;

    marker_pub_->publish(marker);
}

void VisualizationPublisher::publish_gimbal_pose(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation)
{
    // 发布云台姿态
    auto pose_msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
    pose_msg->header.stamp = this->now();
    pose_msg->header.frame_id = frame_id_;
    
    pose_msg->pose.position.x = position[0];
    pose_msg->pose.position.y = position[1];
    pose_msg->pose.position.z = position[2];
    
    pose_msg->pose.orientation.x = orientation.x();
    pose_msg->pose.orientation.y = orientation.y();
    pose_msg->pose.orientation.z = orientation.z();
    pose_msg->pose.orientation.w = orientation.w();
    
    pose_pub_->publish(*pose_msg);

    // 发布TF变换
    publish_tf_transform(frame_id_, "gimbal", position, orientation);
}

void VisualizationPublisher::publish_gimbal_fov(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation,
                                               double fov_angle, double range)
{
    auto marker = create_marker("fov", marker_id_counter_++, visualization_msgs::msg::Marker::TRIANGLE_LIST);
    
    marker.pose.position.x = position[0];
    marker.pose.position.y = position[1];
    marker.pose.position.z = position[2];
    
    marker.pose.orientation.x = orientation.x();
    marker.pose.orientation.y = orientation.y();
    marker.pose.orientation.z = orientation.z();
    marker.pose.orientation.w = orientation.w();
    
    // 创建FOV三角形
    double half_angle = fov_angle * M_PI / 180.0 / 2.0;
    double tan_half_angle = std::tan(half_angle);
    
    // 创建4个三角形组成锥形
    std::vector<Eigen::Vector3d> corners = {
        {0, 0, 0},  // 顶点
        {range, -range * tan_half_angle, -range * tan_half_angle},
        {range, range * tan_half_angle, -range * tan_half_angle},
        {range, range * tan_half_angle, range * tan_half_angle},
        {range, -range * tan_half_angle, range * tan_half_angle}
    };
    
    // 添加三角形
    std::vector<std::vector<int>> triangles = {
        {0, 1, 2}, {0, 2, 3}, {0, 3, 4}, {0, 4, 1}
    };
    
    for (const auto& tri : triangles) {
        for (int idx : tri) {
            geometry_msgs::msg::Point p;
            p.x = corners[idx][0];
            p.y = corners[idx][1];
            p.z = corners[idx][2];
            marker.points.push_back(p);
        }
    }
    
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.3;

    marker_pub_->publish(marker);
}

void VisualizationPublisher::publish_robot_state(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation,
                                                double yaw, double pitch)
{
    // 发布机器人基座
    publish_tf_transform(frame_id_, "base_link", position, orientation);
    
    // 发布云台关节状态
    auto joint_state = std::make_shared<sensor_msgs::msg::JointState>();
    joint_state->header.stamp = this->now();
    joint_state->header.frame_id = frame_id_;
    
    joint_state->name = {"gimbal_yaw_joint", "gimbal_pitch_joint"};
    joint_state->position = {yaw, pitch};
    joint_state->velocity = {0.0, 0.0};
    joint_state->effort = {0.0, 0.0};
    
    joint_state_pub_->publish(*joint_state);
}

void VisualizationPublisher::publish_robot_urdf(const std::string& urdf_content)
{
    auto msg = std::make_shared<std_msgs::msg::String>();
    msg->data = urdf_content;
    robot_description_pub_->publish(*msg);
}

void VisualizationPublisher::publish_joint_states(const std::vector<std::string>& joint_names,
                                                 const std::vector<double>& joint_positions)
{
    if (joint_names.size() != joint_positions.size()) return;

    auto joint_state = std::make_shared<sensor_msgs::msg::JointState>();
    joint_state->header.stamp = this->now();
    joint_state->header.frame_id = frame_id_;
    
    joint_state->name = joint_names;
    joint_state->position = joint_positions;
    joint_state->velocity.resize(joint_names.size(), 0.0);
    joint_state->effort.resize(joint_names.size(), 0.0);
    
    joint_state_pub_->publish(*joint_state);
}

void VisualizationPublisher::publish_ground_plane(double size)
{
    auto marker = create_marker("environment", marker_id_counter_++, visualization_msgs::msg::Marker::CUBE);
    
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = -0.01;  // 稍微低于地面
    
    marker.scale.x = size;
    marker.scale.y = size;
    marker.scale.z = 0.02;
    
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.color.a = 0.3;

    marker_pub_->publish(marker);
}

void VisualizationPublisher::publish_coordinate_frames()
{
    // 发布世界坐标系
    publish_tf_transform("world", "base_link", Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity());
    
    // 发布坐标轴标记
    std::vector<Eigen::Vector3d> axes = {
        {1, 0, 0},  // X轴 - 红色
        {0, 1, 0},  // Y轴 - 绿色  
        {0, 0, 1}   // Z轴 - 蓝色
    };
    
    std::vector<std::vector<double>> colors = {
        {1.0, 0.0, 0.0},  // 红色
        {0.0, 1.0, 0.0},  // 绿色
        {0.0, 0.0, 1.0}   // 蓝色
    };
    
    for (size_t i = 0; i < axes.size(); ++i) {
        auto marker = create_marker("coordinate_axes", marker_id_counter_++, visualization_msgs::msg::Marker::ARROW);
        
        marker.scale.x = 1.0;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;
        
        marker.color.r = colors[i][0];
        marker.color.g = colors[i][1];
        marker.color.b = colors[i][2];
        marker.color.a = 1.0;

        marker_pub_->publish(marker);
    }
}

void VisualizationPublisher::clear_all_markers()
{
    auto marker = create_marker("", 0, visualization_msgs::msg::Marker::DELETEALL);
    marker_pub_->publish(marker);
}

void VisualizationPublisher::clear_marker_by_id(int marker_id)
{
    auto marker = create_marker("", marker_id, visualization_msgs::msg::Marker::DELETE);
    marker_pub_->publish(marker);
}

// 辅助方法实现
visualization_msgs::msg::Marker VisualizationPublisher::create_marker(const std::string& ns, int id, int type)
{
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = this->now();
    marker.header.frame_id = frame_id_;
    marker.ns = ns;
    marker.id = id;
    marker.type = type;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.lifetime = marker_lifetime_;
    return marker;
}

void VisualizationPublisher::publish_tf_transform(const std::string& parent_frame, const std::string& child_frame,
                                                 const Eigen::Vector3d& translation, const Eigen::Quaterniond& rotation)
{
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->now();
    transform.header.frame_id = parent_frame;
    transform.child_frame_id = child_frame;
    
    transform.transform.translation.x = translation[0];
    transform.transform.translation.y = translation[1];
    transform.transform.translation.z = translation[2];
    
    transform.transform.rotation.x = rotation.x();
    transform.transform.rotation.y = rotation.y();
    transform.transform.rotation.z = rotation.z();
    transform.transform.rotation.w = rotation.w();
    
    tf_broadcaster_->sendTransform(transform);
}

} // namespace io