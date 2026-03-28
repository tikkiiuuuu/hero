#include <fmt/core.h>

#include <atomic>
#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"
#include "tools/thread_safe_queue.hpp"

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{@config-path   | configs/sentry.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  tools::Exiter exiter;
  tools::Plotter plotter;
  tools::Recorder recorder(60);

  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  io::Gimbal gimbal(config_path);
  io::Camera camera(config_path);

  auto_aim::YOLO yolo(config_path, true);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Planner planner(config_path);

  tools::ThreadSafeQueue<std::optional<auto_aim::Target>, true> target_queue(1);
  target_queue.push(std::nullopt);

  std::atomic<bool> quit = false;
  auto plan_thread = std::thread([&]() {
    auto t0 = std::chrono::steady_clock::now();
    uint16_t last_bullet_count = 0;

    while (!quit) {
      auto target = target_queue.front();
      auto gs = gimbal.state();
      // auto plan = planner.plan(target, gs.bullet_speed);
      auto plan = planner.plan(target, 10e9);
    auto current_time = std::chrono::steady_clock::now();
    auto q = gimbal.q(current_time);
      Eigen::Vector3d euler_angles = tools::eulers(q, 2, 1, 0, false);
      // Eigen::Vector3d euler_angles = tools::eulers(q, 2, 1, 0, false);
      double yaw_from_quaternion = euler_angles[0];    // 弧度
      double pitch_from_quaternion = euler_angles[1];   // 弧度
      double roll_from_quaternion = euler_angles[2];    // 弧度

      if (target.has_value())
  {    // 获取所有装甲板的世界坐标
      std::vector<Eigen::Vector4d> armor_xyza_list = target->armor_xyza_list();
      
      // 选择最近的装甲板（或任意一个）
      Eigen::Vector3d xyz = armor_xyza_list[0].head(3);  // 取第一个装甲板的xyz
      
      // 转换为球坐标 (yaw, pitch, distance)
      Eigen::Vector3d ypd = tools::xyz2ypd(xyz);
      
      double target_yaw = ypd[0];    // 目标yaw角度
      double target_pitch = ypd[1];  // 目标pitch角度
      double distance = ypd[2];
      gimbal.send(
        plan.control, plan.fire, 
        // -gs.yaw+plan.yaw,
        plan.yaw, 
        plan.yaw_vel, plan.yaw_acc, 
        // -plan.pitch-gs.pitch, 
        // -gs.pitch+plan.pitch,
        -plan.pitch,
        plan.pitch_vel,plan.pitch_acc);}
      // else{
      //   gimbal.send(true, false, gs.yaw, 0, 0, gs.pitch, 0, 0);
      //   // gimbal.send(true, false, -yaw_from_quaternion, 0, 0,  -pitch_from_quaternion, 0, 0);
      // }

      auto fired = gs.bullet_count > last_bullet_count;
      last_bullet_count = gs.bullet_count;

      nlohmann::json data;  
      data["t"] = tools::delta_time(std::chrono::steady_clock::now(), t0);

      data["gimbal_yaw"] = gs.yaw;
      data["gimbal_yaw_vel"] = gs.yaw_vel;
      data["gimbal_pitch"] = gs.pitch;
      data["gimbal_pitch_vel"] = gs.pitch_vel;

      data["target_yaw"] = plan.target_yaw;
      data["target_pitch"] = plan.target_pitch;

      data["plan_yaw"] = plan.yaw;
      data["plan_yaw_vel"] = plan.yaw_vel;
      data["plan_yaw_acc"] = plan.yaw_acc;

      data["plan_pitch"] = -plan.pitch;
      data["plan_pitch_vel"] = plan.pitch_vel;
      data["plan_pitch_acc"] = plan.pitch_acc;

      data["fire"] = plan.fire ? 1 : 0;
      data["fired"] = fired ? 1 : 0;

      if (target.has_value()) {
        data["target_z"] = target->get_state()[4];   //z
        data["target_vz"] = target->get_state()[5];  //vz
        data["target_vx"] = target->get_state()[1];
        data["target_vx"] = target->get_state()[3];
      }

      if (target.has_value()) {
        data["w"] = target->get_state()[7];
      } else {
        data["w"] = 0.0;
      }

      plotter.plot(data);

      std::this_thread::sleep_for(10ms);
    }
  });

  cv::Mat img;
  std::chrono::steady_clock::time_point t;
  auto time_offset_us = std::chrono::microseconds(100);


  while (!exiter.exit()) {
    camera.read(img, t);
    // auto q = gimbal.q(t+100us);
    auto q = gimbal.q(t+time_offset_us);

    solver.set_R_gimbal2world(q);
    auto armors = yolo.detect(img);
    auto targets = tracker.track(armors, t+time_offset_us);
    if (!targets.empty())
      target_queue.push(*targets.front());
    else
      target_queue.push(std::nullopt);

    if (!targets.empty()) {
      auto target = targets.front();

      // 当前帧target更新后
      std::vector<Eigen::Vector4d> armor_xyza_list = target->armor_xyza_list();
      for (const Eigen::Vector4d & xyza : armor_xyza_list) {
        auto image_points =
          solver.reproject_armor(xyza.head(3), xyza[3], target->armor_type, target->name);
        tools::draw_points(img, image_points, {0, 255, 0});
      }

      Eigen::Vector4d aim_xyza = planner.debug_xyza;
      auto image_points =
        solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target->armor_type, target->name);
      tools::draw_points(img, image_points, {0, 0, 255});
      recorder.record(img, q, t);
      // tools::Trajectory final_traj(
      //   gs.bullet_speed, std::hypot(aim_xyza[0], aim_xyza[1]), aim_xyza[2]);
      // // 使用 aimer 计算出的最终 yaw 角来绘制
      // solver.draw_trajectory(img, final_traj, plan.yaw, gs.bullet_speed);
    }

    // 在图像上显示当前的时间偏移量
    auto offset_text = fmt::format("Time Offset: {} us", time_offset_us.count());
    cv::putText(img, offset_text, {10, 60}, cv::FONT_HERSHEY_SIMPLEX, 1.0, {0, 255, 0}, 2);

    cv::resize(img, img, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
    cv::imshow("reprojection", img);
    auto key = cv::waitKey(1);
    if (key == 'q') break;
    if (key == '=' || key == '+') {
      time_offset_us += std::chrono::microseconds(100);
    }
    if (key == '-') {
      time_offset_us -= std::chrono::microseconds(100);
    }

    nlohmann::json data;
    
    data["q_w"] = q.w();
    data["q_x"] = q.x();
    data["q_y"] = q.y();
    data["q_z"] = q.z();
    plotter.plot(data);
  }

  quit = true;
  if (plan_thread.joinable()) plan_thread.join();
  gimbal.send(false, false, 0, 0, 0, 0, 0, 0);

  return 0;
}