#include <fmt/core.h>

#include <chrono>
#include <fstream>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/rw_tracker.hpp"
// #include "tasks/auto_aim/yolo.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

const std::string keys =
  "{help h usage ? |                   | 输出命令行参数说明 }"
  "{config-path c  | configs/demo_rw.yaml | yaml配置文件的路径}"
  "{start-index s  | 2                 | 视频起始帧下标    }"
  "{end-index e    | 0                 | 视频结束帧下标    }"
  "{@input-path    | assets/demo/demo  | avi和txt文件的路径}";

int main(int argc, char * argv[])
{
  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto input_path = cli.get<std::string>(0);
  auto config_path = cli.get<std::string>("config-path");
  auto start_index = cli.get<int>("start-index");
  auto end_index = cli.get<int>("end-index");

  tools::Plotter plotter;
  tools::Exiter exiter;

  auto video_path = fmt::format("{}.avi", input_path);
  auto text_path = fmt::format("{}.txt", input_path);
  cv::VideoCapture video(video_path);
  std::ifstream text(text_path);

  // auto_aim::YOLO yolo(config_path);
  auto_aim::Detector detector(config_path);
  auto_aim::Solver solver(config_path);
  auto_aim::Planner planner(config_path);
  auto_aim::RWTracker rw_tracker(config_path, solver);
//   auto_aim::Aimer aimer(config_path);

  cv::Mat img, drawing;
  auto t0 = std::chrono::steady_clock::now();

//   io::Command last_command;
  double last_t = -1;

  std::chrono::steady_clock::time_point last_timestamp;

  video.set(cv::CAP_PROP_POS_FRAMES, start_index);
  for (int i = 0; i < start_index; i++) {
    double t, w, x, y, z;
    text >> t >> w >> x >> y >> z;
    last_timestamp = t0 + std::chrono::microseconds(int(t * 1e6));
  }

  for (int frame_count = start_index; !exiter.exit(); frame_count++) {
    if (end_index > 0 && frame_count > end_index) break;

    video.read(img);
    if (img.empty()) break;

    double t, w, x, y, z;
    text >> t >> w >> x >> y >> z;
    auto timestamp = t0 + std::chrono::microseconds(int(t * 1e6));

    /// 自瞄核心逻辑

    solver.set_R_gimbal2world({w, x, y, z});

    auto yolo_start = std::chrono::steady_clock::now();
    auto [armors, lightbars] = detector.detect_light(img, frame_count);
    
    // 对每个装甲板调用 solver.solve() 来初始化 xyz_in_world 和 ypr_in_world
    for (auto& armor : armors) {
      solver.solve(armor);
    }
    if(rw_tracker.tracker_state == auto_aim::RWTracker::TrackState::LOST) {
      rw_tracker.init(armors);
    }
    auto tracker_start = std::chrono::steady_clock::now();
    // auto targets = rw_tracker.track(armors, timestamp);
    rw_tracker.dt_ = tools::delta_time(timestamp, last_timestamp);
    last_timestamp = timestamp;
    rw_tracker.update(armors, lightbars);
    std::cout<<"debug: ekf_prediction: "<<rw_tracker.ekf_prediction.transpose()<<std::endl;
    auto aimer_start = std::chrono::steady_clock::now();
    auto plan = planner.plan(rw_tracker.get_interface(), 22.0);
    auto finish = std::chrono::steady_clock::now();
    tools::logger()->info(
      "[{}] yolo: {:.1f}ms, tracker: {:.1f}ms", frame_count,
      tools::delta_time(tracker_start, yolo_start) * 1e3,
      tools::delta_time(aimer_start, tracker_start) * 1e3);
      // tools::delta_time(finish, aimer_start) * 1e3);


    Eigen::Quaternion gimbal_q = {w, x, y, z};
    tools::draw_text(
      img,
      fmt::format(
        "gimbal yaw{:.2f}", (tools::eulers(gimbal_q.toRotationMatrix(), 2, 1, 0) * 57.3)[0]),
      {10, 90}, {255, 255, 255});

    nlohmann::json data;
    data["plan_yaw"] = plan.yaw;
    data["plan_yaw_vel"] = plan.yaw_vel;
    data["plan_yaw_acc"] = plan.yaw_acc;
    data["plan_pitch"] = plan.pitch;
    data["plan_pitch_vel"] = plan.pitch_vel;
    data["plan_pitch_acc"] = plan.pitch_acc;
    plotter.plot(data);

    // // 装甲板原始观测数据
    // data["armor_num"] = armors.size();
    // if (!armors.empty()) {
    //   const auto & armor = armors.front();
    //   data["armor_x"] = armor.xyz_in_world[0];
    //   data["armor_y"] = armor.xyz_in_world[1];
    //   data["armor_yaw"] = armor.ypr_in_world[0] * 57.3;
    //   data["armor_yaw_raw"] = armor.yaw_raw * 57.3;
    //   data["armor_center_x"] = armor.center_norm.x;
    //   data["armor_center_y"] = armor.center_norm.y;
    // }

    // Eigen::Quaternion q{w, x, y, z};
    // auto yaw = tools::eulers(q, 2, 1, 0)[0];
    // data["gimbal_yaw"] = yaw * 57.3;

      // std::vector<Eigen::Vector4d> armor_xyza_list;

      // // 当前帧target更新后
      // armor_xyza_list = rw_tracker.get_interface().armor_xyza_list();
      // for (const Eigen::Vector4d & xyza : armor_xyza_list) {
      //   auto image_points =
      //     solver.reproject_armor(xyza.head(3), xyza[3], rw_tracker.tracked_armor->type, rw_tracker.tracked_armor->name);
      //   tools::draw_points(img, image_points, {0, 255, 0});
      // }
      // std::cout<<"debug: draw_ekf_prediction: "<<rw_tracker.ekf_prediction.transpose()<<std::endl;
      rw_tracker.drawResults(img);

      // aimer瞄准位置
    //   auto aim_point = aimer.debug_aim_point;
    //   Eigen::Vector4d aim_xyza = aim_point.xyza;
    //   auto image_points =
    //     solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
    //   if (aim_point.valid) tools::draw_points(img, image_points, {0, 0, 255});

      // // 观测器内部数据
      // Eigen::VectorXd x = target->get_state();
      // data["x"] = x[0];
      // data["vx"] = x[1];
      // data["y"] = x[2];
      // data["vy"] = x[3];
      // data["z"] = x[4];
      // data["vz"] = x[5];
      // data["a"] = x[6] * 57.3;
      // data["w"] = x[7];
      // data["r"] = x[8];
      // data["l"] = x[9];
      // data["h"] = x[10];
    //   data["last_id"] = target.last_id;

    //   // 卡方检验数据
    //   data["residual_yaw"] = target.ekf().data.at("residual_yaw");
    //   data["residual_pitch"] = target.ekf().data.at("residual_pitch");
    //   data["residual_distance"] = target.ekf().data.at("residual_distance");
    //   data["residual_angle"] = target.ekf().data.at("residual_angle");
    //   data["nis"] = target.ekf().data.at("nis");
    //   data["nees"] = target.ekf().data.at("nees");
    //   data["nis_fail"] = target.ekf().data.at("nis_fail");
    //   data["nees_fail"] = target.ekf().data.at("nees_fail");
    //   data["recent_nis_failures"] = target.ekf().data.at("recent_nis_failures");
    

    // plotter.plot(data);

    // // === 添加弹道绘制调用 ===
    // if (!targets.empty() && aimer.debug_aim_point.valid) {
    //   auto aim_xyza = aimer.debug_aim_point.xyza;
    //   double bullet_speed = 27.0; // 与 aimer.aim() 调用中使用的弹速保持一致
    //   // 使用 aimer 计算出的最终瞄准点来创建弹道
    //   tools::Trajectory final_traj(
    //     bullet_speed, std::hypot(aim_xyza[0], aim_xyza[1]), aim_xyza[2]);
    //   // 使用 aimer 计算出的最终 yaw 角来绘制
    //   solver.draw_trajectory(img, final_traj, command.yaw, bullet_speed);
    // }
    // // =======================
    
    cv::resize(img, img, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
    cv::imshow("reprojection", img);
    auto key = cv::waitKey(30);
    if (key == 'q') break;
  }

  return 0;
}