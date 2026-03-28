#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "tasks/auto_aim/planner/planner.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/rw_tracker.hpp"
#include "tools/timer.hpp"
#include "tools/extended_kalman_filter.hpp"
#include "io/ros2/ros2.hpp"
using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |        | 输出命令行参数说明    }"
  "{d              | 3.0    | Target距离(m)       }"
  "{w              | 5.0    | Target角速度(rad/s) }"
  "{v              | 1.0    | 速度                }"
  "{dir            | M_PI/4 | 方向                }"
  "{@config-path   |        | yaml配置文件路径     }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>("@config-path");
  auto d = cli.get<double>("d");
  auto w = cli.get<double>("w");
  auto v = cli.get<double>("v");
  auto dir = cli.get<double>("dir");
  if (cli.has("help") || !cli.has("@config-path")) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;
  tools::Plotter plotter;

  io::Gimbal gimbal(config_path);
  io::ROS2 ros2;
  gimbal.set_interface(ros2.subscribe2nav_->ros2_msg_);
  auto_aim::Planner planner(config_path);
  auto_aim::Target target(d,dir,v, w, 0.25, 0.1);
  // auto_aim::Solver solver(config_path);
  // auto_aim::RWTracker rw_tracker(config_path, solver);

  auto t0 = std::chrono::steady_clock::now();
  double delta_time = 0.0;
  nlohmann::json data;
  tools::KalmanFilter kalman_filter(10,25,1,0.0625,1,0.0625);

  int count = 0;

  while (!exiter.exit()) {
    {
      tools::ScopedTimer timer("planner", delta_time);
      // target.update_xyz(1,0,M_PI/4);
      target.predict(0.01);
      // auto plan = planner.plan(target, 22);
      const double send_time = 0.005;

      // auto plan1 = planner.plan_with_ruckig(target, 22, send_time);
      auto plan = planner.plan(target, 22);

      
      data["t"] = tools::delta_time(std::chrono::steady_clock::now(), t0);

      data["target_yaw"] = plan.target_yaw;
      data["target_pitch"] = plan.target_pitch;
      data["plan_yaw"] = plan.yaw;
      data["plan_yaw_vel"] = plan.yaw_vel;
      data["plan_yaw_acc"] = plan.yaw_acc;
      data["plan_pitch"] = plan.pitch;
      data["plan_pitch_vel"] = plan.pitch_vel;
      data["plan_pitch_acc"] = plan.pitch_acc;

      // data["target_yaw1"] = plan1.target_yaw;
      // data["target_pitch1"] = plan1.target_pitch;
      // data["plan_yaw1"] = plan1.yaw;
      // data["plan_yaw_vel1"] = plan1.yaw_vel;
      // data["plan_yaw_acc1"] = plan1.yaw_acc;
      // data["plan_pitch1"] = plan1.pitch;
      // data["plan_pitch_vel1"] = plan1.pitch_vel;
      // data["plan_pitch_acc1"] = plan1.pitch_acc;

      // data["fire"] = plan.fire ? 1 : 0;

      data["gimbal_yaw"] = gimbal.state().yaw;
      data["gimbal_yaw_vel"] = gimbal.state().yaw_vel;
      data["gimbal_pitch"] = gimbal.state().pitch;
      data["gimbal_pitch_vel"] = gimbal.state().pitch_vel;
      gimbal.send_sentry(
          plan.control,
          plan.fire,
          plan.yaw,
          plan.yaw_vel,
          plan.yaw_acc,
          plan.pitch,
          plan.pitch_vel,
          plan.pitch_acc,
          plan.yaw*1.1,
          target.get_state()[0],
          target.get_state()[2],
          target.get_state()[4],
          0,
          0
      );
      // if(count <50) {
      //   gimbal.send_sentry(
      //     plan.control,
      //     plan.fire,
      //     0.0,
      //     plan.yaw_vel,
      //     plan.yaw_acc,
      //     plan.pitch,
      //     plan.pitch_vel,
      //     plan.pitch_acc,
      //     plan.yaw*0.1,
      //     target.get_state()[0],
      //     target.get_state()[2],
      //     target.get_state()[4]
      // );
      // data["count"] = 0;
      // }else{
      //   gimbal.send_sentry(
      //     plan.control,
      //     plan.fire,
      //     0.5,
      //     plan.yaw_vel,
      //     plan.yaw_acc,
      //     plan.pitch,
      //     plan.pitch_vel,
      //     plan.pitch_acc,
      //     plan.yaw*0.1,
      //     target.get_state()[0],
      //     target.get_state()[2],
      //     target.get_state()[4]
      // );
      // data["count"] = 1;
      // }


      kalman_filter.predict();
      kalman_filter.update(Eigen::VectorXd::Constant(1, delta_time));
      data["kalman_filter_x"] = kalman_filter.getx();
      std::this_thread::sleep_for(10ms);
    }
    data["delta_time"] = delta_time;
    plotter.plot(data);
    count = (count+1)%100;
  }

  return 0;
}