#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "tasks/auto_aim/planner/planner.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

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

  auto_aim::Planner planner(config_path,1);
  auto_aim::Target target(d,dir,v, w, 0.2, 0.1);

  auto t0 = std::chrono::steady_clock::now();
  nlohmann::json data;

  while (!exiter.exit()) {
    // target.update_xyz(1,0,M_PI/4);
    target.predict(0.01);

    planner.plan_double(target, 22);

    nlohmann::json data;
    data["t"] = tools::delta_time(std::chrono::steady_clock::now(), t0);
    t0 = std::chrono::steady_clock::now();

    data["target_yaw"] = planner.plan_double_.target_yaw;
    data["target_pitch"] = planner.plan_double_.target_pitch;
    data["target_yaw_vel"] = planner.plan_double_.target_yaw_vel;
    
    data["plan_yaw"] = planner.plan_double_.yaw;
    data["plan_yaw_vel"] = planner.plan_double_.yaw_vel;
    data["plan_yaw_acc"] = planner.plan_double_.yaw_acc;

    data["plan_yaw_big"] = planner.plan_double_.yaw_big;
    data["plan_yaw_vel_big"] = planner.plan_double_.yaw_vel_big;
    data["plan_yaw_acc_big"] = planner.plan_double_.yaw_acc_big;
    
    data["plan_yaw_small"] = planner.plan_double_.yaw_small;
    data["plan_yaw_vel_small"] = planner.plan_double_.yaw_vel_small;
    data["plan_yaw_acc_small"] = planner.plan_double_.yaw_acc_small;

    data["plan_pitch"] = planner.plan_double_.pitch;
    data["plan_pitch_vel"] = planner.plan_double_.pitch_vel;
    data["plan_pitch_acc"] = planner.plan_double_.pitch_acc;

    data["plan_yaw_sum"] = planner.plan_double_.yaw_big+planner.plan_double_.yaw_small;
    data["plan_yaw_vel_sum"] = planner.plan_double_.yaw_vel_big+planner.plan_double_.yaw_vel_small;

    plotter.plot(data);

    std::this_thread::sleep_for(10ms);
  }

  return 0;
}