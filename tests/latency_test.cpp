#include "io/gimbal/gimbal.hpp"
#include "tools/exiter.hpp"
#include "tools/plotter.hpp"
#include "tools/yaml.hpp"
#include <opencv2/opencv.hpp>

const std::string keys =
  "{help h usage ? | | 输出命令行参数说明}"
  "{@config-path   | configs/standard3.yaml | yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  io::Gimbal gimbal(config_path,1);

  tools::Exiter exiter;
  tools::Plotter plotter;

  nlohmann::json data;
  auto t2 = std::chrono::steady_clock::now();

  while (!exiter.exit()) {
    auto t1 = std::chrono::steady_clock::now();
    gimbal.send_timestamp(std::chrono::steady_clock::now());
    std::this_thread::sleep_for(std::chrono::microseconds(100));

    auto [receive_time, send_time] = gimbal.queue_timestamp_.pop();
    data["latency"] = receive_time - send_time;
    plotter.plot(data);
    std::cout<<std::chrono::duration_cast<std::chrono::microseconds>( std::chrono::steady_clock::now()- t1).count()<<std::endl;
  }
  return 0;
}