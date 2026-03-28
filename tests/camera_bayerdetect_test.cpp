#include <fmt/core.h>

#include <chrono>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明 }"
  "{@config-path   | configs/standard3.yaml    | yaml配置文件的路径}"
  "{tradition t    |  false               | 是否使用传统方法识别}";

int main(int argc, char * argv[])
{
  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>(0);
  auto use_tradition = cli.get<bool>("tradition");

  tools::Exiter exiter;

  io::Camera camera(config_path);
  auto_aim::Detector detector(config_path, true);
  auto_aim::YOLO yolo(config_path, true);

  std::chrono::steady_clock::time_point timestamp;//* 为了比较图像插值的时间
  auto last_timestamp = std::chrono::steady_clock::now();

  while (!exiter.exit()) {
    cv::Mat img,img_bayer;
    std::list<auto_aim::Armor> armors;

    // camera.read(img, timestamp);
    camera.read_bayerboth(img,img_bayer,timestamp);
    auto img_dt = tools::delta_time(timestamp, last_timestamp);
    last_timestamp =timestamp;
    tools::logger()->info("{:.4f} img_dt", img_dt);
    
    if (img.empty()) break;

    auto last = std::chrono::steady_clock::now();

    if (use_tradition)
      armors = detector.detect(img);
    else
      armors = yolo.detect(img);

    auto now = std::chrono::steady_clock::now();
    auto dt = tools::delta_time(now, last);
    tools::logger()->info("{:.2f} fps", 1 / dt);

    for (const auto & armor : armors) {
      for (int i = 0; i < 4; i++) {
        cv::line(img, armor.points[i], armor.points[(i + 1) % 4], cv::Scalar(0, 255, 0), 1);
      }
    }

    cv::Mat show_img;
    cv::Mat show_img_bayer;
    
    // cv::resize(img, show_img, cv::Size(), 0.5, 0.5); 
    // cv::resize(img_bayer, show_img_bayer, cv::Size(), 0.5, 0.5);
    cv::imshow("Camera Detect Result", img);
    cv::imshow("bayer",img_bayer);

    // auto key = cv::waitKey(33);
    // if (key == 'q') break;
  }

  return 0;
}