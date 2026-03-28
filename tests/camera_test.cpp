#include "io/camera.hpp"

#include <opencv2/opencv.hpp>

#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

const std::string keys =
  "{help h usage ? |                     | 输出命令行参数说明}"
  "{config-path c  | configs/camera.yaml | yaml配置文件路径 }"
  "{d display      |                     | 显示视频流       }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;

  auto config_path = cli.get<std::string>("config-path");
  auto display = cli.has("display");
  io::Camera camera(config_path);

  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;
  auto last_stamp = std::chrono::steady_clock::now();
  while (!exiter.exit()) {
    camera.read(img, timestamp);

    auto dt = tools::delta_time(timestamp, last_stamp);
    last_stamp = timestamp;

    tools::logger()->info("{:.2f} fps", 1 / dt);

    if (!display) continue;
    cv::imshow("img", img);
    // 1. 转换为灰度图
    cv::Mat gray_img;
    cv::cvtColor(img, gray_img, cv::COLOR_BGR2GRAY);

    // 2. 高斯滤波
    cv::Mat blur_img;
    cv::GaussianBlur(gray_img, blur_img, cv::Size(5, 5), 0);
    cv::imshow("Gaussian Blur", blur_img);

    // 3. 二值化
    cv::Mat binary_img;
    cv::threshold(blur_img, binary_img, 128, 255, cv::THRESH_BINARY);
    cv::imshow("Binary Image", binary_img);

    // 4. 开闭运算
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));

    // 开运算
    cv::Mat open_img;
    cv::morphologyEx(binary_img, open_img, cv::MORPH_OPEN, kernel);
    cv::imshow("Open Operation", open_img);

    // 闭运算
    cv::Mat close_img;
    cv::morphologyEx(binary_img, close_img, cv::MORPH_CLOSE, kernel);
    cv::imshow("Close Operation", close_img);

    if (cv::waitKey(1) == 'q') break;
  }
}