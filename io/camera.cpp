#include "camera.hpp"

#include <stdexcept>

#include "hikrobot/hikrobot.hpp"
#include "mindvision/mindvision.hpp"
#include "tools/yaml.hpp"

namespace io
{
Camera::Camera(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto camera_name = tools::read<std::string>(yaml, "camera_name");
  auto exposure_ms = tools::read<double>(yaml, "exposure_ms");
  auto img_type = tools::read<double>(yaml, "img_type");

  // if (camera_name == "mindvision") {
  //   auto gamma = tools::read<double>(yaml, "gamma");
  //   auto vid_pid = tools::read<std::string>(yaml, "vid_pid");
  //   camera_ = std::make_unique<MindVision>(exposure_ms, gamma, vid_pid);
  // }
  //*不用就直接注释了

  if (camera_name == "hikrobot") {
    auto gain = tools::read<double>(yaml, "gain");
    auto vid_pid = tools::read<std::string>(yaml, "vid_pid");
    camera_ = std::make_unique<HikRobot>(exposure_ms, gain, vid_pid,img_type);
  }

  else {
    throw std::runtime_error("Unknow camera_name: " + camera_name + "!");
  }
}

void Camera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  camera_->read(img, timestamp);
}

void Camera::read_bayerboth(cv::Mat & img_bgr,cv::Mat & img_bayer, std::chrono::steady_clock::time_point & timestamp)
{
  camera_->read_bayerboth(img_bgr,img_bayer, timestamp);
}

}  // namespace io