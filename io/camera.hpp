#ifndef IO__CAMERA_HPP
#define IO__CAMERA_HPP

#include <chrono>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>

namespace io
{
class CameraBase
{
public:
  virtual ~CameraBase() = default;
  virtual void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) = 0;
  virtual void read_bayerboth(cv::Mat & img_bgr,cv::Mat & img_bayer, std::chrono::steady_clock::time_point & timestamp) = 0;

};

class Camera
{
public:
  Camera(const std::string & config_path);
  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp);
  void read_bayerboth(cv::Mat & img_bgr,cv::Mat & img_bayer, std::chrono::steady_clock::time_point & timestamp);

private:
  std::unique_ptr<CameraBase> camera_;
};

}  // namespace io

#endif  // IO__CAMERA_HPP