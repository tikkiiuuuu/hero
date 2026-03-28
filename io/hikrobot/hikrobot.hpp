#ifndef IO__HIKROBOT_HPP
#define IO__HIKROBOT_HPP

#include <atomic>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>

#include "MvCameraControl.h"
#include "io/camera.hpp"
#include "tools/thread_safe_queue.hpp"

namespace io
{
class HikRobot : public CameraBase
{
public:
  HikRobot(double exposure_ms, double gain, const std::string & vid_pid, const int option);
  ~HikRobot() override;
  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) override;
  void read_bayerboth(cv::Mat & img_bgr,cv::Mat & img_bayer, std::chrono::steady_clock::time_point & timestamp) override;

private:
  struct CameraData
  {
    cv::Mat img;
    std::chrono::steady_clock::time_point timestamp;
  };
  struct CameraBayerBothData
  {
    cv::Mat img_bgr;
    cv::Mat img_bayer;
    std::chrono::steady_clock::time_point timestamp;
  };

  double exposure_us_;
  double gain_;

  std::thread daemon_thread_;
  std::atomic<bool> daemon_quit_;

  void * handle_;
  std::thread capture_thread_;
  std::atomic<bool> capturing_;
  std::atomic<bool> capture_quit_;
  tools::ThreadSafeQueue<CameraData> queue_;
  tools::ThreadSafeQueue<CameraBayerBothData> queue_both_;

  int vid_, pid_;

  void capture_start(const int option);//* 把插值前的时间作为图像时间戳
  //* 0为原始的只有双线性插值，1为bayer图像和最近邻插值
  void capture_loop();
  void capture_loop_bayerboth();
  void capture_stop();

  void set_float_value(const std::string & name, double value);
  void set_enum_value(const std::string & name, unsigned int value);

  void set_vid_pid(const std::string & vid_pid);
  void reset_usb() const;
};

}  // namespace io

#endif  // IO__HIKROBOT_HPP