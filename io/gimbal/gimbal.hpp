#ifndef IO__GIMBAL_HPP
#define IO__GIMBAL_HPP

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>
#include <tuple>
#include <condition_variable>
#include "serial/serial.h"
#include "tools/crc.hpp"
#include "tools/thread_safe_queue.hpp"

namespace io
{
struct __attribute__((packed)) GimbalToVision
{
  uint8_t head[2] = {'S', 'P'};
  uint8_t mode;  // 0: 空闲, 1: 自瞄, 2: 小符, 3: 大符
  float q[4];    // wxyz顺序
  float yaw;
  float yaw_vel;
  float pitch;
  float pitch_vel;
  float bullet_speed;
  float fire_calm; //开火冷却 1是可以开火
  uint8_t enemy_color; //敌方颜色 101：自身是蓝色，敌方红色  1：自身是红色，敌方蓝色
  uint16_t bullet_count;  // 子弹累计发送次数
  uint16_t crc16;
};

static_assert(sizeof(GimbalToVision) <= 64);

struct __attribute__((packed)) VisionToGimbal
{
  uint8_t head[2] = {'S', 'P'};
  uint8_t mode;  // 0: 不控制, 1: 控制云台但不开火，2: 控制云台且开火
  float yaw;
  float yaw_vel;
  float yaw_acc;
  float pitch;
  float pitch_vel;
  float pitch_acc;
  double dist;
  uint16_t crc16;
};

struct __attribute__((packed)) VisionToGimbal_sentry
{
  uint8_t head[2] = {'S', 'P'};
  uint8_t mode;  // 0: 不控制, 1: 控制云台但不开火，2: 控制云台且开火
  float yaw;
  float yaw_vel;
  float yaw_acc;
  float pitch;
  float pitch_vel;
  float pitch_acc;
  float big_yaw;
  uint8_t w1;
  float x;
  float y;
  float z;
  float dir1;
  float dir2;
  float dir3;
  float dir4;
  uint16_t crc16;
};

static_assert(sizeof(VisionToGimbal) <= 64);

enum class GimbalMode
{
  IDLE,        // 空闲
  AUTO_AIM,    // 自瞄
  SMALL_BUFF,  // 小符
  BIG_BUFF     // 大符
};

struct GimbalState
{
  float yaw;
  float yaw_vel;
  float pitch;
  float pitch_vel;
  float bullet_speed;
  float fire_calm;  //开火冷却
  uint8_t enemy_color; //敌方颜色 101：自身是蓝色，敌方红色  1：自身是红色，敌方蓝色
  uint16_t bullet_count;
};

class Gimbal
{
public:
  explicit Gimbal(const std::string & config_path);
  Gimbal(const std::string & config_path, int use_timestamp);

  ~Gimbal();

  GimbalMode mode() const;
  GimbalState state() const;
  std::string str(GimbalMode mode) const;
  Eigen::Quaterniond q(std::chrono::steady_clock::time_point t);
  Eigen::Quaterniond q(size_t prev_us);

  void send(
    bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
    float pitch_acc);
  void send_sentry(
      bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
      float pitch_acc,float big_yaw,float x,float y,float z,float dir1,float dir2);

  void send(io::VisionToGimbal VisionToGimbal);
  void send_lose(io::GimbalState state);

  void send_lose_ui(io::GimbalState state,double dist);
  void send_ui(
    bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
    float pitch_acc,double dist);
  void read_thread();
  void reconnect();
  bool read_header();
/************************************************************************************************************************
 * @brief 设计timestamp的都是测量通信用的
***********************************************************************************************************************/
  //!设计timestamp的都是测量通信用的
  struct __attribute__((packed)) VisionToGimbalTimestamp
  {
    uint8_t head[2] = {'S', 'P'};
    float timestamp;
    uint16_t crc16;
  };
  struct __attribute__((packed)) GimbalToVisionTimestamp
  {
    uint8_t head[2] = {'S', 'P'};
    float timestamp;
    uint16_t crc16;
  };

  void send_timestamp(std::chrono::steady_clock::time_point t);
  tools::ThreadSafeQueue<std::tuple<float,float>> queue_timestamp_{1000};

  //* 多线程通信
  //* 一定是传入了线程安全的数据结构，所以要求实现back等，已实现
  //* 这里懒了，也没必要写泛型
  // template<typename BufferType>
  // void set_interface(BufferType& buffer) {
  //     interface_buffer_ = &buffer;
  // }
  void set_interface(tools::ThreadSafeRingBuffer<std::tuple<float,float,float>, 10>& buffer){
    interface_buffer_ = &buffer;
  }
  //bool read_thread(){};
  void wait_for_first_packet();
private:
  serial::Serial serial_;

  std::thread thread_;
  std::atomic<bool> quit_ = false;
  mutable std::mutex mutex_;

  GimbalToVision rx_data_;
  VisionToGimbal tx_data_;
  VisionToGimbal_sentry tx_data_sentry_;

  GimbalMode mode_ = GimbalMode::IDLE;
  GimbalState state_;
  tools::ThreadSafeQueue<std::tuple<Eigen::Quaterniond, std::chrono::steady_clock::time_point>>
    queue_{100};
  // tools::TreadSafeRingBuffer<std::tuple<Eigen::Quaterniond, std::chrono::steady_clock::time_point>, 100> queue_;

  bool read(uint8_t * buffer, size_t size);

  /*通信延时用*/
  VisionToGimbalTimestamp tx_data_timestamp_;
  GimbalToVisionTimestamp rx_data_timestamp_;
  std::chrono::steady_clock::time_point program_start_time_;
  void read_thread_timestamp();
  
  //* 另一线程附加通信用
  std::thread thread_send_;
  std::atomic<bool> thread_send_ready_ = false;
  void thread_send_thread();
  std::condition_variable condition_send_;
  tools::ThreadSafeRingBuffer<std::tuple<float,float,float>, 10>* interface_buffer_ = nullptr;
    //*所以这里就永久引入了锁
  mutable std::mutex mutex_send_;
  // std::any interface_buffer_;
};

}  // namespace io

#endif  // IO__GIMBAL_HPP