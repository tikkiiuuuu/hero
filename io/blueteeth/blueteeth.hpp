#pragma once

#include <memory>
#include <string>
#include <thread>
#include <atomic>

#include <Eigen/Dense>

#include "serial/serial.h"

namespace io
{

struct Frame
{
  int head;
  Eigen::Matrix4Xd data[4][3];
  int tail;
};


class FrameStrategy
{
public:
  virtual ~FrameStrategy() = default;
  virtual void process(const Frame & frame) = 0;
};


class SampleStrategy : public FrameStrategy
{
public:
  void process(const Frame & frame) override;
};


class ConcreteStrategy2 : public FrameStrategy
{
public:
  void process(const Frame & frame) override;
};


class BluetoothReceiver
{
public:

  BluetoothReceiver(const std::string & port);
  ~BluetoothReceiver();

  void set_strategy(std::unique_ptr<FrameStrategy> strategy);

private:

  void receive_thread();

  serial::Serial serial_;
  std::unique_ptr<FrameStrategy> strategy_;
  std::thread thread_;
  std::atomic<bool> quit_{false};
};

}  // namespace io