#include "gimbal.hpp"

#include "tools/crc.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

namespace io
{
Gimbal::Gimbal(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto com_port = tools::read<std::string>(yaml, "com_port");

  try {
    serial_.setPort(com_port);
    serial_.setBaudrate(115200); 
    serial::Timeout time_out = serial::Timeout::simpleTimeout(20);
    serial_.setTimeout(time_out);
    serial_.open();
  } catch (const std::exception & e) {
    tools::logger()->error("[Gimbal] Failed to open serial: {}", e.what());
    exit(1);
  }
  //* 这样gimbal.q用在上一个标识中搜索
  thread_ = std::thread(&Gimbal::read_thread, this);

  // queue_.pop();
  // tools::logger()->info("[Gimbal] First q received.");
  wait_for_first_packet();
  thread_send_ = std::thread(&Gimbal::thread_send_thread, this);
}

Gimbal::~Gimbal()
{
  quit_ = true;
  if (thread_.joinable()) thread_.join();
  if (thread_send_.joinable()) thread_send_.join();
  serial_.close();
}

GimbalMode Gimbal::mode() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return mode_;
}

GimbalState Gimbal::state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

std::string Gimbal::str(GimbalMode mode) const
{
  switch (mode) {
    case GimbalMode::IDLE:
      return "IDLE";
    case GimbalMode::AUTO_AIM:
      return "AUTO_AIM";
    case GimbalMode::SMALL_BUFF:
      return "SMALL_BUFF";
    case GimbalMode::BIG_BUFF:
      return "BIG_BUFF";
    default:
      return "INVALID";
  }
}

Eigen::Quaterniond Gimbal::q(std::chrono::steady_clock::time_point t)
{
  while (true) {
    auto [q_a, t_a] = queue_.pop();
    auto [q_b, t_b] = queue_.front();
    auto t_ab = tools::delta_time(t_a, t_b);
    auto t_ac = tools::delta_time(t_a, t);
    auto k = t_ac / t_ab;
    Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();
    if (t < t_a) return q_c;
    if (!(t_a < t && t <= t_b)) continue;

    return q_c;
  }
}

void Gimbal::send_lose(io::GimbalState state)
{
  tx_data_.mode = 0;
  tx_data_.yaw = state.yaw;
  tx_data_.yaw_vel = 0;
  tx_data_.yaw_acc = 0;
  tx_data_.pitch = state.pitch;
  tx_data_.pitch_vel = 0;
  tx_data_.pitch_acc = 0;
  tx_data_.crc16 = tools::get_crc16(
    reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_) - sizeof(tx_data_.crc16));

  try {
    serial_.write(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_));
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
  }
}

void Gimbal::send_lose_ui(io::GimbalState state,double dist)
{
  tx_data_.mode = 0;
  tx_data_.yaw = state.yaw;
  tx_data_.yaw_vel = 0;
  tx_data_.yaw_acc = 0;
  tx_data_.pitch = state.pitch;
  tx_data_.pitch_vel = 0;
  tx_data_.pitch_acc = 0;
  tx_data_.dist = 0;
  tx_data_.crc16 = tools::get_crc16(
    reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_) - sizeof(tx_data_.crc16));

  try {
    serial_.write(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_));
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
  }
}

void Gimbal::send(io::VisionToGimbal VisionToGimbal)
{
  tx_data_.mode = VisionToGimbal.mode;
  tx_data_.yaw = VisionToGimbal.yaw;
  tx_data_.yaw_vel = VisionToGimbal.yaw_vel;
  tx_data_.yaw_acc = VisionToGimbal.yaw_acc;
  tx_data_.pitch = VisionToGimbal.pitch;
  tx_data_.pitch_vel = VisionToGimbal.pitch_vel;
  tx_data_.pitch_acc = VisionToGimbal.pitch_acc;
  tx_data_.crc16 = tools::get_crc16(
    reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_) - sizeof(tx_data_.crc16));

  try {
    serial_.write(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_));
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
  }
}

void Gimbal::send_ui(
  bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
  float pitch_acc,double dist)
{
  tx_data_.mode = control ? (fire ? 2 : 1) : 0;
  tx_data_.yaw = yaw;
  tx_data_.yaw_vel = yaw_vel;
  tx_data_.yaw_acc = yaw_acc;
  tx_data_.pitch = pitch;
  tx_data_.pitch_vel = pitch_vel;
  tx_data_.pitch_acc = pitch_acc;
  tx_data_.dist = dist;
  tx_data_.crc16 = tools::get_crc16(
    reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_) - sizeof(tx_data_.crc16));

  try {
    serial_.write(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_));
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
  }
}

void Gimbal::send(
  bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
  float pitch_acc)
{
  tx_data_.mode = control ? (fire ? 2 : 1) : 0;
  tx_data_.yaw = yaw;
  tx_data_.yaw_vel = yaw_vel;
  tx_data_.yaw_acc = yaw_acc;
  tx_data_.pitch = pitch;
  tx_data_.pitch_vel = pitch_vel;
  tx_data_.pitch_acc = pitch_acc;
  tx_data_.crc16 = tools::get_crc16(
    reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_) - sizeof(tx_data_.crc16));

  try {
    serial_.write(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_));
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
  }
}


//* 这里使用前一定要传入ros2的buffer
void Gimbal::send_sentry(
  bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
  float pitch_acc,float big_yaw,float x,float y,float z,float dir1,float dir2)
{
  tx_data_sentry_.mode = control ? (fire ? 2 : 1) : 0;
  tx_data_sentry_.yaw = yaw;
  tx_data_sentry_.yaw_vel = yaw_vel;
  tx_data_sentry_.yaw_acc = yaw_acc;
  tx_data_sentry_.pitch = pitch;
  tx_data_sentry_.pitch_vel = pitch_vel;
  tx_data_sentry_.pitch_acc = pitch_acc;
  tx_data_sentry_.big_yaw = big_yaw;
  tx_data_sentry_.x = x;
  tx_data_sentry_.y = y;
  tx_data_sentry_.z = z;
  tx_data_sentry_.dir1 = dir1;
  tx_data_sentry_.dir2 = dir2;

  tx_data_sentry_.crc16 = tools::get_crc16(
    reinterpret_cast<uint8_t *>(&tx_data_sentry_), sizeof(tx_data_sentry_) - sizeof(tx_data_sentry_.crc16));

  {
    std::lock_guard<std::mutex> lock(mutex_send_);
    thread_send_ready_.store(false);
    try {
      serial_.write(reinterpret_cast<uint8_t *>(&tx_data_sentry_), sizeof(tx_data_sentry_));
    } catch (const std::exception & e) {
      tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
    }
    thread_send_ready_.store(true);
    condition_send_.notify_one();
  }
}

void Gimbal::thread_send_thread(){
  while (!quit_){
    std::unique_lock<std::mutex> lock(mutex_send_);
    condition_send_.wait(lock, [this] { return thread_send_ready_.load()&&interface_buffer_!=nullptr&&!quit_; });
    if(interface_buffer_->empty()){
      lock.unlock();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }
    auto [x,y,z] = interface_buffer_->back();
    tx_data_sentry_.dir1 = x;
    tx_data_sentry_.dir2 = y;
    tx_data_sentry_.dir3 = z;
    tx_data_sentry_.crc16 = tools::get_crc16(
      reinterpret_cast<uint8_t *>(&tx_data_sentry_), sizeof(tx_data_sentry_) - sizeof(tx_data_sentry_.crc16));
    
    // lock.unlock(); 
    // serial_.write(reinterpret_cast<uint8_t *>(&tx_data_sentry_), sizeof(tx_data_sentry_));
    try {
      serial_.write(reinterpret_cast<uint8_t *>(&tx_data_sentry_), sizeof(tx_data_sentry_));
    } catch (const std::exception & e) {
      tools::logger()->warn("[Gimbal] Failed to write serial in thread_send_thread: {}", e.what());
      // 可以选择重连或继续循环
    }
    // thread_send_ready_.store(true);
    // condition_send_.notify_one();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

bool Gimbal::read(uint8_t * buffer, size_t size)
{
  try {
    return serial_.read(buffer, size) == size;
  } catch (const std::exception & e) {
    // tools::logger()->warn("[Gimbal] Failed to read serial: {}", e.what());
    return false;
  }
}

bool Gimbal::read_header() {
        uint8_t byte = 0;
        bool saw_s = false;

        while (!quit_) {
            if (!read(&byte, 1))
                return false;

            if (!saw_s) {
                saw_s = byte == 'S';
                continue;
            }

            if (byte == 'P') {
                rx_data_.head[0] = 'S';
                rx_data_.head[1] = 'P';
                return true;
            }

            saw_s = byte == 'S';
        }

        return false;
    }

void Gimbal::wait_for_first_packet() {
        constexpr auto retry_interval = std::chrono::seconds(5);
        auto last_retry = std::chrono::steady_clock::now();

        while (!quit_) {
            if (!queue_.empty()) {
                tools::logger()->info("[Gimbal] First q received.");
                return;
            }

            const auto now = std::chrono::steady_clock::now();
            if (now - last_retry >= retry_interval) {
                tools::logger()->warn(
                    "[Gimbal] Waiting for first valid packet timed out, reconnecting serial..."
                );
                reconnect();
                last_retry = now;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }


void Gimbal::read_thread()
{
  tools::logger()->info("[Gimbal] read_thread started.");
  int error_count = 0;
  tools::logger()->warn("[Gimbal] Invalid mode: {}", rx_data_.mode);

  while (!quit_) {
    if (error_count > 5000) {
      error_count = 0;
      tools::logger()->warn("[Gimbal] Too many errors, attempting to reconnect...");
      reconnect();
      continue;
    }

    if (!read(reinterpret_cast<uint8_t *>(&rx_data_), sizeof(rx_data_.head))) {
      error_count++;
      continue;
    }

    // if (rx_data_.head[0] != 'S' || rx_data_.head[1] != 'P') continue;
    if (!read_header()) {   //单字节搜寻SP
            error_count++;
            continue;
    }

    auto t = std::chrono::steady_clock::now();

    if (!read(
          reinterpret_cast<uint8_t *>(&rx_data_) + sizeof(rx_data_.head),
          sizeof(rx_data_) - sizeof(rx_data_.head))) {
      error_count++;
      continue;
    }

    if (!tools::check_crc16(reinterpret_cast<uint8_t *>(&rx_data_), sizeof(rx_data_))) {
      tools::logger()->debug("[Gimbal] CRC16 check failed.");
      continue;
    }

    error_count = 0;
    Eigen::Quaterniond q(rx_data_.q[0], rx_data_.q[1], rx_data_.q[2], rx_data_.q[3]);
    queue_.push({q, t});

    std::lock_guard<std::mutex> lock(mutex_);

    state_.yaw = rx_data_.yaw;
    state_.yaw_vel = rx_data_.yaw_vel;
    state_.pitch = rx_data_.pitch;
    state_.pitch_vel = rx_data_.pitch_vel;
    state_.bullet_speed = rx_data_.bullet_speed;
    state_.fire_calm = rx_data_.fire_calm;
    state_.enemy_color=rx_data_.enemy_color;
    state_.bullet_count = rx_data_.bullet_count;

    switch (rx_data_.mode) {
      case 0:
        mode_ = GimbalMode::IDLE;
        break;
      case 1:
        mode_ = GimbalMode::AUTO_AIM;
        break;
      case 2:
        mode_ = GimbalMode::SMALL_BUFF;
        break;
      case 3:
        mode_ = GimbalMode::BIG_BUFF;
        break;
      default:
        mode_ = GimbalMode::IDLE;
        tools::logger()->warn("[Gimbal] Invalid mode: {}", rx_data_.mode);
        break;
    }
  }

  tools::logger()->info("[Gimbal] read_thread stopped.");
}

void Gimbal::reconnect()
{
  int max_retry_count = 10;
  for (int i = 0; i < max_retry_count && !quit_; ++i) {
    tools::logger()->warn("[Gimbal] Reconnecting serial, attempt {}/{}...", i + 1, max_retry_count);
    try {
      serial_.close();
      std::this_thread::sleep_for(std::chrono::seconds(1));
    } catch (...) {
    }

    try {
      serial_.open();  // 尝试重新打开
      queue_.clear();
      queue_timestamp_.clear();
      tools::logger()->info("[Gimbal] Reconnected serial successfully.");
      break;
    } catch (const std::exception & e) {
      tools::logger()->warn("[Gimbal] Reconnect failed: {}", e.what());
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
}

/************************************************************************************************************************
 * @brief 测量通信用的各种实现
***********************************************************************************************************************/


Gimbal::Gimbal(const std::string & config_path, int use_timestamp)
{
  auto yaml = tools::load(config_path);
  auto com_port = tools::read<std::string>(yaml, "com_port");
  try {
    serial_.setPort(com_port);
    serial_.setBaudrate(115200);
    serial::Timeout time_out = serial::Timeout::simpleTimeout(20);
    serial_.setTimeout(time_out);
    serial_.open();
  } catch (const std::exception & e) {
    tools::logger()->error("[Gimbal] Failed to open serial: {}", e.what());
    exit(1);
  }
  thread_ = std::thread(&Gimbal::read_thread_timestamp, this);

  queue_timestamp_.pop();
  tools::logger()->info("[Gimbal] First timestamp received.");

  program_start_time_ = std::chrono::steady_clock::now();
}

void Gimbal::read_thread_timestamp(){
  tools::logger()->info("[Gimbal] read_thread started.");
  int error_count = 0;

  while (!quit_) {
    if (error_count > 5000) {
      error_count = 0;
      tools::logger()->warn("[Gimbal] Too many errors, attempting to reconnect...");
      reconnect();
      continue;
    }

    if (!read(reinterpret_cast<uint8_t *>(&rx_data_timestamp_), sizeof(rx_data_timestamp_.head))) {
      error_count++;
      std::cout<<"read error 1"<<std::endl;
      continue;
    }

    float t = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - program_start_time_).count();

    if (!read(
          reinterpret_cast<uint8_t *>(&rx_data_timestamp_) + sizeof(rx_data_timestamp_.head),
          sizeof(rx_data_timestamp_) - sizeof(rx_data_timestamp_.head))) {
      error_count++;
      std::cout<<"read error 2"<<std::endl;
      continue;
    }

    if (!tools::check_crc16(reinterpret_cast<uint8_t *>(&rx_data_timestamp_), sizeof(rx_data_timestamp_))) {
      tools::logger()->debug("[Gimbal] CRC16 check failed.");
      continue;
    }

    error_count = 0;
    float receive_time = rx_data_timestamp_.timestamp;
    std::cout << "receive_time: " << receive_time << std::endl;
    queue_timestamp_.push({receive_time,t});

    std::lock_guard<std::mutex> lock(mutex_);

  }

  tools::logger()->info("[Gimbal] read_thread stopped.");
}

void Gimbal::send_timestamp(std::chrono::steady_clock::time_point t)
{
  float time = std::chrono::duration_cast<std::chrono::microseconds>(t - program_start_time_).count();
  tx_data_timestamp_.timestamp = time;
  tx_data_timestamp_.crc16 = tools::get_crc16(
    reinterpret_cast<uint8_t *>(&tx_data_timestamp_), sizeof(tx_data_timestamp_) - sizeof(tx_data_timestamp_.crc16));
  serial_.write(reinterpret_cast<uint8_t *>(&tx_data_timestamp_), sizeof(tx_data_timestamp_));
}

}  // namespace io