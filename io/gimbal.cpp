#include "gimbal.hpp"

#include <array>
#include <cstring>

#include "tools/crc.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

namespace io
{
Gimbal::Gimbal(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto com_port = tools::read<std::string>(yaml, "com_port", "/dev/ttyACM0");
  auto baudrate = tools::read<uint32_t>(yaml, "baudrate", 921600);

  try {
    serial_.setPort(com_port);
    serial_.setBaudrate(baudrate);
    serial_.setFlowcontrol(serial::flowcontrol_none);
    serial_.setParity(serial::parity_none);
    serial_.setStopbits(serial::stopbits_one);
    serial_.setBytesize(serial::eightbits);
    auto timeout = serial::Timeout::simpleTimeout(20);
    serial_.setTimeout(timeout);
    serial_.open();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    serial_.flushInput();
  } catch (const std::exception & e) {
    tools::logger()->error("[Gimbal] Failed to open serial: {}", e.what());
    exit(1);
  }

  thread_ = std::thread(&Gimbal::read_thread, this);

  queue_.pop();
  tools::logger()->info("[Gimbal] First q received.");
}

Gimbal::~Gimbal()
{
  quit_ = true;
  if (thread_.joinable()) thread_.join();
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

Eigen::Quaterniond Gimbal::imu_at(std::chrono::steady_clock::time_point t)
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

size_t Gimbal::read(uint8_t * buffer, size_t size)
{
  try {
    // The serial library waits for the requested bytes up to its configured timeout.
    return serial_.read(buffer, size);
  } catch (const std::exception & e) {
    tools::logger()->debug("[Gimbal] Failed to read serial: {}", e.what());
    return 0;
  }
}

void Gimbal::read_thread()
{
  tools::logger()->info("[Gimbal] read_thread started.");
  int error_count = 0;
  int header_read_fail = 0;
  int body_read_fail = 0;
  int bad_header = 0;
  int crc_fail = 0;
  std::array<uint8_t, sizeof(GimbalToVision)> buffer{};
  size_t buffered = 0;
  bool header_found = false;
  std::chrono::steady_clock::time_point t;

  auto discard_byte = [&] {
    --buffered;
    std::memmove(buffer.data(), buffer.data() + 1, buffered);
    header_found = false;
  };

  while (!quit_) {
    if (error_count > 5000) {
      error_count = 0;
      tools::logger()->warn(
        "[Gimbal] Too many errors, attempting to reconnect... "
        "(header_read_fail={}, body_read_fail={}, bad_header={}, crc_fail={})",
        header_read_fail, body_read_fail, bad_header, crc_fail);
      header_read_fail = 0;
      body_read_fail = 0;
      bad_header = 0;
      crc_fail = 0;
      buffered = 0;
      header_found = false;
      reconnect();
      continue;
    }

    const auto required = header_found ? buffer.size() : sizeof(rx_data_.head);
    if (buffered < required) {
      auto n = read(buffer.data() + buffered, required - buffered);
      buffered += n;
      if (n == 0) {
        ++error_count;
        if (header_found) ++body_read_fail;
        else ++header_read_fail;
      }
      // Keep partial data even when a read times out; it is not a frame boundary.
      if (buffered < required) continue;
    }

    if (!header_found) {
      if (buffer[0] != 'S' || buffer[1] != 'P') {
        ++error_count;
        ++bad_header;
        tools::logger()->debug(
          "[Gimbal] Invalid frame header: 0x{:02X} 0x{:02X}", buffer[0], buffer[1]);
        discard_byte();
        continue;
      }
      t = std::chrono::steady_clock::now();
      header_found = true;
      continue;
    }

    if (!tools::check_crc16(buffer.data(), buffer.size())) {
      ++error_count;
      ++crc_fail;
      tools::logger()->debug("[Gimbal] CRC16 check failed.");
      // A damaged frame can contain the start of the next valid frame.
      discard_byte();
      continue;
    }

    std::memcpy(&rx_data_, buffer.data(), sizeof(rx_data_));
    buffered = 0;
    header_found = false;
    error_count = 0;
    Eigen::Quaterniond q(rx_data_.q[0], rx_data_.q[1], rx_data_.q[2], rx_data_.q[3]);
    queue_.push({q, t});

    std::lock_guard<std::mutex> lock(mutex_);

    state_.yaw = rx_data_.yaw;
    state_.yaw_vel = rx_data_.yaw_vel;
    state_.pitch = rx_data_.pitch;
    state_.pitch_vel = rx_data_.pitch_vel;
    state_.bullet_speed = rx_data_.bullet_speed;
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
      std::this_thread::sleep_for(std::chrono::seconds(1));
      serial_.flushInput();
      queue_.clear();
      tools::logger()->info("[Gimbal] Reconnected serial successfully.");
      break;
    } catch (const std::exception & e) {
      tools::logger()->warn("[Gimbal] Reconnect failed: {}", e.what());
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
}

}  // namespace io
