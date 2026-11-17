#ifndef IO__COMMUNICATION_HPP
#define IO__COMMUNICATION_HPP

#include <optional>
#include <string>
#include <variant>
#include <Eigen/Geometry>
#include <chrono>

#include "cboard/cboard.hpp"
#include "gimbal/gimbal.hpp"

namespace io
{

class Communication
{
public:
  explicit Communication(const std::string & config_path);
  ~Communication() = default;

  Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point t);
  Mode mode() const;
  double bullet_speed() const;
  ShootMode shoot_mode() const;
  std::string str(Mode mode) const;

  // CAN协议没有云台位置/速度反馈，MPC状态只在gimbal模式下可用。
  GimbalState state() const;
  void send(Command command);
  void send(VisionToGimbal command);
  void send(
    bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch,
    float pitch_vel, float pitch_acc);

private:
  using Device = std::variant<Gimbal, CBoard>;

  std::optional<Device> device_;
  void init_device(const std::string & config_path);
};

}  // namespace io

#endif  // IO__COMMUNICATION_HPP
