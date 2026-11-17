#include "communication.hpp"

#include <stdexcept>

#include "tools/logger.hpp"
#include "tools/yaml.hpp"

namespace io
{
Communication::Communication(const std::string & config_path)
{
  init_device(config_path);
}

Eigen::Quaterniond Communication::imu_at(std::chrono::steady_clock::time_point t)
{
  return std::visit([t](auto & device) { return device.imu_at(t); }, device_.value());
}

Mode Communication::mode() const
{
  if (const auto * cboard = std::get_if<CBoard>(&device_.value())) return cboard->state().mode;
  return static_cast<Mode>(std::get<Gimbal>(device_.value()).mode());
}

double Communication::bullet_speed() const
{
  if (const auto * cboard = std::get_if<CBoard>(&device_.value())) return cboard->state().bullet_speed;
  return std::get<Gimbal>(device_.value()).state().bullet_speed;
}

ShootMode Communication::shoot_mode() const
{
  if (const auto * cboard = std::get_if<CBoard>(&device_.value())) return cboard->state().shoot_mode;
  // 串口协议没有左右枪选择，使用默认的左枪瞄准偏置。
  return ShootMode::left_shoot;
}

std::string Communication::str(Mode mode) const
{
  return MODES.at(static_cast<std::size_t>(mode));
}

GimbalState Communication::state() const
{
  if (const auto * gimbal = std::get_if<Gimbal>(&device_.value())) return gimbal->state();
  throw std::logic_error("Communication::state requires mode: gimbal; CAN has no MPC feedback");
}

void Communication::send(Command command)
{
  if (auto * cboard = std::get_if<CBoard>(&device_.value())) {
    cboard->send(command);
  } else {
    std::get<Gimbal>(device_.value()).send(
      command.control, command.shoot, command.yaw, 0, 0, command.pitch, 0, 0);
  }
}

void Communication::send(VisionToGimbal command)
{
  if (command.mode > 2) throw std::invalid_argument("Invalid transmit mode: expected 0, 1 or 2");
  send(command.mode != 0, command.mode == 2, command.yaw, command.yaw_vel, command.yaw_acc,
       command.pitch, command.pitch_vel, command.pitch_acc);
}

void Communication::send(
  bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch,
  float pitch_vel, float pitch_acc)
{
  if (auto * gimbal = std::get_if<Gimbal>(&device_.value())) {
    gimbal->send(control, fire, yaw, yaw_vel, yaw_acc, pitch, pitch_vel, pitch_acc);
  } else {
    if (yaw_vel != 0 || yaw_acc != 0 || pitch_vel != 0 || pitch_acc != 0) {
      throw std::invalid_argument("MPC velocity/acceleration commands require mode: gimbal");
    }
    std::get<CBoard>(device_.value()).send(Command{control, fire, yaw, pitch});
  }
}

void Communication::init_device(const std::string & config_path)
{
  const auto config = tools::load(config_path);
  const auto mode = tools::read<std::string>(config, "mode", "");
  if (mode == "gimbal") {
    device_.emplace(std::in_place_type<Gimbal>, config_path);
    tools::logger()->info("[Communication] Using gimbal serial communication");
  } else if (mode == "cboard") {
    device_.emplace(std::in_place_type<CBoard>, config_path);
    tools::logger()->info("[Communication] Using CBoard CAN communication");
  } else {
    throw std::invalid_argument("Invalid communication mode '" + mode + "' in " + config_path +
                                "; expected gimbal or cboard");
  }
}
}  // namespace io
