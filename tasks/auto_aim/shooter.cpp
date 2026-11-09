#include "shooter.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>

#include "tools/math_tools.hpp"

namespace auto_aim
{
Shooter::Shooter(const std::string & config_path) : last_command_{false, false, 0, 0}
{
  auto yaml = YAML::LoadFile(config_path);
  first_tolerance_ = yaml["first_tolerance"].as<double>() / 57.3;    // degree to rad
  second_tolerance_ = yaml["second_tolerance"].as<double>() / 57.3;  // degree to rad
  judge_distance_ = yaml["judge_distance"].as<double>();
  if (yaml["pitch_tolerance"]) pitch_tolerance_ = yaml["pitch_tolerance"].as<double>() / 57.3;
  auto_fire_ = yaml["auto_fire"].as<bool>();
}

bool Shooter::shoot(
  const io::Command & command, const auto_aim::Aimer & aimer,
  const std::list<auto_aim::Target> & targets, const Eigen::Vector3d & gimbal_pos,
  std::chrono::steady_clock::time_point /*t*/)
{
  if (targets.empty() || !command.control || !auto_fire_) {
    last_command_ = command;
    return false;
  }

  const auto ex = targets.front().ekf_x();
  auto target_x = ex[0];
  auto target_y = ex[2];
  auto tolerance = std::sqrt(tools::square(target_x) + tools::square(target_y)) > judge_distance_
                     ? second_tolerance_
                     : first_tolerance_;

  // 目标刚创建或刚重捕，状态未收敛，预测量不可信
  const bool converged = ex.size() >= 11 && targets.front().is_converged();

  if (
    converged &&
    std::abs(last_command_.yaw - command.yaw) < tolerance * 2 &&  //此时认为command突变不应该射击
    std::abs(gimbal_pos[0] - last_command_.yaw) < tolerance &&    //应该减去上一次command的yaw值
    // 云台 pitch 与 command.pitch 同号（均含下坠补偿），可直接相减
    (pitch_tolerance_ <= 0 || std::abs(gimbal_pos[1] - last_command_.pitch) < pitch_tolerance_) &&
    aimer.debug_aim_point.valid) {
    last_command_ = command;
    return true;
  }

  last_command_ = command;
  return false;
}

}  // namespace auto_aim
