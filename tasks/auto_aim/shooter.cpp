#include "shooter.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>

#include "tools/logger.hpp"
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
  if (yaml["max_vel_change"]) max_vel_change_ = yaml["max_vel_change"].as<double>();
  max_vel_change_z_ = yaml["max_vel_change_z"] ? yaml["max_vel_change_z"].as<double>()
                                               : max_vel_change_;
  if (yaml["vel_window"]) vel_window_ = yaml["vel_window"].as<double>();
  if (yaml["steady_release"]) steady_release_ = yaml["steady_release"].as<double>();
  if (yaml["block_windows"]) block_windows_ = yaml["block_windows"].as<double>();
}
bool Shooter::plate_crossing(
  const Target & target, double tau, double gimbal_yaw, double tol) const
{
  if (tau <= 0) return false;
  auto tg = target;
  tg.predict(tau);
  double best = 1e9;
  for (const auto & xyza : tg.armor_xyza_list()) {
    auto az = std::atan2(xyza[1], xyza[0]);
    best = std::min(best, std::abs(tools::limit_rad(az - gimbal_yaw)));
  }
  return best < tol;
}
bool Shooter::segment_mean(double t_mid, bool newer, Eigen::Vector3d & v) const
{
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  int n = 0;
  for (const auto & s : vel_hist_) {
    if ((s.first > t_mid) != newer) continue;
    sum += s.second;
    ++n;
  }
  if (n < 2) return false;
  v = sum / static_cast<double>(n);
  return true;
}
bool Shooter::steady(const Eigen::VectorXd & ex)
{
  if (max_vel_change_ <= 0 || ex.size() < 6) return true;
  const double t_now = std::chrono::duration<double>(last_t_.time_since_epoch()).count();
  vel_hist_.emplace_back(t_now, Eigen::Vector3d(ex[1], ex[3], ex[5]));
  while (t_now - vel_hist_.front().first > vel_window_) vel_hist_.pop_front();
  if (vel_hist_.size() < 4) return !blocked_;
  const double t0 = vel_hist_.front().first;
  const double t_mid = 0.5 * (t0 + t_now);
  Eigen::Vector3d v_old, v_new;
  if (!segment_mean(t_mid, false, v_old) || !segment_mean(t_mid, true, v_new)) return !blocked_;
  const Eigen::Vector3d dv = v_new - v_old;
  const double old_xy = std::hypot(v_old[0], v_old[1]);
  const double rev_xy =
    old_xy > 1e-3 ? -(dv[0] * v_old[0] + dv[1] * v_old[1]) / old_xy : 0.0;
  const double rev_z = std::abs(v_old[2]) > 1e-3 ? -dv[2] * std::copysign(1.0, v_old[2]) : 0.0;
  const bool trip =
    rev_xy >= max_vel_change_ || (max_vel_change_z_ > 0 && rev_z >= max_vel_change_z_);
  if (!blocked_) {
    if (trip) {
      blocked_ = true;
      block_frames_ = 0;
    }
    return !blocked_;
  }
  const bool quiet = rev_xy < steady_release_ * max_vel_change_ &&
                     (max_vel_change_z_ <= 0 || rev_z < steady_release_ * max_vel_change_z_);
  const int cap =
    std::max(1, static_cast<int>(std::lround(block_windows_ * vel_window_ / last_dt_)));
  if (quiet || ++block_frames_ > cap) {
    blocked_ = false;
    block_frames_ = 0;
    return true;
  }
  return false;
}

bool Shooter::shoot(
  const io::Command & command, const auto_aim::Aimer & aimer,
  const std::list<auto_aim::Target> & targets, const Eigen::Vector3d & gimbal_pos,
  std::chrono::steady_clock::time_point t)
{
  if (has_last_t_) {
    last_dt_ = std::clamp(tools::delta_time(t, last_t_), 0.004, 0.2);
  }
  last_t_ = t;
  has_last_t_ = true;

  if (targets.empty()) {
    vel_hist_.clear();
    blocked_ = false;
    block_frames_ = 0;
    last_command_ = command;
    return false;
  }
  if (!command.control || !auto_fire_) {
    vel_hist_.clear();
    blocked_ = false;
    block_frames_ = 0;
    last_command_ = command;
    return false;
  }
  const auto ex = targets.front().ekf_x();
  auto target_x = ex[0];
  auto target_y = ex[2];
  auto tolerance = std::sqrt(tools::square(target_x) + tools::square(target_y)) > judge_distance_
                     ? second_tolerance_
                     : first_tolerance_;
  // tools::logger()->debug("d(command.yaw) is {:.4f}", std::abs(last_command_.yaw - command.yaw));
  const bool ok = steady(ex);
  const bool converged = ex.size() >= 11 && targets.front().is_converged();
  const bool wait_plate = aimer.wait_plate_active();
  const bool wait_ok =
    !wait_plate || !ok || plate_crossing(
                            targets.front(), aimer.debug_dt + aimer.debug_fly_time, gimbal_pos[0],
                            aimer.wait_plate_tolerance());
  if (
    ok && converged && wait_ok &&
    std::abs(last_command_.yaw - command.yaw) < tolerance * 2 &&  //此时认为command突变不应该射击
    std::abs(gimbal_pos[0] - last_command_.yaw) < tolerance &&    //应该减去上一次command的yaw值
    (pitch_tolerance_ <= 0 || std::abs(gimbal_pos[1] - last_command_.pitch) < pitch_tolerance_) &&
    aimer.debug_aim_point.valid) {
    last_command_ = command;
    return true;
  }

  last_command_ = command;
  return false;
}

}  // namespace auto_aim