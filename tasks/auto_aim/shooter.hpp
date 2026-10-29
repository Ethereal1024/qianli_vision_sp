#ifndef AUTO_AIM__SHOOTER_HPP
#define AUTO_AIM__SHOOTER_HPP

#include <chrono>
#include <deque>
#include <string>
#include <utility>

#include "io/command.hpp"
#include "tasks/auto_aim/aimer.hpp"

namespace auto_aim
{
class Shooter
{
public:
  Shooter(const std::string & config_path);

  bool shoot(
    const io::Command & command, const auto_aim::Aimer & aimer,
    const std::list<auto_aim::Target> & targets, const Eigen::Vector3d & gimbal_pos,
    std::chrono::steady_clock::time_point t = std::chrono::steady_clock::now());

private:
  io::Command last_command_;
  double judge_distance_;
  double first_tolerance_;
  double second_tolerance_;
  double pitch_tolerance_ = 0;
  bool auto_fire_;
  double max_vel_change_ = 0;
  double max_vel_change_z_ = 0;
  double vel_window_ = 0.2;
  double steady_release_ = 0.5;
  double block_windows_ = 3.0;
  bool blocked_ = false;
  int block_frames_ = 0;
  double last_dt_ = 0.02;
  bool has_last_t_ = false;
  std::chrono::steady_clock::time_point last_t_;
  std::deque<std::pair<double, Eigen::Vector3d>> vel_hist_;
  bool steady(const Eigen::VectorXd & ex);
  bool segment_mean(double t_mid, bool newer, Eigen::Vector3d & v) const;
  bool plate_crossing(
    const auto_aim::Target & target, double tau, double gimbal_yaw, double tol) const;
};
}  // namespace auto_aim

#endif  // AUTO_AIM__SHOOTER_HPP