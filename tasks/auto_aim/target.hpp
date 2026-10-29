#ifndef AUTO_AIM__TARGET_HPP
#define AUTO_AIM__TARGET_HPP

#include <array>
#include <Eigen/Dense>
#include <chrono>
#include <optional>
#include <queue>
#include <string>
#include <vector>

#include "armor.hpp"
#include "tools/extended_kalman_filter.hpp"

namespace auto_aim
{

class Target
{
public:
  ArmorName name;
  ArmorType armor_type;
  ArmorPriority priority;
  bool jumped;
  int last_id;  // debug only

  Target() = default;
  Target(
    const Armor & armor, std::chrono::steady_clock::time_point t, double radius, int armor_num,
    Eigen::VectorXd P0_dig, double q_accel_xy = 100.0, double q_accel_z = 100.0,
    double q_accel_w = 400.0, double r_yaw = 4e-3, double r_pitch = 4e-3,
    double qw_alpha0 = 1.0, double qw_scale = 0.6, double qw_gain = 1.0,
    double qz_spin_w0 = 0.0, double qz_spin_scale = 2.0);
  Target(double x, double vyaw, double radius, double h);

  void predict(std::chrono::steady_clock::time_point t);
  void predict(double dt);
  void update(const Armor & armor);

  Eigen::VectorXd ekf_x() const;
  const tools::ExtendedKalmanFilter & ekf() const;
  std::vector<Eigen::Vector4d> armor_xyza_list() const;

  bool diverged() const;

  bool convergened();

  bool is_converged() const { return is_converged_; }
  double res4() const { return res4_; }
  double sd4() const { return sd4_; }
  bool isinit = false;

  bool checkinit();

private:
  int armor_num_;
  int switch_count_;
  int update_count_;

  bool is_switch_, is_converged_;

  double q_accel_xy_, q_accel_z_, q_accel_w_;
  double qw_alpha0_ = 1.0, qw_scale_ = 0.6, qw_gain_ = 1.0;
  static constexpr int kQwHist = 64;
  std::array<double, kQwHist> qw_hist_w_{};
  std::array<double, kQwHist> qw_hist_t_{};
  int qw_hist_n_ = 0, qw_hist_i_ = 0;
  double qw_clock_ = 0.0, qw_d2_ = 0.0, qw_mult_ = 1.0;
  bool qw_d2_init_ = false;
  double qz_spin_w0_ = 0.0, qz_spin_scale_ = 2.0;
  double qz_mult_ = 1.0;
  double res4_ = 0.0, sd4_ = 0.0;
  double r_yaw_, r_pitch_;
  tools::ExtendedKalmanFilter ekf_;
  std::chrono::steady_clock::time_point t_;

  void update_ypda(const Armor & armor, int id);  // yaw pitch distance angle

  Eigen::Vector3d h_armor_xyz(const Eigen::VectorXd & x, int id) const;
  Eigen::MatrixXd h_jacobian(const Eigen::VectorXd & x, int id) const;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TARGET_HPP