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
    double h_table_gain = 0.0, double huber_k = 0.0);
  Target(double x, double vyaw, double radius, double h);

  void predict(std::chrono::steady_clock::time_point t);
  void predict(double dt);
  void update(const Armor & armor);

  Eigen::VectorXd ekf_x() const;
  const tools::ExtendedKalmanFilter & ekf() const;
  std::vector<Eigen::Vector4d> armor_xyza_list() const;

  bool diverged() const;

  bool convergened();

  // convergened() 会更新收敛标志，故开火守卫要用这个只读版本
  bool is_converged() const { return is_converged_; }

  // 第 4 维（板角度）新息及其预测 sd（含 R，rad），供离线标定
  double res4() const { return res4_; }
  double sd4() const { return sd4_; }

  bool isinit = false;

  bool checkinit();

  // 板高修正量（米）
  double h_table_gain() const { return h_table_gain_; }
  double h_pair_a() const { return h_pair_a_; }
  double h_pair_b() const { return h_pair_b_; }

private:
  int armor_num_;
  int switch_count_;
  int update_count_;

  bool is_switch_, is_converged_;

  double q_accel_xy_, q_accel_z_, q_accel_w_;
  double res4_ = 0.0, sd4_ = 0.0;
  double r_yaw_, r_pitch_;

  tools::ExtendedKalmanFilter ekf_;
  std::chrono::steady_clock::time_point t_;

  double h_table_gain_ = 0.0;
  // 归一化新息（单位 σ），<= 0 关闭
  double huber_k_ = 0.0;
  // 两对板（{0,2}、{1,3}）的板内高度差（米）；只取板内差才与 EKF 状态正交
  double h_pair_a_ = 0.0;
  double h_pair_b_ = 0.0;
  std::array<double, 4> h_meas_{{0, 0, 0, 0}};      // 逐板板高的慢平均，仅用于计算 a、b
  std::array<int, 4> h_meas_count_{{0, 0, 0, 0}};
  std::array<bool, 4> h_meas_valid_{{false, false, false, false}};

  void update_ypda(const Armor & armor, int id);  // yaw pitch distance angle
  void update_height_table(const Armor & armor, int id);

  Eigen::Vector3d h_armor_xyz(const Eigen::VectorXd & x, int id) const;
  Eigen::MatrixXd h_jacobian(const Eigen::VectorXd & x, int id) const;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TARGET_HPP