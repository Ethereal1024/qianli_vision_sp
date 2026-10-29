#ifndef IO__CBOARD_HPP
#define IO__CBOARD_HPP

#include <Eigen/Geometry>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "io/command.hpp"

namespace io
{
enum Mode
{
  idle,
  auto_aim,
  small_buff,
  big_buff,
  outpost
};
const std::vector<std::string> MODES = {"idle", "auto_aim", "small_buff", "big_buff", "outpost"};

// 哨兵专有
enum ShootMode
{
  left_shoot,
  right_shoot,
  both_shoot
};
const std::vector<std::string> SHOOT_MODES = {"left_shoot", "right_shoot", "both_shoot"};

class CBoardBase
{
public:
  virtual ~CBoardBase() = default;

  virtual Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp) = 0;

  virtual void send(Command command) = 0;
};

class CBoard
{
public:
  double bullet_speed;
  Mode mode;
  ShootMode shoot_mode;
  double ft_angle;  //无人机专有

  CBoard(const std::string & config_path);

  Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp);

  void send(Command command);

private:
  // 驱动构造时会回写上面的公开字段，故声明在其后
  std::unique_ptr<CBoardBase> board_;
};

}  // namespace io

#endif  // IO__CBOARD_HPP
