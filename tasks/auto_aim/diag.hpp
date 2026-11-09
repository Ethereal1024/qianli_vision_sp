#ifndef AUTO_AIM__DIAG_HPP
#define AUTO_AIM__DIAG_HPP

#include <Eigen/Dense>
#include <chrono>
#include <list>

#include "io/command.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/target.hpp"

namespace auto_aim
{

struct DiagFrame
{
  int frame = 0;
  double t = 0;  // 秒，steady_clock 绝对时间
  const std::list<Armor> & armors;
  const std::list<Target> & targets;
  const Eigen::Vector3d & ypr;
  const io::Command & command;
  const AimPoint & aim_point;
  double aim_dt = 0;    // 管道延迟（含发弹延迟）
  double fly_time = 0;
};

class Diag
{
public:
  explicit Diag(bool enabled) : on_(enabled) {}

  bool on() const { return on_; }

  void log(const DiagFrame & f);

private:
  bool on_;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__DIAG_HPP
