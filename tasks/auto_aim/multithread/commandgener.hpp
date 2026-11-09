#ifndef AUTO_AIM_MULTITHREAD__HPP
#define AUTO_AIM_MULTITHREAD__HPP

#include <optional>

#include "io/cboard.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/omniperception/decider.hpp"
#include "tools/plotter.hpp"

namespace auto_aim
{
namespace multithread
{

// 主循环只读这份快照；直接读 aimer 的 debug_* 会与指令线程竞态
struct AimResult
{
  io::Command command{};
  AimPoint aim_point{false, Eigen::Vector4d::Zero(), -1};
  double dt = 0;        // 管道延迟（含发弹延迟）
  double fly_time = 0;
  std::chrono::steady_clock::time_point t;
  std::chrono::steady_clock::time_point input_t;
  bool has_value = false;
};

class CommandGener
{
public:
  CommandGener(
    auto_aim::Shooter & shooter, auto_aim::Aimer & aimer, io::CBoard & cboard,
    tools::Plotter & plotter, bool debug = false, bool use_shooter = true);

  ~CommandGener();

  void push(
    const std::list<auto_aim::Target> & targets, const std::chrono::steady_clock::time_point & t,
    double bullet_speed, const Eigen::Vector3d & gimbal_pos);

  AimResult last_result() const;

  // [diag] 需要与云台角同帧的指令，故等待该帧下发完成
  AimResult wait_result(const std::chrono::steady_clock::time_point & t, double timeout_s) const;

private:
  struct Input
  {
    std::list<auto_aim::Target> targets_;
    std::chrono::steady_clock::time_point t;
    // std::function<void()> decide;
    double bullet_speed;
    Eigen::Vector3d gimbal_pos;
  };

  io::CBoard & cboard_;
  auto_aim::Shooter & shooter_;
  auto_aim::Aimer & aimer_;
  tools::Plotter & plotter_;

  std::optional<Input> latest_;
  std::mutex mtx_;
  std::condition_variable cv_;
  std::thread thread_;
  bool stop_, debug_, use_shooter_;

  // 击发判决每帧一次：Shooter 的速度样本与机动守卫按每帧一次设计；
  // CMD_SHOOT_EVERY_TICK=1 恢复每 tick 判决
  std::optional<std::chrono::steady_clock::time_point> last_frame_t_;
  bool shoot_every_tick_ = false;
  bool have_shoot_ = false;
  bool last_shoot_ = false;

  mutable std::mutex result_mtx_;
  mutable std::condition_variable result_cv_;
  AimResult result_;

  void generate_command();
  void publish_result(const io::Command & command, const AimPoint & aim_point, double dt, double fly_time, const std::chrono::steady_clock::time_point & input_t);
};

}  // namespace multithread

}  // namespace auto_aim

#endif  // AUTO_AIM_MULTITHREAD__HPP
