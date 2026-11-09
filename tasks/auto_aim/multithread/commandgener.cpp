#include <cstdlib>

#include "commandgener.hpp"

#include "tools/math_tools.hpp"

namespace auto_aim
{
namespace multithread
{

CommandGener::CommandGener(
  auto_aim::Shooter & shooter, auto_aim::Aimer & aimer, io::CBoard & cboard,
  tools::Plotter & plotter, bool debug, bool use_shooter)
: shooter_(shooter),
  aimer_(aimer),
  cboard_(cboard),
  plotter_(plotter),
  stop_(false),
  debug_(debug),
  use_shooter_(use_shooter)
{
  shoot_every_tick_ = std::getenv("CMD_SHOOT_EVERY_TICK") != nullptr;
  thread_ = std::thread(&CommandGener::generate_command, this);
}

CommandGener::~CommandGener()
{
  {
    std::lock_guard<std::mutex> lock(mtx_);
    stop_ = true;
  }
  cv_.notify_all();
  if (thread_.joinable()) thread_.join();
}

void CommandGener::push(
  const std::list<auto_aim::Target> & targets, const std::chrono::steady_clock::time_point & t,
  double bullet_speed, const Eigen::Vector3d & gimbal_pos)
{
  std::lock_guard<std::mutex> lock(mtx_);
  latest_ = {targets, t, bullet_speed, gimbal_pos};
  cv_.notify_one();
}

void CommandGener::publish_result(
  const io::Command & command, const AimPoint & aim_point, double dt, double fly_time,
  const std::chrono::steady_clock::time_point & input_t)
{
  {
    std::lock_guard<std::mutex> lock(result_mtx_);
    result_.command = command;
    result_.aim_point = aim_point;
    result_.dt = dt;
    result_.fly_time = fly_time;
    result_.t = std::chrono::steady_clock::now();
    result_.input_t = input_t;
    result_.has_value = true;
  }
  result_cv_.notify_all();
}

AimResult CommandGener::last_result() const
{
  std::lock_guard<std::mutex> lock(result_mtx_);
  return result_;
}

AimResult CommandGener::wait_result(
  const std::chrono::steady_clock::time_point & t, double timeout_s) const
{
  std::unique_lock<std::mutex> lock(result_mtx_);
  result_cv_.wait_for(lock, std::chrono::duration<double>(timeout_s), [&] {
    return result_.has_value && result_.input_t == t;
  });
  return result_;
}

void CommandGener::generate_command()
{
  auto t0 = std::chrono::steady_clock::now();
  while (!stop_) {
    std::optional<Input> input;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      if (latest_ && tools::delta_time(std::chrono::steady_clock::now(), latest_->t) < 0.2) {
        input = latest_;
      } else
        input = std::nullopt;
    }
    if (input) {
      // 同一帧输入在 500 Hz 下被复用约 7 次
      const bool new_input = !last_frame_t_.has_value() || input->t != *last_frame_t_;
      if (new_input) last_frame_t_ = input->t;
      // 把预测量推到 now()
      auto command = aimer_.aim(input->targets_, input->t, input->bullet_speed);
      if (use_shooter_) {
        if (new_input || !have_shoot_ || shoot_every_tick_)
          last_shoot_ = shooter_.shoot(command, aimer_, input->targets_, input->gimbal_pos);
        command.shoot = last_shoot_;
        have_shoot_ = true;
      } else
        command.shoot = false;
      command.horizon_distance = input->targets_.empty()
                                   ? 0
                                   : std::sqrt(
                                       tools::square(input->targets_.front().ekf_x()[0]) +
                                       tools::square(input->targets_.front().ekf_x()[2]));
      cboard_.send(command);
      // debug_* 只由本线程写
      publish_result(
        command, aimer_.debug_aim_point, aimer_.debug_dt, aimer_.debug_fly_time, input->t);
      if (debug_) {
        nlohmann::json data;
        data["t"] = tools::delta_time(std::chrono::steady_clock::now(), t0);
        data["cmd_yaw"] = command.yaw * 57.3;
        data["cmd_pitch"] = command.pitch * 57.3;
        data["shoot"] = command.shoot;
        data["horizon_distance"] = command.horizon_distance;
        plotter_.plot(data);
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));  //approximately 500Hz
  }
}

}  // namespace multithread

}  // namespace auto_aim
