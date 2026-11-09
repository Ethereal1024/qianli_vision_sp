#include "simcboard.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <thread>

#include "tools/logger.hpp"

namespace io::sim
{

namespace
{
uint32_t load_u32(const uint8_t * p, size_t off)
{
  uint32_t v = 0;
  std::memcpy(&v, p + off, sizeof(v));
  return v;
}

double load_f64(const uint8_t * p, size_t off)
{
  double v = 0;
  std::memcpy(&v, p + off, sizeof(v));
  return v;
}

void store_u32(uint8_t * p, size_t off, uint32_t v) { std::memcpy(p + off, &v, sizeof(v)); }
void store_f64(uint8_t * p, size_t off, double v) { std::memcpy(p + off, &v, sizeof(v)); }
}  // namespace

SimCBoard::SimCBoard(CBoard & owner, const std::string & config_path)
: owner_(owner), cfg_(SimShmConfig::from_yaml(config_path)),
  state_buffer_(SIM_CBOARD_STATE_SIZE, 0), cmd_buffer_(SIM_CBOARD_CMD_SIZE, 0)
{
  if (!cmd_ch_.open_writer(cfg_.cboard_cmd, 4, SIM_CBOARD_CMD_SIZE))
    tools::logger()->warn("[sim] failed to create command channel {}", cfg_.cboard_cmd);

  tools::logger()->info("[sim] waiting for cboard state channel {}", cfg_.cboard_state);
  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::milliseconds(cfg_.wait_timeout_ms);
  while (!read_cboard_state()) {
    open_state_channel();
    if (std::chrono::steady_clock::now() >= deadline)
      throw std::runtime_error(
        "[sim] cboard state channel " + cfg_.cboard_state +
        " not found (Unity side CBoardShmLink running?)");
    std::this_thread::sleep_for(std::chrono::milliseconds(std::max(1, cfg_.open_retry_ms)));
  }
}

void SimCBoard::open_state_channel()
{
  if (state_ch_.opened()) return;
  if (!state_ch_.open_reader(cfg_.cboard_state)) return;

  if (state_buffer_.size() < state_ch_.slot_size()) state_buffer_.resize(state_ch_.slot_size(), 0);
}

void SimCBoard::tick()
{
  const auto now = std::chrono::steady_clock::now();
  if (!state_ch_.opened() && now >= next_open_attempt_) {
    next_open_attempt_ = now + std::chrono::milliseconds(cfg_.open_retry_ms > 0 ? cfg_.open_retry_ms : 500);
    open_state_channel();
  }

  if (state_ch_.opened()) read_cboard_state();

  if (history_.size() > kHistory) history_.pop_front();
}

bool SimCBoard::read_cboard_state()
{
  uint64_t stamp_ns = 0;
  const int n = state_ch_.read(state_buffer_.data(), state_buffer_.size(), &stamp_ns);
  if (n <= 0) return false;

  const uint8_t * p = state_buffer_.data();
  Eigen::Quaterniond q{
    load_f64(p, kOffStateQw), load_f64(p, kOffStateQx), load_f64(p, kOffStateQy),
    load_f64(p, kOffStateQz)};
  if (q.norm() < 1e-6) return false;  // 仿真侧尚未写入
  q.normalize();

  const uint32_t raw_mode = load_u32(p, kOffStateMode);
  if (raw_mode < io::MODES.size()) owner_.mode = static_cast<Mode>(raw_mode);
  const uint32_t raw_shoot = load_u32(p, kOffStateShootMode);
  if (raw_shoot < io::SHOOT_MODES.size()) owner_.shoot_mode = static_cast<ShootMode>(raw_shoot);
  owner_.bullet_speed = load_f64(p, kOffStateBulletSpeed);
  owner_.ft_angle = load_f64(p, kOffStateFtAngle);

  // 时间戳取写者写入时刻而非读取时刻：后者偏晚，会使插值超前于画面
  push({q, stamp_ns != 0 ? steady_from_realtime(stamp_ns) : std::chrono::steady_clock::now()});
  return true;
}

void SimCBoard::push(Sample sample)
{
  if (!history_.empty() && sample.t <= history_.back().t) return;
  history_.push_back(sample);
}

Eigen::Quaterniond SimCBoard::imu_at(std::chrono::steady_clock::time_point timestamp)
{
  tick();

  if (history_.empty()) return Eigen::Quaterniond::Identity();

  if (timestamp >= history_.back().t) return history_.back().q;
  if (timestamp <= history_.front().t) return history_.front().q;

  size_t i = history_.size() - 1;
  while (i > 0 && history_[i - 1].t > timestamp) i--;
  const auto & a = history_[i - 1];
  const auto & b = history_[i];

  std::chrono::duration<double> t_ab = b.t - a.t;
  std::chrono::duration<double> t_ac = timestamp - a.t;
  if (t_ab.count() <= 0) return b.q;

  Eigen::Quaterniond q_a = a.q.normalized();
  Eigen::Quaterniond q_b = b.q.normalized();
  const double k = t_ac.count() / t_ab.count();
  return q_a.slerp(k, q_b).normalized();
}

void SimCBoard::send(Command command)
{
  write_command(command);
}

void SimCBoard::write_command(const Command & command)
{
  if (!cmd_ch_.opened()) cmd_ch_.open_writer(cfg_.cboard_cmd, 4, SIM_CBOARD_CMD_SIZE);
  if (!cmd_ch_.opened()) return;

  uint8_t * p = cmd_buffer_.data();
  std::memset(p, 0, cmd_buffer_.size());
  store_u32(p, kOffCmdControl, command.control ? 1u : 0u);
  store_u32(p, kOffCmdShoot, command.shoot ? 1u : 0u);
  store_f64(p, kOffCmdYaw, command.yaw);
  store_f64(p, kOffCmdPitch, command.pitch);
  store_f64(p, kOffCmdHorizonDistance, command.horizon_distance);
  cmd_ch_.write(p, cmd_buffer_.size());
}

}  // namespace io::sim
