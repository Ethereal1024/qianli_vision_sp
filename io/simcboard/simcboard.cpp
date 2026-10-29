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
        "[sim] cboard state channel " + cfg_.cboard_state
        + " not found (Unity side not running?)");
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
  Eigen::Quaterniond q{load_f64(p, 0), load_f64(p, 8), load_f64(p, 16), load_f64(p, 24)};
  if (q.norm() < 1e-6) return false;
  q.normalize();

  const uint32_t raw_mode = load_u32(p, 32);
  if (raw_mode < io::MODES.size()) owner_.mode = static_cast<Mode>(raw_mode);
  const uint32_t raw_shoot = load_u32(p, 36);
  if (raw_shoot < io::SHOOT_MODES.size()) owner_.shoot_mode = static_cast<ShootMode>(raw_shoot);
  owner_.bullet_speed = load_f64(p, 40);
  owner_.ft_angle = load_f64(p, 48);

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
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    const auto now = std::chrono::steady_clock::now();
    if (!have_command_) {
      prev_command_ = command;
      have_command_ = true;
    } else {
      const double dt = std::chrono::duration<double>(now - prev_send_t_).count();
      if (dt > 1e-4 && dt < 1.0) cmd_period_s_ = 0.8 * cmd_period_s_ + 0.2 * dt;
      prev_command_ = latest_command_;
    }
    latest_command_ = command;
    prev_send_t_ = now;
  }
  start_publisher();
}

void SimCBoard::start_publisher()
{
  if (publisher_started_) return;
  publisher_started_ = true;
  if (!cmd_ch_.opened()) cmd_ch_.open_writer(cfg_.cboard_cmd, 4, SIM_CBOARD_CMD_SIZE);
  publisher_ = std::thread([this] { publish_loop(); });
}

SimCBoard::~SimCBoard()
{
  publisher_stop_ = true;
  if (publisher_.joinable()) publisher_.join();
}

void SimCBoard::publish_loop()
{
  constexpr auto kPeriod = std::chrono::microseconds(2500);
  constexpr double kMaxLag = 0.004;
  constexpr double kMaxRate = 120.0 * M_PI / 180.0;
  Command out{};
  bool have_out = false;
  auto last = std::chrono::steady_clock::now();
  auto next = last;
  while (!publisher_stop_) {
    Command target, base;
    double period = 0.015;
    std::chrono::steady_clock::time_point t_send;
    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      target = latest_command_;
      base = prev_command_;
      period = cmd_period_s_;
      t_send = prev_send_t_;
    }
    const auto now = std::chrono::steady_clock::now();
    const double dt = std::chrono::duration<double>(now - last).count();
    last = now;
    if (!have_out || !target.control) {
      out = target;
      have_out = true;
    } else {
      const double f = std::clamp(
        std::chrono::duration<double>(now - t_send).count() / std::max(period, 1e-3), 0.0, 1.0);
      const double want_yaw =
        std::remainder(base.yaw + f * std::remainder(target.yaw - base.yaw, 2 * M_PI), 2 * M_PI);
      const double want_pitch = base.pitch + f * (target.pitch - base.pitch);
      const double step = kMaxRate * std::max(dt, 0.0);
      const double dyaw = std::remainder(want_yaw - out.yaw, 2 * M_PI);
      out.yaw = std::remainder(out.yaw + std::clamp(dyaw, -step, step), 2 * M_PI);
      out.pitch = out.pitch + std::clamp(want_pitch - out.pitch, -step, step);
      out.control = target.control;
      out.shoot = target.shoot;
      out.horizon_distance = target.horizon_distance;
    }
    write_command(out);
    next += kPeriod;
    if (now - next > std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                           std::chrono::duration<double>(kMaxLag)))
      next = now;
    std::this_thread::sleep_until(next);
  }
}
void SimCBoard::write_command(const Command & command)
{
  if (!cmd_ch_.opened()) cmd_ch_.open_writer(cfg_.cboard_cmd, 4, SIM_CBOARD_CMD_SIZE);
  if (!cmd_ch_.opened()) return;

  uint8_t * p = cmd_buffer_.data();
  std::memset(p, 0, cmd_buffer_.size());
  store_u32(p, 0, command.control ? 1u : 0u);
  store_u32(p, 4, command.shoot ? 1u : 0u);
  store_f64(p, 8, command.yaw);
  store_f64(p, 16, command.pitch);
  store_f64(p, 24, command.horizon_distance);
  cmd_ch_.write(p, cmd_buffer_.size());
}

}  // namespace io::sim
