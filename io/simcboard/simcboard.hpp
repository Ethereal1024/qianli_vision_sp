#ifndef IO__SIMCBOARD__SIMCBOARD_HPP
#define IO__SIMCBOARD__SIMCBOARD_HPP

#include <chrono>
#include <deque>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "io/cboard.hpp"
#include "io/sim/sim_config.hpp"
#include "io/sim/simg.hpp"

namespace io::sim
{

constexpr size_t SIM_CBOARD_STATE_SIZE = 64;
constexpr size_t SIM_CBOARD_CMD_SIZE = 32;

// 与 Unity 侧 CBoardShmLink 的写入布局一致，两侧需同步
enum : size_t
{
  kOffStateQw = 0,
  kOffStateQx = 8,
  kOffStateQy = 16,
  kOffStateQz = 24,
  kOffStateMode = 32,
  kOffStateShootMode = 36,
  kOffStateBulletSpeed = 40,
  kOffStateFtAngle = 48,
  kOffStateReserved = 56,
  kOffCmdControl = 0,
  kOffCmdShoot = 4,
  kOffCmdYaw = 8,
  kOffCmdPitch = 16,
  kOffCmdHorizonDistance = 24,
};

class SimCBoard : public CBoardBase
{
public:
  SimCBoard(CBoard & owner, const std::string & config_path);
  ~SimCBoard() override = default;

  Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp) override;

  void send(Command command) override;

private:
  struct Sample
  {
    Eigen::Quaterniond q;
    std::chrono::steady_clock::time_point t;
  };

  void tick();
  void open_state_channel();
  bool read_cboard_state();
  void push(Sample sample);

  void write_command(const Command & command);

  CBoard & owner_;

  SimShmConfig cfg_;
  ShmChannel state_ch_;
  ShmChannel cmd_ch_;

  std::deque<Sample> history_;
  std::vector<uint8_t> state_buffer_;
  std::chrono::steady_clock::time_point next_open_attempt_ =
    std::chrono::steady_clock::time_point::min();

  std::vector<uint8_t> cmd_buffer_;

  static constexpr size_t kHistory = 256;  // 2.5 s @100 Hz
};

}  // namespace io::sim

#endif  // IO__SIMCBOARD__SIMCBOARD_HPP
