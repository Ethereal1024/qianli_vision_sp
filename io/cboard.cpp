#include "cboard.hpp"

#include <stdexcept>

#include "io/canboard/canboard.hpp"
#include "io/simcboard/simcboard.hpp"
#include "tools/yaml.hpp"

namespace io
{
CBoard::CBoard(const std::string & config_path)
: bullet_speed(0), mode(Mode::idle), shoot_mode(ShootMode::left_shoot), ft_angle(0)
{
  auto yaml = tools::load(config_path);

  std::string cboard_name = "can";
  if (yaml["cboard_name"]) cboard_name = yaml["cboard_name"].as<std::string>();

  if (cboard_name == "can") {
    board_ = std::make_unique<CanBoard>(*this, config_path);
  }

  else if (cboard_name == "sim") {
    board_ = std::make_unique<sim::SimCBoard>(*this, config_path);
  }

  else {
    throw std::runtime_error("Unknow cboard_name: " + cboard_name + "!");
  }
}

Eigen::Quaterniond CBoard::imu_at(std::chrono::steady_clock::time_point timestamp)
{
  return board_->imu_at(timestamp);
}

void CBoard::send(Command command) { board_->send(command); }

}  // namespace io
