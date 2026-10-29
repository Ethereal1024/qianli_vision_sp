#ifndef IO__SIM__SIM_CONFIG_HPP
#define IO__SIM__SIM_CONFIG_HPP

#include <string>

namespace io::sim
{

struct SimShmConfig
{
  std::string camera_frames = "/dev/shm/sim_camera_frames";
  std::string cboard_state = "/dev/shm/sim_cboard_state";
  std::string cboard_cmd = "/dev/shm/sim_cboard_cmd";

  int wait_timeout_ms = 8000;
  int open_retry_ms = 500;

  static SimShmConfig from_yaml(const std::string & config_path);
};

}  // namespace io::sim

#endif  // IO__SIM__SIM_CONFIG_HPP
