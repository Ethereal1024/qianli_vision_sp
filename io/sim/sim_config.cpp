#include "sim_config.hpp"

#include "tools/yaml.hpp"

namespace io::sim
{

namespace
{
template <typename T>
void read_if(const YAML::Node & node, const std::string & key, T & value)
{
  if (node[key]) value = node[key].as<T>();
}
}  // namespace

SimShmConfig SimShmConfig::from_yaml(const std::string & config_path)
{
  SimShmConfig cfg;
  auto yaml = tools::load(config_path);
  auto node = yaml["sim_shm"];
  if (!node) {
    tools::logger()->warn(
      "[sim] {} has no sim_shm section, using default channel names", config_path);
    return cfg;
  }

  read_if(node, "camera_frames", cfg.camera_frames);
  read_if(node, "cboard_state", cfg.cboard_state);
  read_if(node, "cboard_cmd", cfg.cboard_cmd);

  read_if(node, "wait_timeout_ms", cfg.wait_timeout_ms);
  read_if(node, "open_retry_ms", cfg.open_retry_ms);
  read_if(node, "frame_lag_ms", cfg.frame_lag_ms);

  return cfg;
}

}  // namespace io::sim
