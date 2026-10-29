#ifndef IO__SIMCAMERA__SIMCAMERA_HPP
#define IO__SIMCAMERA__SIMCAMERA_HPP

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/sim/sim_config.hpp"
#include "io/sim/simg.hpp"

namespace io::sim
{

class SimCamera : public CameraBase
{
public:
  explicit SimCamera(const std::string & config_path);

  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) override;

private:
  void open(bool throw_on_timeout);
  void sync_geometry();

  SimShmConfig cfg_;
  ShmChannel frames_;
  std::vector<uint8_t> buffer_;
  int width_ = 0;
  int height_ = 0;
};

}  // namespace io::sim

#endif  // IO__SIMCAMERA__SIMCAMERA_HPP
