#ifndef IO__CANBOARD__CANBOARD_HPP
#define IO__CANBOARD__CANBOARD_HPP

#include <linux/can.h>

#include <Eigen/Geometry>
#include <chrono>
#include <memory>
#include <string>

#include "io/cboard.hpp"
#include "tools/thread_safe_queue.hpp"

namespace io
{
class SocketCAN;

class CanBoard : public CBoardBase
{
public:
  CanBoard(CBoard & owner, const std::string & config_path);

  ~CanBoard() override;

  Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp) override;

  void send(Command command) override;

private:
  struct IMUData
  {
    Eigen::Quaterniond q;
    std::chrono::steady_clock::time_point timestamp;
  };

  void callback(const can_frame & frame);

  std::string read_yaml(const std::string & config_path);

  CBoard & owner_;
  // 接收回调会写入该队列，故先于 can_ 声明
  tools::ThreadSafeQueue<IMUData> queue_;
  std::unique_ptr<SocketCAN> can_;
  IMUData data_ahead_;
  IMUData data_behind_;

  int quaternion_canid_, bullet_speed_canid_, send_canid_;
};

}  // namespace io

#endif  // IO__CANBOARD__CANBOARD_HPP
