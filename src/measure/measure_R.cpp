#include <Eigen/Dense>
#include <chrono>
#include <cstddef>
#include <opencv2/opencv.hpp>
#include <optional>
#include <string>

#include "io/camera.hpp"
#include "io/communication.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明 }"
  "{@config-path   | configs/measure_R.yaml | yaml 的配置路径   }"
  "{measure-times  |                   2000 | 有效观测的采样次数 }"
  "{condition      |                 未标记 | 工况标签，可注明距离、视角和光照 }"
  "{display        |                   true | 是否可视化        }";

namespace
{
std::optional<Eigen::Vector4d> sample_ypda(
  const cv::Mat & img, auto_aim::YOLO & detector, const auto_aim::Solver & solver)
{
  auto armors = detector.detect(img);
  if (armors.size() != 1) return std::nullopt;

  auto & armor = armors.front();
  if (!solver.try_solve(armor)) return std::nullopt;
  const auto & ypd = armor.ypd_in_world;
  const Eigen::Vector4d ypda{ypd[0], ypd[1], ypd[2], armor.ypr_in_world[0]};
  if (!ypda.allFinite() || ypda[2] <= 0.0) return std::nullopt;
  return ypda;
}
}  // namespace

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  const auto config_path = cli.get<std::string>(0);
  const auto measure_times = cli.get<int>("measure-times");
  const auto condition = cli.get<std::string>("condition");
  const auto display = cli.get<bool>("display");
  if (!cli.check()) {
    cli.printErrors();
    return 1;
  }
  if (measure_times < 2) {
    tools::logger()->error("样本方差至少需要两次有效观测");
    return 1;
  }

  tools::Exiter exiter;
  io::Communication communication(config_path);
  io::Camera camera(config_path);
  auto_aim::YOLO detector(config_path, display);
  auto_aim::Solver solver(config_path);

  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;
  int sample_count = 0;
  std::size_t frame_count = 0;
  Eigen::Vector4d reference = Eigen::Vector4d::Zero();
  Eigen::Vector4d mean = Eigen::Vector4d::Zero();
  Eigen::Matrix4d centered_products = Eigen::Matrix4d::Zero();

  tools::logger()->info(
    "工况 [{}]，配置 {}。请固定相机和装甲板，保证画面内只有一块装甲板。目标样本数 {}",
    condition, config_path, measure_times);
  while (!exiter.exit() && sample_count < measure_times) {
    camera.read(img, timestamp);
    if (img.empty()) {
      if (display && cv::waitKey(1) == 'q') break;
      continue;
    }
    ++frame_count;

    solver.set_R_gimbal2world(communication.imu_at(timestamp - 1ms));
    const auto ypda = sample_ypda(img, detector, solver);
    if (ypda) {
      if (sample_count == 0) reference = *ypda;
      Eigen::Vector4d offset = *ypda - reference;
      // 以首帧为参考展开三个角度，避免跨越 ±π 时放大方差。
      for (const int index : {0, 1, 3}) offset[index] = tools::limit_rad(offset[index]);

      // Welford 在线协方差，用外积累计各分量之间的协变。
      ++sample_count;
      const Eigen::Vector4d delta = offset - mean;
      mean += delta / sample_count;
      centered_products.noalias() += delta * (offset - mean).transpose();
    }
    if (display && cv::waitKey(1) == 'q') break;
  }

  const double acceptance = frame_count ? 100.0 * sample_count / frame_count : 0.0;
  tools::logger()->info(
    "工况 [{}] 有效样本 {}/{}，非空图像 {} 帧，接受率 {:.2f}%",
    condition, sample_count, measure_times, frame_count, acceptance);
  if (sample_count < measure_times) {
    tools::logger()->error("采样未完成，未达到目标有效样本数");
    return 1;
  }

  const Eigen::Matrix4d covariance =
    (centered_products + centered_products.transpose()) / (2.0 * (sample_count - 1));
  const Eigen::Vector4d variance = covariance.diagonal();
  tools::logger()->info(
    "工况 [{}] 的样本协方差 R，行列顺序均为 [视线 yaw, 视线 pitch, 距离, 装甲板 yaw]，"
    "各维单位为 [rad, rad, m, rad]",
    condition);
  for (int row = 0; row < covariance.rows(); ++row) {
    tools::logger()->info(
      "[{:.9g}, {:.9g}, {:.9g}, {:.9g}]",
      covariance(row, 0), covariance(row, 1), covariance(row, 2), covariance(row, 3));
  }
  tools::logger()->info(
    "ypda 方差 [{:.9g}, {:.9g}, {:.9g}, {:.9g}]，单位依次为 rad²、rad²、m²、rad²",
    variance[0], variance[1], variance[2], variance[3]);
  return 0;
}
