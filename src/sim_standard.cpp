#include <fmt/core.h>

#include <chrono>
#include <opencv2/opencv.hpp>
#include <string>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |                  | print this message}"
  "{@config-path   | configs/sim.yaml | path of the yaml config }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  const auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;

  const auto yaml = tools::load(config_path);
  const double print_hz = yaml["print_hz"] ? yaml["print_hz"].as<double>() : 1.0;
  const bool use_shooter = yaml["use_shooter"] ? yaml["use_shooter"].as<bool>() : true;

  const auto image_size = yaml["image_size"];

  io::Camera camera(config_path);
  io::CBoard cboard(config_path);

  auto_aim::YOLO detector(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);

  cv::Mat img;
  std::chrono::steady_clock::time_point t;

  auto last_mode = io::Mode::idle;
  const auto t_begin = std::chrono::steady_clock::now();
  auto next_print = t_begin;
  int frames = 0;
  bool size_checked = false;

  while (!exiter.exit()) {
    camera.read(img, t);

    if (!size_checked) {
      size_checked = true;
      if (image_size && image_size.size() == 2) {
        const int w = image_size[0].as<int>();
        const int h = image_size[1].as<int>();
        if (w != img.cols || h != img.rows)
          tools::logger()->warn(
            "[sim] frame {}x{} differs from image_size {}x{} in sim.yaml, intrinsics assume {}x{}",
            img.cols, img.rows, w, h, w, h);
      }
    }

    const Eigen::Quaterniond q = cboard.imu_at(t - 1ms);
    if (last_mode != cboard.mode) {
      tools::logger()->info("Switch to {}", io::MODES[cboard.mode]);
      last_mode = cboard.mode;
    }

    solver.set_R_gimbal2world(q);
    const Eigen::Vector3d ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

    auto armors = detector.detect(img);
    auto targets = tracker.track(armors, t);
    auto command = aimer.aim(targets, t, cboard.bullet_speed);
    if (use_shooter) command.shoot = shooter.shoot(command, aimer, targets, ypr);
    cboard.send(command);
    frames++;

    tools::draw_text(img, fmt::format("[{}]", tracker.state()), {10, 30}, {255, 255, 255});

    for (const auto & armor : armors) tools::draw_points(img, armor.points, {0, 255, 0});

    if (!targets.empty()) {
      auto target = targets.front();

      for (const Eigen::Vector4d & xyza : target.armor_xyza_list()) {
        auto image_points =
          solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
        tools::draw_points(img, image_points, {0, 255, 0});
      }

      auto aim_point = aimer.debug_aim_point;
      Eigen::Vector4d aim_xyza = aim_point.xyza;
      auto image_points =
        solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
      if (aim_point.valid)
        tools::draw_points(img, image_points, {0, 0, 255});
      else
        tools::draw_points(img, image_points, {255, 0, 0});
    }

    cv::resize(img, img, {}, 0.5, 0.5);
    cv::imshow("sim_standard", img);
    if (cv::waitKey(1) == 'q') break;

    const auto now = std::chrono::steady_clock::now();
    if (print_hz > 0 && now >= next_print) {
      next_print = now + std::chrono::microseconds(static_cast<int64_t>(1e6 / print_hz));
      tools::logger()->info(
        "[sim] #{:6d} {:.0f}fps det {} tgt {} | mode {} v {:.1f} | cmd control={} shoot={} "
        "yaw={:+.2f} pitch={:+.2f} deg | imu yaw={:+.2f} pitch={:+.2f} deg",
        frames, frames / tools::delta_time(now, t_begin), armors.size(), targets.size(),
        io::MODES[cboard.mode], cboard.bullet_speed, command.control, command.shoot,
        command.yaw * 57.3, command.pitch * 57.3, ypr[0] * 57.3, ypr[1] * 57.3);
    }
  }

  cboard.send({false, false, 0, 0});
  return 0;
}
