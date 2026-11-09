#include "app.hpp"

#include <fmt/core.h>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <list>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <thread>
#include <tuple>
#include <vector>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/diag.hpp"
#include "tasks/auto_aim/multithread/commandgener.hpp"
#include "tasks/auto_aim/multithread/mt_detector.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/target.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/auto_buff/buff_aimer.hpp"
#include "tasks/auto_buff/buff_detector.hpp"
#include "tasks/auto_buff/buff_solver.hpp"
#include "tasks/auto_buff/buff_target.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/yaml.hpp"

using namespace std::chrono_literals;

namespace app
{
namespace
{

bool env_on(const char * name) { return std::getenv(name) != nullptr; }

// 模型加载耗时数百毫秒，且仅 buff 模式需要，故延迟构造
struct BuffPipeline
{
  auto_buff::Buff_Detector detector;
  auto_buff::Solver solver;
  auto_buff::SmallTarget small_target;
  auto_buff::BigTarget big_target;
  auto_buff::Aimer aimer;

  explicit BuffPipeline(const std::string & config_path)
  : detector(config_path), solver(config_path), aimer(config_path)
  {
  }
};

}  // namespace

int run(const std::string & config_path)
{
  const auto yaml = tools::load(config_path);

  const double print_hz = yaml["print_hz"] ? yaml["print_hz"].as<double>() : 1.0;
  const bool use_shooter = yaml["use_shooter"] ? yaml["use_shooter"].as<bool>() : true;
  const std::string detect_thread =
    yaml["detect_thread"] ? yaml["detect_thread"].as<std::string>() : "async";
  const int command_hz = yaml["command_hz"] ? yaml["command_hz"].as<int>() : 500;
  const bool async_detect = detect_thread != "sync";
  const bool cmd_thread = command_hz > 0;

  // 环境变量优先于 yaml；SIM_* 为旧名，保留兼容
  const bool diag_on =
    (yaml["diag"] ? yaml["diag"].as<bool>() : false) || env_on("DIAG") || env_on("SIM_DIAG");
  // imshow 每帧 X11 上传耗时数毫秒，故默认关闭
  const bool show = env_on("SHOW") || env_on("SIM_SHOW");
  const bool time_on = env_on("TIME") || env_on("SIM_TIME");

  // 内参按 image_size 标定，画面尺寸不符会使 PnP 角度整体偏移
  const auto image_size = yaml["image_size"];

  tools::Exiter exiter;
  tools::Plotter plotter;

  io::Camera camera(config_path);
  io::CBoard cboard(config_path);
  // 相机不支持并发 read
  std::mutex camera_mtx;

  std::unique_ptr<auto_aim::multithread::MultiThreadDetector> mt_detector;
  std::unique_ptr<auto_aim::YOLO> detector;
  if (async_detect)
    mt_detector = std::make_unique<auto_aim::multithread::MultiThreadDetector>(config_path);
  else
    detector = std::make_unique<auto_aim::YOLO>(config_path, false);

  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);
  auto_aim::Diag diag(diag_on);

  std::unique_ptr<auto_aim::multithread::CommandGener> commandgener;
  if (cmd_thread)
    commandgener = std::make_unique<auto_aim::multithread::CommandGener>(
      shooter, aimer, cboard, plotter, false, use_shooter);

  std::unique_ptr<BuffPipeline> buff;

  std::atomic<io::Mode> mode{io::Mode::idle};
  std::thread detect_thread_handle;
  if (async_detect) {
    detect_thread_handle = std::thread([&] {
      cv::Mat img;
      std::chrono::steady_clock::time_point t;
      while (!exiter.exit()) {
        if (mode.load() != io::Mode::auto_aim) {
          std::this_thread::sleep_for(1ms);
          continue;
        }
        {
          std::lock_guard<std::mutex> lock(camera_mtx);
          camera.read(img, t);
        }
        mt_detector->push(img, t);
      }
    });
  }

  auto last_mode = io::Mode::idle;
  const auto t_begin = std::chrono::steady_clock::now();
  auto next_print = t_begin;
  int frames = 0, frames_since_print = 0;
  bool size_checked = false;

  double ms_det = 0, ms_trk = 0, ms_aim = 0, ms_gui = 0, ms_lag = 0;

  cv::Mat img;
  std::list<auto_aim::Armor> armors;
  std::list<auto_aim::Target> targets;
  std::chrono::steady_clock::time_point t = std::chrono::steady_clock::now();
  io::Command command{false, false, 0, 0};
  Eigen::Vector3d ypr = Eigen::Vector3d::Zero();

  while (!exiter.exit()) {
    mode = cboard.mode;

    if (last_mode != mode) {
      tools::logger()->info(
        "switch to {} (detect {}, command {})", io::MODES[mode],
        async_detect ? "thread" : "inline",
        cmd_thread ? std::to_string(command_hz) + "Hz" : "per-frame");
      last_mode = mode;
    }

    if (mode.load() == io::Mode::auto_aim) {
      armors.clear();
      targets.clear();

      const auto tick0 = std::chrono::steady_clock::now();
      if (async_detect) {
        std::tie(img, armors, t) = mt_detector->debug_pop();
      } else {
        {
          std::lock_guard<std::mutex> lock(camera_mtx);
          camera.read(img, t);
        }
        armors = detector->detect(img);
      }

      if (!size_checked && !img.empty()) {
        size_checked = true;
        if (image_size && image_size.size() == 2) {
          const int w = image_size[0].as<int>();
          const int h = image_size[1].as<int>();
          if (w != img.cols || h != img.rows)
            tools::logger()->warn(
              "frame {}x{} differs from image_size {}x{} in yaml; intrinsics assume {}x{} "
              "so all aim angles are offset (match the window size or recalibrate)",
              img.cols, img.rows, w, h, w, h);
        }
      }

      // 采集到当前的间隔：异步含排队与推理延迟，同步为 0
      const double frame_age = tools::delta_time(std::chrono::steady_clock::now(), t);

      const Eigen::Quaterniond q = cboard.imu_at(t - 1ms);
      solver.set_R_gimbal2world(q);
      ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

      const auto tick1 = std::chrono::steady_clock::now();
      targets = tracker.track(armors, t);
      const auto tick2 = std::chrono::steady_clock::now();

      auto_aim::multithread::AimResult result;
      if (cmd_thread) {
        commandgener->push(targets, t, cboard.bullet_speed, ypr);
        // diag 需要本帧指令，故等待下发；否则只读快照
        result = diag.on() ? commandgener->wait_result(t, 0.03) : commandgener->last_result();
        if (result.has_value) command = result.command;
      } else {
        command = aimer.aim(targets, t, cboard.bullet_speed);
        if (use_shooter) command.shoot = shooter.shoot(command, aimer, targets, ypr);
        cboard.send(command);
        result.command = command;
        result.aim_point = aimer.debug_aim_point;
        result.dt = aimer.debug_dt;
        result.fly_time = aimer.debug_fly_time;
        result.input_t = t;
        result.has_value = true;
      }
      const auto tick3 = std::chrono::steady_clock::now();

      if (time_on) {
        ms_det += 1e3 * tools::delta_time(tick1, tick0);
        ms_trk += 1e3 * tools::delta_time(tick2, tick1);
        ms_aim += 1e3 * tools::delta_time(tick3, tick2);
        ms_lag += 1e3 * frame_age;
      }

      if (diag.on())
        diag.log(
          {frames, tools::delta_time(t, std::chrono::steady_clock::time_point{}), armors, targets,
           ypr, result.command, result.aim_point, result.dt, result.fly_time});

      const auto tick4 = std::chrono::steady_clock::now();
      if (show) {
        tools::draw_text(img, fmt::format("[{}]", tracker.state()), {10, 30}, {255, 255, 255});

        for (const auto & armor : armors) tools::draw_points(img, armor.points, {0, 255, 0});

        if (!targets.empty()) {
          auto target = targets.front();

          for (const Eigen::Vector4d & xyza : target.armor_xyza_list()) {
            auto image_points =
              solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
            tools::draw_points(img, image_points, {0, 255, 0});
          }

          // debug_* 由指令线程写，此处只读快照
          const Eigen::Vector4d aim_xyza = result.aim_point.xyza;
          auto image_points = solver.reproject_armor(
            aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
          if (result.aim_point.valid)
            tools::draw_points(img, image_points, {0, 0, 255});
          else
            tools::draw_points(img, image_points, {255, 0, 0});
        }

        cv::resize(img, img, {}, 0.5, 0.5);
        cv::imshow("app", img);
        if (cv::waitKey(1) == 'q') break;
      }
      if (time_on) ms_gui += 1e3 * tools::delta_time(std::chrono::steady_clock::now(), tick4);

      frames++;
      frames_since_print++;

      const auto now = std::chrono::steady_clock::now();
      if (print_hz > 0 && now >= next_print) {
        next_print = now + std::chrono::microseconds(static_cast<int64_t>(1e6 / print_hz));
        const double n = std::max(1, frames_since_print);
        if (time_on) {
          tools::logger()->info(
            "[time] ms/frame: capture+detect {:.1f} track {:.1f} aim {:.1f} draw {:.1f} | "
            "sum {:.1f} | frame age {:.1f}",
            ms_det / n, ms_trk / n, ms_aim / n, ms_gui / n,
            (ms_det + ms_trk + ms_aim + ms_gui) / n, ms_lag / n);
          ms_det = ms_trk = ms_aim = ms_gui = ms_lag = 0;
          frames_since_print = 0;
        }
        tools::logger()->info(
          "#{:6d} {:.0f}fps det {} tgt {} | mode {} v {:.1f} | cmd control={} shoot={} "
          "yaw={:+.2f}° pitch={:+.2f}° | gim yaw={:+.2f}° pitch={:+.2f}°",
          frames, frames / tools::delta_time(now, t_begin), armors.size(), targets.size(),
          io::MODES[cboard.mode], cboard.bullet_speed, command.control, command.shoot,
          command.yaw * 57.3, command.pitch * 57.3, ypr[0] * 57.3, ypr[1] * 57.3);
      }
    }

    else if (mode.load() == io::Mode::small_buff || mode.load() == io::Mode::big_buff) {
      armors.clear();
      targets.clear();

      if (!buff) buff = std::make_unique<BuffPipeline>(config_path);

      {
        std::lock_guard<std::mutex> lock(camera_mtx);
        camera.read(img, t);
      }
      const Eigen::Quaterniond q = cboard.imu_at(t - 1ms);
      buff->solver.set_R_gimbal2world(q);

      auto power_runes = buff->detector.detect(img);
      buff->solver.solve(power_runes);

      io::Command buff_command{false, false, 0, 0};
      if (mode.load() == io::Mode::small_buff) {
        buff->small_target.get_target(power_runes, t);
        auto target_copy = buff->small_target;
        buff_command = buff->aimer.aim(target_copy, t, cboard.bullet_speed, true);
      } else {
        buff->big_target.get_target(power_runes, t);
        auto target_copy = buff->big_target;
        buff_command = buff->aimer.aim(target_copy, t, cboard.bullet_speed, true);
      }
      cboard.send(buff_command);
    }

    else
      std::this_thread::sleep_for(2ms);
  }

  if (detect_thread_handle.joinable()) detect_thread_handle.join();
  // 退出前停控，否则云台会保持最后一条指令
  cboard.send({false, false, 0, 0});
  return 0;
}

}  // namespace app
