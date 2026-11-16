#include <fmt/core.h>
#include <optional>

#include <filesystem>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <variant>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "io/gimbal.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

const std::string keys =
  "{help h usage ?  |                          | 输出命令行参数说明}"
  "{@config-path c  | configs/calibration.yaml | yaml配置文件路径 }"
  "{output-folder o |      assets/img_with_q   | 输出文件夹路径   }";

using Communication = std::variant<io::CBoard, io::Gimbal>;

void write_q(const std::string q_path, const Eigen::Quaterniond & q)
{
  std::ofstream q_file(q_path);
  Eigen::Vector4d xyzw = q.coeffs();
  // 输出顺序为wxyz
  q_file << fmt::format("{} {} {} {}", xyzw[3], xyzw[0], xyzw[1], xyzw[2]);
  q_file.close();
}

void capture_loop(
  const std::string & config_path, std::optional<Communication>& communication, const std::string & output_folder, const cv::Size& pattern_size)
{
  if (!communication.has_value()) tools::logger() -> error("[capture] Communication is empty!");
  io::Camera camera(config_path);
  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;

  int count = 0;
  while (true) {
    camera.read(img, timestamp);
    Eigen::Quaterniond q = std::visit(
      [&](auto & device) { return device.imu_at(timestamp); }, communication.value());

    // 在图像上显示欧拉角，用来判断imuabs系的xyz正方向，同时判断imu是否存在零漂
    auto img_with_ypr = img.clone();
    Eigen::Vector3d zyx = tools::eulers(q, 2, 1, 0) * 57.3;  // degree
    tools::draw_text(img_with_ypr, fmt::format("Z {:.2f}", zyx[0]), {40, 40}, {0, 0, 255});
    tools::draw_text(img_with_ypr, fmt::format("Y {:.2f}", zyx[1]), {40, 80}, {0, 0, 255});
    tools::draw_text(img_with_ypr, fmt::format("X {:.2f}", zyx[2]), {40, 120}, {0, 0, 255});

    // 使用缩小分辨率的灰度图进行棋盘格检测，加速“未识别到棋盘格”时的处理
    const double scale = 0.5;
    cv::Mat gray, gray_small;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    cv::resize(gray, gray_small, {}, scale, scale);

    std::vector<cv::Point2f> centers_2d;
    std::vector<cv::Point2f> centers_2d_small;
    auto success = cv::findChessboardCorners(
      gray_small, pattern_size, centers_2d_small,
      cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE +
        cv::CALIB_CB_FAST_CHECK);  // 棋盘格角点
    if (success) {
      centers_2d.reserve(centers_2d_small.size());
      for (const auto & p : centers_2d_small) {
        centers_2d.emplace_back(p.x / scale, p.y / scale);
      }
      cv::cornerSubPix(
        gray, centers_2d, cv::Size(11, 11), cv::Size(-1, -1),
        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
    }
    cv::drawChessboardCorners(img_with_ypr, pattern_size, centers_2d, success);  // 显示识别结果
    cv::resize(img_with_ypr, img_with_ypr, {}, 0.5, 0.5);  // 显示时缩小图片尺寸

    // 按“s”保存图片和对应四元数，按“q”退出程序
    cv::imshow("Press s to save, q to quit", img_with_ypr);
    auto key = cv::waitKey(1);
    if (key == 'q')
      break;
    else if (key != 's')
      continue;

    // 保存图片和四元数
    count++;
    auto img_path = fmt::format("{}/{}.jpg", output_folder, count);
    auto q_path = fmt::format("{}/{}.txt", output_folder, count);
    cv::imwrite(img_path, img);
    write_q(q_path, q);
    tools::logger()->info("[{}] Saved in {}", count, output_folder);
  }

  // 离开该作用域时，camera和cboard会自动关闭
}

int main(int argc, char * argv[])
{
  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>(0);
  auto output_folder = cli.get<std::string>("output-folder");

  // 新建输出文件夹
  std::filesystem::create_directory(output_folder);

  tools::logger()->info("[capture]默认标定板尺寸为10列7行");
  // 主循环，保存图片和对应四元数

  // 按配置原地构造通信对象
  auto config = tools::load(config_path);
  auto mode = tools::read<std::string>(config, "mode", "gimbal");
  std::optional<Communication> communication;
  if (mode == "gimbal") {
    communication.emplace(std::in_place_type<io::Gimbal>, config_path);
    tools::logger()->info("[capture] 使用云台(gimbal)串口读取IMU/姿态数据");
  }

  else if (mode == "cboard") {
    communication.emplace(std::in_place_type<io::CBoard>, config_path);
    tools::logger()->info("[capture] 使用CAN总线(CBoard)读取IMU数据");
  }

  else {
    tools::logger() -> error("[capture] mode设置错误(不为gimbal或cboard)或没有设置!");
    return 0;
  }

  auto pattern_cols = tools::read(config, "pattern_cols", 11); // 行数
  auto pattern_rows = tools::read(config, "pattern_rows", 8);  // 列数
  auto pattern_size = cv::Size(pattern_rows, pattern_cols);

  capture_loop(config_path, communication, output_folder, pattern_size);
  tools::logger()->warn("[capture]注意四元数输出顺序为wxyz");

  return 0;
}
