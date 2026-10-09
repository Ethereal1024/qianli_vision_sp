#include "mt_detector.hpp"

#include <yaml-cpp/yaml.h>

namespace auto_aim
{
namespace multithread
{

MultiThreadDetector::MultiThreadDetector(const std::string & config_path, bool debug)
: yolo_(config_path, debug), backend_(config_path)
{
  auto yaml = YAML::LoadFile(config_path);
  auto yolo_name = yaml["yolo_name"].as<std::string>();
  auto model_path = yaml[yolo_name + "_model_path"].as<std::string>();

  BackendConfig config;
  config.input_size = cv::Size(640, 640);
  config.throughput_priority = true;

  if (!backend_.init(model_path, config)) {
    throw std::runtime_error("Backend initializing failed");
  }
}

void MultiThreadDetector::push(cv::Mat img, std::chrono::steady_clock::time_point t)
{
  auto ctx = backend_.create_ctx();
  backend_.execute_async(img, ctx.get());
  queue_.push({img.clone(), t, std::move(ctx)});
}

std::tuple<std::list<Armor>, std::chrono::steady_clock::time_point> MultiThreadDetector::pop()
{
  auto [img, t, ctx] = queue_.pop();

  cv::Mat output;
  backend_.wait_for_result(output, ctx.get());
  auto armors = yolo_.postprocess(ctx->scale, output, img, 0);

  return {std::move(armors), t};
}

std::tuple<cv::Mat, std::list<Armor>, std::chrono::steady_clock::time_point>
MultiThreadDetector::debug_pop()
{
  auto [img, t, ctx] = queue_.pop();

  cv::Mat output;
  backend_.wait_for_result(output, ctx.get());
  auto armors = yolo_.postprocess(ctx->scale, output, img, 0);

  return {img, std::move(armors), t};
}

}  // namespace multithread

}  // namespace auto_aim
