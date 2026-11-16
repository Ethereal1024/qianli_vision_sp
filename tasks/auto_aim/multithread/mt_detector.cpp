#include "mt_detector.hpp"

namespace auto_aim
{
namespace multithread
{

MultiThreadDetector::MultiThreadDetector(const std::string & config_path, bool debug)
: yolo_(config_path, debug)
{
}

void MultiThreadDetector::push(cv::Mat img, std::chrono::steady_clock::time_point t)
{
  auto pending = yolo_.submit(img);
  queue_.push({img.clone(), t, std::move(pending)});
}

std::tuple<std::list<Armor>, std::chrono::steady_clock::time_point> MultiThreadDetector::pop()
{
  auto [img, t, pending] = queue_.pop();

  auto armors = yolo_.collect(pending, img);

  return {std::move(armors), t};
}

std::tuple<cv::Mat, std::list<Armor>, std::chrono::steady_clock::time_point>
MultiThreadDetector::debug_pop()
{
  auto [img, t, pending] = queue_.pop();

  auto armors = yolo_.collect(pending, img);

  return {img, std::move(armors), t};
}

}  // namespace multithread

}  // namespace auto_aim
