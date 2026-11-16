#include "yolo11_buff.hpp"

#include <algorithm>
#include <cmath>
#include "tools/yaml.hpp"

namespace auto_buff
{
YOLO11_BUFF::YOLO11_BUFF(const std::string & config)
{
  const auto yaml = tools::load(config);
  detector_ = std::make_unique<backend::Detector>(
    yaml["rune_pipeline"] ? yaml["rune_pipeline"] : yaml["pipeline"]);
}

std::vector<YOLO11_BUFF::Object> YOLO11_BUFF::get_multicandidateboxes(cv::Mat & image)
{
  return detect(image, false);
}

std::vector<YOLO11_BUFF::Object> YOLO11_BUFF::get_onecandidatebox(cv::Mat & image)
{
  return detect(image, true);
}

std::vector<YOLO11_BUFF::Object> YOLO11_BUFF::detect(cv::Mat & image, bool one_candidate)
{
  const auto start = cv::getTickCount();
  if (image.empty()) return {};
  const auto decoded = detector_->detect(image, frame_count_++);
  const auto detections = std::get_if<std::vector<backend::types::RuneResult>>(&decoded);
  if (!detections) return {};

  std::vector<Object> results;
  for (const auto & result : *detections) {
    if (one_candidate && result.class_id != backend::types::RuneType::UNACTIVATED) continue;
    Object object;
    object.label = static_cast<int>(result.class_id);
    object.prob = result.confidence;
    // 保持解算用的 top、left、bottom、right、扇叶中心、R中心顺序。
    object.kpt = {cv::Point2f(result.top), cv::Point2f(result.left),
                  cv::Point2f(result.bottom), cv::Point2f(result.right)};
    cv::Point2f center{};
    for (const auto & point : object.kpt) center += point * 0.25f;
    object.rect = cv::boundingRect(object.kpt);
    object.kpt.push_back(center);
    object.kpt.emplace_back(result.point_R);
    results.push_back(std::move(object));
  }
  if (one_candidate && !results.empty()) {
    const auto best = std::max_element(results.begin(), results.end(),
      [](const auto & a, const auto & b) { return a.prob < b.prob; });
    Object object = std::move(*best);
    results.clear();
    results.push_back(std::move(object));
  }
  for (const auto & object : results) {
    cv::rectangle(image, object.rect, cv::Scalar(255, 255, 255), 1);
    cv::putText(image, class_names[object.label] + cv::format(":%.2f", object.prob),
                object.rect.tl(), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255));
    for (int point = 0; point < NUM_POINTS; ++point) {
      cv::circle(image, object.kpt[point], 2, cv::Scalar(255, 255, 0), -1, cv::LINE_AA);
    }
  }
  const auto seconds = (cv::getTickCount() - start) / cv::getTickFrequency();
  cv::putText(image, cv::format("FPS: %.2f", 1.0 / seconds), {20, 40},
              cv::FONT_HERSHEY_PLAIN, 2.0, cv::Scalar(255, 0, 0), 2);
  return results;
}

void YOLO11_BUFF::save(const std::string & programName, const cv::Mat & image)
{
  const std::filesystem::path directory = "../result/";
  std::filesystem::create_directories(directory);
  cv::imwrite((directory / (programName + ".jpg")).string(), image);
}
}  // namespace auto_buff
