#include "yolo.hpp"

#include "tools/img_tools.hpp"
#include "tools/yaml.hpp"

namespace auto_aim
{
YOLO::YOLO(const std::string & config_path, bool debug)
: refiner_(config_path, false), debug_(debug)
{
  const auto yaml = tools::load(config_path);
  backend_ = std::make_unique<backend::Detector>(yaml["pipeline"]);
  use_roi_ = tools::read(yaml, "use_roi", false);
  use_traditional_ = tools::read(yaml, "use_traditional", false);
  min_confidence_ = tools::read(yaml, "min_confidence", 0.0);
  const auto roi = yaml["roi"];
  roi_ = {tools::read(roi, "x", 0), tools::read(roi, "y", 0),
          tools::read(roi, "width", -1), tools::read(roi, "height", -1)};
  offset_ = use_roi_ ? cv::Point2f(roi_.x, roi_.y) : cv::Point2f();
}

cv::Mat YOLO::crop(const cv::Mat & img)
{
  if (img.empty() || !use_roi_) return img;
  const cv::Rect region(roi_.x, roi_.y,
    roi_.width == -1 ? img.cols - roi_.x : roi_.width,
    roi_.height == -1 ? img.rows - roi_.y : roi_.height);
  return img(region);
}

std::list<Armor> YOLO::detect(const cv::Mat & img, int frame_count)
{
  if (img.empty()) return {};
  const auto id = frame_count < 0 ? frame_count_++ : static_cast<std::uint64_t>(frame_count);
  return convert(backend_->detect(crop(img), id), img, id);
}

backend::Detector::PendingFrame YOLO::submit(const cv::Mat & img, int frame_count)
{
  const auto id = frame_count < 0 ? frame_count_++ : static_cast<std::uint64_t>(frame_count);
  return backend_->submit(crop(img), id);
}

std::list<Armor> YOLO::collect(backend::Detector::PendingFrame & pending, const cv::Mat & img)
{
  return convert(backend_->collect(pending), img, pending.frame_count);
}

std::list<Armor> YOLO::convert(
  const backend::types::DetectionResults & results, const cv::Mat & img, std::uint64_t frame_count)
{
  std::list<Armor> armors;
  const auto detections = std::get_if<std::vector<backend::types::YOLOResult>>(&results);
  if (!detections || img.empty()) return armors;
  for (const auto & result : *detections) {
    if (result.points_2d_.size() != 4 || result.confidence_ <= min_confidence_) continue;
    // 两个新装甲板模型都输出 LT、LB、RB、RT，解算接口需要 LT、RT、RB、LB。
    std::vector<cv::Point2f> points;
    for (const auto index : {0, 3, 2, 1}) {
      points.push_back(cv::Point2f(result.points_2d_[index]) + offset_);
    }
    armors.emplace_back(
      static_cast<int>(result.color_), static_cast<int>(result.type_), result.confidence_,
      cv::boundingRect(points), points);
    auto & armor = armors.back();
    armor.class_id = static_cast<int>(result.type_);
    armor.duplicated = false;
    if (use_traditional_) refiner_.detect(armor, img);
    armor.center_norm = {armor.center.x / img.cols, armor.center.y / img.rows};
  }
  if (debug_) {
    auto drawing = img.clone();
    tools::draw_text(drawing, fmt::format("[{}]", frame_count), {10, 30}, {255, 255, 255});
    for (const auto & armor : armors) {
      tools::draw_points(drawing, armor.points, {0, 255, 0});
      tools::draw_text(drawing,
        fmt::format("{:.2f} {} {} {}", armor.confidence, COLORS[armor.color],
                    ARMOR_NAMES[armor.name], ARMOR_TYPES[armor.type]), armor.center, {0, 255, 0});
    }
    cv::resize(drawing, drawing, {}, 0.5, 0.5);
    cv::imshow("detection", drawing);
  }
  return armors;
}
}  // namespace auto_aim
