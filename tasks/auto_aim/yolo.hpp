#ifndef AUTO_AIM__YOLO_HPP
#define AUTO_AIM__YOLO_HPP

#include <list>
#include <memory>
#include <opencv2/opencv.hpp>

#include "armor.hpp"
#include "detector.hpp"
#include "backend/detector.hpp"

namespace auto_aim
{
class YOLO
{
public:
  YOLO(const std::string & config_path, bool debug = true);
  std::list<Armor> detect(const cv::Mat & img, int frame_count = -1);
  backend::Detector::PendingFrame submit(const cv::Mat & img, int frame_count = -1);
  std::list<Armor> collect(backend::Detector::PendingFrame & pending, const cv::Mat & img);

private:
  std::unique_ptr<backend::Detector> backend_;
  Detector refiner_;
  bool debug_ = false, use_roi_ = false, use_traditional_ = false;
  double min_confidence_ = 0;
  cv::Rect roi_;
  cv::Point2f offset_;
  std::uint64_t frame_count_ = 0;

  cv::Mat crop(const cv::Mat & img);
  std::list<Armor> convert(
    const backend::types::DetectionResults & results, const cv::Mat & img, std::uint64_t frame_count);
};
}  // namespace auto_aim

#endif  // AUTO_AIM__YOLO_HPP
