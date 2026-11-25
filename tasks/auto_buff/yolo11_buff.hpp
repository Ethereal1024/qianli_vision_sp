#ifndef AUTO_BUFF__YOLO11_BUFF_HPP
#define AUTO_BUFF__YOLO11_BUFF_HPP
#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <opencv2/opencv.hpp>

#include "backend/detector.hpp"
#include "tools/logger.hpp"

namespace auto_buff
{
const std::vector<std::string> class_names = {"unactivated", "small_activated", "large_activated"};

class YOLO11_BUFF
{
public:
  struct Object
  {
    cv::Rect_<float> rect;
    int label;
    float prob;
    std::vector<cv::Point2f> kpt;
  };

  YOLO11_BUFF(const std::string & config);

  // 使用NMS，用来获取多个框
  std::vector<Object> get_multicandidateboxes(cv::Mat & image);

  // 寻找置信度最高的框
  std::vector<Object> get_onecandidatebox(cv::Mat & image);

private:
  std::unique_ptr<backend::Detector> detector_;
  std::uint64_t frame_count_ = 0;
  const int NUM_POINTS = 6;

  std::vector<Object> detect(cv::Mat & image, bool one_candidate);

  // 将image保存为"../result/$${programName}.jpg"
  void save(const std::string & programName, const cv::Mat & image);
};
}  // namespace auto_buff
#endif
