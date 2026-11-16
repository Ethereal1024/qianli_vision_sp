#ifndef CALIBRATION__COMMON_HPP
#define CALIBRATION__COMMON_HPP

#include <Eigen/Geometry>
#include <cmath>
#include <fstream>
#include <opencv2/calib3d.hpp>
#include <optional>
#include <string>
#include <vector>

namespace calibration
{
inline std::optional<Eigen::Quaterniond> read_q(const std::string & path)
{
  std::ifstream file(path);
  double w, x, y, z;
  if (!(file >> w >> x >> y >> z)) return std::nullopt;

  Eigen::Quaterniond q(w, x, y, z);
  const auto norm = q.norm();
  if (!q.coeffs().allFinite() || !std::isfinite(norm) || norm <= 1e-12) return std::nullopt;
  return q.normalized();
}

inline bool has_observable_rotations(const std::vector<cv::Mat> & rotations)
{
  if (rotations.size() < 3) return false;

  // 单轴旋转时，所有 (R_i - I) 在同一个方向上均为零，无法唯一确定外参。
  cv::Mat normal = cv::Mat::zeros(3, 3, CV_64F);
  const cv::Mat identity = cv::Mat::eye(3, 3, CV_64F);
  for (const auto & rotation : rotations) {
    cv::Mat difference = rotations.front().t() * rotation - identity;
    normal += difference.t() * difference;
  }
  cv::Mat singular_values;
  cv::SVD::compute(normal, singular_values);
  const auto largest = singular_values.at<double>(0);
  const auto smallest = singular_values.at<double>(2);
  // 拒绝静止或接近单轴的运动，避免把数值噪声当成第二个有效转轴。
  return largest > 1e-8 && smallest > largest * 1e-4;
}
}  // namespace calibration

#endif  // CALIBRATION__COMMON_HPP
