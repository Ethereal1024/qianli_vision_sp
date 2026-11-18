#pragma once

#include <chrono>
#include <list>
#include <optional>
#include <utility>

#include "io/camera.hpp"
#include "tasks/auto_aim/yolo.hpp"

class SyncArmorFrameReader
{
public:
  SyncArmorFrameReader(io::Camera & camera, auto_aim::YOLO & detector)
  : camera_(camera), detector_(detector)
  {
  }

  void read(cv::Mat & image, std::chrono::steady_clock::time_point & timestamp)
  {
    camera_.read(image, timestamp);
  }

  std::list<auto_aim::Armor> detect(const cv::Mat & image) { return detector_.detect(image); }

private:
  io::Camera & camera_;
  auto_aim::YOLO & detector_;
};

class AsyncArmorFrameReader
{
public:
  AsyncArmorFrameReader(io::Camera & camera, auto_aim::YOLO & detector)
  : camera_(camera), detector_(detector)
  {
  }

  void read(cv::Mat & image, std::chrono::steady_clock::time_point & timestamp)
  {
    if (!next_) next_ = submit_frame();
    image = std::move(next_->image);
    timestamp = next_->timestamp;
    pending_ = std::move(next_->pending);
    next_.reset();
  }

  std::list<auto_aim::Armor> detect(const cv::Mat & image)
  {
    // 当前帧推理期间读取并提交下一帧，结果仍按采集顺序处理。
    next_ = submit_frame();
    return detector_.collect(pending_, image);
  }

private:
  struct Frame
  {
    cv::Mat image;
    std::chrono::steady_clock::time_point timestamp;
    backend::Detector::PendingFrame pending;
  };

  Frame submit_frame()
  {
    Frame frame;
    camera_.read(frame.image, frame.timestamp);
    frame.pending = detector_.submit(frame.image);
    return frame;
  }

  io::Camera & camera_;
  auto_aim::YOLO & detector_;
  std::optional<Frame> next_;
  backend::Detector::PendingFrame pending_;
};

#ifdef QIANLI_ASYNC_INFERENCE
using ArmorFrameReader = AsyncArmorFrameReader;
#else
using ArmorFrameReader = SyncArmorFrameReader;
#endif
