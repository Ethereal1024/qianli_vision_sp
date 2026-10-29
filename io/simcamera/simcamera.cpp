#include "simcamera.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <stdexcept>
#include <thread>

#include "tools/logger.hpp"

namespace io::sim
{

namespace
{
constexpr uint32_t kFrameFormatRgb24 = 1u;

uint32_t meta_u32(const uint8_t * meta, size_t off)
{
  uint32_t v = 0;
  std::memcpy(&v, meta + off, sizeof(v));
  return v;
}
}  // namespace

SimCamera::SimCamera(const std::string & config_path) : cfg_(SimShmConfig::from_yaml(config_path))
{
  open(true);
}

void SimCamera::open(bool throw_on_timeout)
{
  if (frames_.opened()) return;

  tools::logger()->info("[sim] waiting for frame channel {}", cfg_.camera_frames);
  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::milliseconds(cfg_.wait_timeout_ms);

  while (true) {
    if (frames_.open_reader(cfg_.camera_frames)) {
      sync_geometry();
      return;
    }

    if (throw_on_timeout && std::chrono::steady_clock::now() >= deadline)
      throw std::runtime_error(
        "[sim] frame channel " + cfg_.camera_frames + " not found (Unity side not running?)");

    std::this_thread::sleep_for(std::chrono::milliseconds(std::max(1, cfg_.open_retry_ms)));
  }
}

void SimCamera::sync_geometry()
{
  uint8_t meta[SIMG_META_SIZE] = {0};
  size_t meta_len = sizeof(meta);
  if (!frames_.meta(meta, &meta_len) || meta_len < 12) return;

  const uint32_t w = meta_u32(meta, 0);
  const uint32_t h = meta_u32(meta, 4);
  const uint32_t format = meta_u32(meta, 8);
  if (format != 0 && format != kFrameFormatRgb24) return;
  if (w == 0 || h == 0) return;
  if (static_cast<int>(w) == width_ && static_cast<int>(h) == height_) return;

  width_ = static_cast<int>(w);
  height_ = static_cast<int>(h);
  buffer_.assign(
    std::max<size_t>(frames_.slot_size(), static_cast<size_t>(width_) * height_ * 3), 0);
}

void SimCamera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  while (true) {
    open(false);
    sync_geometry();

    if (width_ <= 0 || height_ <= 0 || buffer_.empty()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(std::max(1, cfg_.open_retry_ms)));
      continue;
    }

    const int n = frames_.read(buffer_.data(), buffer_.size());
    if (n < 0) {
      tools::logger()->warn("[sim] frame channel read error {}, reopening", n);
      frames_.close();
      continue;
    }
    if (n == 0) {
      std::this_thread::sleep_for(std::chrono::microseconds(200));
      continue;
    }

    cv::Mat rgb(height_, width_, CV_8UC3, buffer_.data());
    cv::cvtColor(rgb, img, cv::COLOR_RGB2BGR);
    timestamp = std::chrono::steady_clock::now();
    return;
  }
}

}  // namespace io::sim
