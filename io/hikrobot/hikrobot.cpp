#include "hikrobot.hpp"

#include <libusb-1.0/libusb.h>

#include <unordered_map>
#include <utility>

#include "tools/logger.hpp"

using namespace std::chrono_literals;

namespace io
{
HikRobot::HikRobot(
  double exposure_ms, double gain, const std::string & vid_pid, int width, int height,
  float fps)
: exposure_us_(exposure_ms * 1e3),
  gain_(gain),
  width_(width),
  height_(height),
  fps_(fps),
  daemon_quit_(false),
  handle_(nullptr),
  capturing_(false),
  capture_quit_(false),
  queue_(1),
  vid_(-1),
  pid_(-1)
{
  set_vid_pid(vid_pid);
  if (libusb_init(NULL)) tools::logger()->warn("Unable to init libusb!");

  daemon_thread_ = std::thread{[this] {
    tools::logger()->info("HikRobot's daemon thread started.");

    capture_start();

    while (!daemon_quit_) {
      std::this_thread::sleep_for(100ms);

      if (capturing_) continue;

      capture_stop();
      reset_usb();
      capture_start();
    }

    capture_stop();

    tools::logger()->info("HikRobot's daemon thread stopped.");
  }};
}

HikRobot::~HikRobot()
{
  daemon_quit_ = true;
  if (daemon_thread_.joinable()) daemon_thread_.join();
  tools::logger()->info("HikRobot destructed.");
}

void HikRobot::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  auto data = queue_.pop();
  img = std::move(data.img);
  timestamp = data.timestamp;
}

void HikRobot::capture_start()
{
  if (handle_ != nullptr) return;

  capturing_ = false;
  capture_quit_ = false;

  unsigned int ret;

  MV_CC_DEVICE_INFO_LIST device_list;
  ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_EnumDevices failed: {:#x}", ret);
    return;
  }

  if (device_list.nDeviceNum == 0) {
    tools::logger()->warn("Not found camera!");
    return;
  }

  ret = MV_CC_CreateHandle(&handle_, device_list.pDeviceInfo[0]);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_CreateHandle failed: {:#x}", ret);
    return;
  }

  ret = MV_CC_OpenDevice(handle_);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_OpenDevice failed: {:#x}", ret);
    return;
  }

  set_enum_value("BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_CONTINUOUS);
  set_enum_value("AcquisitionMode", MV_ACQ_MODE_CONTINUOUS);
  set_enum_value("TriggerMode", MV_TRIGGER_MODE_OFF);
  set_enum_value("ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
  set_enum_value("GainAuto", MV_GAIN_MODE_OFF);
  set_float_value("ExposureTime", exposure_us_);
  set_float_value("Gain", gain_);

  // ROI affects the maximum frame rate, so configure it before frame rate.
  if (width_ > 0 && height_ > 0) {
    MVCC_INTVALUE max_w{};
    MVCC_INTVALUE max_h{};
    auto width_ret = MV_CC_GetIntValue(handle_, "WidthMax", &max_w);
    auto height_ret = MV_CC_GetIntValue(handle_, "HeightMax", &max_h);

    if (width_ret != MV_OK || height_ret != MV_OK) {
      tools::logger()->warn(
        "Failed to query camera ROI limits: width={:#x}, height={:#x}", width_ret, height_ret);
    } else if (
      width_ > static_cast<int>(max_w.nCurValue) ||
      height_ > static_cast<int>(max_h.nCurValue)) {
      tools::logger()->warn(
        "Invalid camera ROI {}x{}; sensor maximum is {}x{}", width_, height_,
        max_w.nCurValue, max_h.nCurValue);
    } else {
      int offset_x = (static_cast<int>(max_w.nCurValue) - width_) / 2;
      int offset_y = (static_cast<int>(max_h.nCurValue) - height_) / 2;
      offset_x = offset_x / 2 * 2;
      offset_y = offset_y / 2 * 2;

      // Clear offsets first so reducing width/height cannot violate the
      // current offset + size constraint on GenICam devices.
      unsigned int roi_ret = MV_CC_SetIntValue(handle_, "OffsetX", 0);
      if (roi_ret == MV_OK) roi_ret = MV_CC_SetIntValue(handle_, "OffsetY", 0);
      if (roi_ret == MV_OK) roi_ret = MV_CC_SetIntValue(handle_, "Width", width_);
      if (roi_ret == MV_OK) roi_ret = MV_CC_SetIntValue(handle_, "Height", height_);
      if (roi_ret == MV_OK) roi_ret = MV_CC_SetIntValue(handle_, "OffsetX", offset_x);
      if (roi_ret == MV_OK) roi_ret = MV_CC_SetIntValue(handle_, "OffsetY", offset_y);

      if (roi_ret == MV_OK) {
        tools::logger()->info(
          "Camera ROI: {}x{} offset({}, {})", width_, height_, offset_x, offset_y);
      } else {
        tools::logger()->warn("Failed to set camera ROI: {:#x}", roi_ret);
      }
    }
  }

  ret = MV_CC_SetBoolValue(handle_, "AcquisitionFrameRateEnable", true);
  if (ret != MV_OK) {
    tools::logger()->warn("Failed to enable camera frame-rate control: {:#x}", ret);
  }

  ret = MV_CC_SetFloatValue(handle_, "AcquisitionFrameRate", fps_);
  if (ret != MV_OK) {
    tools::logger()->warn("Failed to set camera frame rate to {}: {:#x}", fps_, ret);
  }

  MVCC_FLOATVALUE actual_fps{};
  ret = MV_CC_GetFloatValue(handle_, "ResultingFrameRate", &actual_fps);
  if (ret != MV_OK) {
    ret = MV_CC_GetFloatValue(handle_, "AcquisitionFrameRate", &actual_fps);
  }
  if (ret == MV_OK) {
    tools::logger()->info(
      "Camera frame rate: requested {:.1f}, resulting {:.1f} fps", fps_,
      actual_fps.fCurValue);
  } else {
    tools::logger()->warn("Failed to read resulting camera frame rate: {:#x}", ret);
  }

  ret = MV_CC_SetImageNodeNum(handle_, 3);
  if (ret != MV_OK) {
    tools::logger()->warn("Failed to set SDK image buffer nodes: {:#x}", ret);
  }

  ret = MV_CC_StartGrabbing(handle_);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_StartGrabbing failed: {:#x}", ret);
    return;
  }

  capture_thread_ = std::thread{[this] {
    tools::logger()->info("HikRobot's capture thread started.");

    capturing_ = true;

    MV_FRAME_OUT raw{};

    while (!capture_quit_) {
      unsigned int ret;
      unsigned int nMsec = 100;

      ret = MV_CC_GetImageBuffer(handle_, &raw, nMsec);
      if (ret != MV_OK) {
        tools::logger()->warn("MV_CC_GetImageBuffer failed: {:#x}", ret);
        break;
      }

      auto timestamp = std::chrono::steady_clock::now();
      const auto & frame_info = raw.stFrameInfo;
      auto pixel_type = frame_info.enPixelType;
      cv::Mat img;

      if (pixel_type == PixelType_Gvsp_BGR8_Packed) {
        // The SDK owns raw.pBufAddr, so the queued image must own a copy.
        img = cv::Mat(
                cv::Size(frame_info.nWidth, frame_info.nHeight), CV_8UC3,
                raw.pBufAddr)
                .clone();
      } else {
        const static std::unordered_map<MvGvspPixelType, cv::ColorConversionCodes> type_map = {
          {PixelType_Gvsp_BayerGR8, cv::COLOR_BayerGR2BGR},
          {PixelType_Gvsp_BayerRG8, cv::COLOR_BayerRG2RGB},
          {PixelType_Gvsp_BayerGB8, cv::COLOR_BayerGB2BGR},
          {PixelType_Gvsp_BayerBG8, cv::COLOR_BayerBG2BGR}};
        auto conversion = type_map.find(pixel_type);
        if (conversion != type_map.end()) {
          cv::Mat raw_img(
            cv::Size(frame_info.nWidth, frame_info.nHeight), CV_8UC1,
            raw.pBufAddr);
          cv::cvtColor(raw_img, img, conversion->second);
        } else {
          tools::logger()->warn("Unsupported camera pixel type: {:#x}", pixel_type);
        }
      }

      ret = MV_CC_FreeImageBuffer(handle_, &raw);
      if (ret != MV_OK) {
        tools::logger()->warn("MV_CC_FreeImageBuffer failed: {:#x}", ret);
        break;
      }

      if (!img.empty()) queue_.push({std::move(img), timestamp});
    }

    capturing_ = false;
    tools::logger()->info("HikRobot's capture thread stopped.");
  }};
}

void HikRobot::capture_stop()
{
  capture_quit_ = true;
  if (capture_thread_.joinable()) capture_thread_.join();

  if (handle_ == nullptr) return;

  unsigned int ret;

  ret = MV_CC_StopGrabbing(handle_);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_StopGrabbing failed: {:#x}", ret);
  }

  ret = MV_CC_CloseDevice(handle_);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_CloseDevice failed: {:#x}", ret);
  }

  ret = MV_CC_DestroyHandle(handle_);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_DestroyHandle failed: {:#x}", ret);
    return;
  }
  handle_ = nullptr;
}

void HikRobot::set_float_value(const std::string & name, double value)
{
  unsigned int ret;

  ret = MV_CC_SetFloatValue(handle_, name.c_str(), value);

  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_SetFloatValue(\"{}\", {}) failed: {:#x}", name, value, ret);
    return;
  }
}

void HikRobot::set_enum_value(const std::string & name, unsigned int value)
{
  unsigned int ret;

  ret = MV_CC_SetEnumValue(handle_, name.c_str(), value);

  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_SetEnumValue(\"{}\", {}) failed: {:#x}", name, value, ret);
    return;
  }
}

void HikRobot::set_vid_pid(const std::string & vid_pid)
{
  auto index = vid_pid.find(':');
  if (index == std::string::npos) {
    tools::logger()->warn("Invalid vid_pid: \"{}\"", vid_pid);
    return;
  }

  auto vid_str = vid_pid.substr(0, index);
  auto pid_str = vid_pid.substr(index + 1);

  try {
    vid_ = std::stoi(vid_str, 0, 16);
    pid_ = std::stoi(pid_str, 0, 16);
  } catch (const std::exception &) {
    tools::logger()->warn("Invalid vid_pid: \"{}\"", vid_pid);
  }
}

void HikRobot::reset_usb() const
{
  if (vid_ == -1 || pid_ == -1) return;

  // https://github.com/ralight/usb-reset/blob/master/usb-reset.c
  auto handle = libusb_open_device_with_vid_pid(NULL, vid_, pid_);
  if (!handle) {
    tools::logger()->warn("Unable to open usb!");
    return;
  }

  if (libusb_reset_device(handle))
    tools::logger()->warn("Unable to reset usb!");
  else
    tools::logger()->info("Reset usb successfully :)");

  libusb_close(handle);
}

}  // namespace io
