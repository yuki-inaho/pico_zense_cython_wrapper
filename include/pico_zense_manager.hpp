#pragma once
#include <thread>

#include "Vzense_api2.h"
#include "common.hpp"
#include <experimental/filesystem>

namespace efs = std::experimental::filesystem;

#define MAX_DEVICECOUNT 10
class PicoZenseManager {
 public:
  PicoZenseManager();

  bool openDevice(int32_t deviceIndex);
  bool openDevice(std::string serial_number);
  void closeDevice();
  bool setupDevice(int32_t range1 = PsNearRange,
                   int32_t range2 = PsFarRange, bool isRGB = false);
  bool startDevice();
  bool updateDevice();
  void getDeviceInfo();
  std::string getSerialNumber() {
    return serialNumber_;
  }

  int32_t getDepthRange() {
    return depthRange_;
  }
  cv::Mat getDepthImage() { return depthImg_; }
  cv::Mat getIRImage() { return irImg_; }
  cv::Mat getRgbImage() { return rgbImg_; }

  CameraParameter getCameraParameter(int32_t sensor_type) {
    if (sensor_type == PsDepthSensor) {
      return camera_param_depth_;
    } else {
      return camera_param_rgb_;
    }
  }
  ExtrinsicParameter getExtrinsicParameter(){
    return extrinsic_param_;
  }

  bool setPulseCount(uint32_t pulse_count);
  bool getPulseCount(uint32_t &pulse_count);
  bool setPulseCountWDR(uint32_t pulse_count_range1, uint32_t pulse_count_range2);
  bool getPulseCountWDR(uint32_t &pulse_count_range1, uint32_t &pulse_count_range2);
  bool setDepthRange(std::string given_depth_range);

  bool isWDR() { return isWDR_; }
  bool isRGB() { return isRGB_; }

 private:
  PsDeviceInfo* pDeviceListInfo;
  PsDeviceHandle deviceHandle;
  uint32_t deviceCount_;
  uint32_t deviceIndex_;
  uint32_t sessionIndex_;

  DeviceState deviceState_;
  std::string serialNumber_;
  bool isWDR_;
  bool isRGB_;

  int32_t depthRange_;
  cv::Mat depthImg_;
  cv::Mat irImg_;
  cv::Mat rgbImg_;

  CameraParameter camera_param_depth_;
  CameraParameter camera_param_rgb_;
  ExtrinsicParameter extrinsic_param_;
  CameraParameter setCameraParameter_(PsSensorType sensor_type);
  ExtrinsicParameter setExtrinsicParameter_();
};
