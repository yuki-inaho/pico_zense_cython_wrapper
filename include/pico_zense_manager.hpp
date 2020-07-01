#pragma once
<<<<<<< HEAD
#include <thread>

#include "Vzense_api2.h"
=======
#include "PicoZense_api.h"
>>>>>>> develop
#include "common.hpp"

#define MAX_DEVICECOUNT 10

class PicoZenseManager {
 public:
  PicoZenseManager();
  ~PicoZenseManager();

<<<<<<< HEAD
  bool openDevice(int32_t deviceIndex);
  void closeDevice();
  bool setupDevice(int32_t range1 = PsNearRange,
                   int32_t range2 = PsFarRange, bool isRGB = false);
  bool startDevice();
  bool updateDevice();
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
=======
  void openDeviceByIdx(int32_t _deviceIdx);

  bool openAllDevices();
  bool openDevice(int32_t deviceIndex);

  void closeAllDevices();
  void closeDevice(int32_t deviceIndex);

  bool setupDevice(int32_t deviceIndex, int32_t range1 = PsNearRange,
                   int32_t range2 = PsFarRange, bool isRGB = false);
  bool startDevice(int32_t deviceIndex);

  bool updateDevice(int32_t deviceIndex);

  int32_t getDeviceCount() { return deviceCount_; }
  std::string getSerialNumber(int32_t deviceIndex) {
    return serialNumber_[deviceIndex];
  }
  int32_t getDeviceIndex(std::string strSerial);

  void setSmoothingFilter(int32_t deviceIndex, bool enable) {
    PsSetFilter(deviceIndex, PsSmoothingFilter, enable);
    std::cout << "Device " << deviceIndex
              << " SmoothingFilter : " << (enable ? "ON" : "OFF") << std::endl;
  }

  bool isWDR(int32_t deviceIndex) { return isWDR_[deviceIndex]; }
  bool isRGB(int32_t deviceIndex) { return isRGB_[deviceIndex]; }

  /*
  ros::Time getImageTimestamp(int32_t deviceIndex) {
    return imageTimestamps_[deviceIndex];
  }
  */
  bool setDepthRange(int32_t deviceIndex, std::string given_depth_range); 
  int32_t getDepthRange(int32_t deviceIndex) {
    return depthRange_[deviceIndex];
  }

  cv::Mat getDepthImage(int32_t deviceIndex) { return depthImg_[deviceIndex]; }
  cv::Mat getIRImage(int32_t deviceIndex) { return irImg_[deviceIndex]; }
  cv::Mat getRgbImage(int32_t deviceIndex) { return rgbImg_[deviceIndex]; }

  // @param sensor_type 0: depth, 1: rgb
  CameraParameter getCameraParameter(int32_t deviceIndex, int32_t sensor_type) {
    return cameraParams_[deviceIndex][sensor_type];
  }

  ExtrinsicParameter getExtrinsicParameter(int32_t deviceIndex) {
    PsCameraExtrinsicParameters pCameraExtrinsicParameters;
    ExtrinsicParameter extrinsic_param_;
    PsGetCameraExtrinsicParameters(deviceIndex, &pCameraExtrinsicParameters);
    std::vector<double> _rotation(std::begin(pCameraExtrinsicParameters.rotation),
                                  std::end(pCameraExtrinsicParameters.rotation));
    std::vector<double> _translation(
        std::begin(pCameraExtrinsicParameters.translation),
        std::end(pCameraExtrinsicParameters.translation));
    extrinsic_param_.rotation = _rotation;
    extrinsic_param_.translation = _translation;
    return extrinsic_param_;
  }

  bool getPulseCount(int32_t deviceIndex, uint32_t &pulseCount);
  bool setPulseCount(int32_t deviceIndex, uint32_t pulseCount);

  typedef enum {
    DeviceClosed = 0,
    DeviceOpened = 1,
    DeviceStarted = 2,
  } DeviceState;

 private:
  int32_t deviceCount_;

  DeviceState deviceState_[MAX_DEVICECOUNT];
  std::string serialNumber_[MAX_DEVICECOUNT];
  bool isWDR_[MAX_DEVICECOUNT];
  bool isRGB_[MAX_DEVICECOUNT];

  //ros::Time imageTimestamps_[MAX_DEVICECOUNT];
  int32_t depthRange_[MAX_DEVICECOUNT];
  cv::Mat depthImg_[MAX_DEVICECOUNT];
  cv::Mat irImg_[MAX_DEVICECOUNT];
  cv::Mat rgbImg_[MAX_DEVICECOUNT];

  CameraParameter cameraParams_[MAX_DEVICECOUNT][2];  // 0: depth, 1: rgb

  CameraParameter updateCameraParameter_(
      int32_t deviceIndex, PsSensorType sensor_type = PsDepthSensor);
};
>>>>>>> develop
