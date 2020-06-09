#include <omp.h>
#include <fstream>
#include <functional>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <thread>
#include "Vzense_api2.h"

#define MAX_DEVICECOUNT 10

struct CameraParameter {
  int image_width;
  int image_height;
  float fx;  //!< Focal length x (pixel)
  float fy;  //!< Focal length y (pixel)
  float cx;  //!< Principal point x (pixel)
  float cy;  //!< Principal point y (pixel)
  float p1;  //!< Tangential distortion coefficient
  float p2;  //!< Tangential distortion coefficient
  float k1;  //!< Radial distortion coefficient, 1st-order
  float k2;  //!< Radial distortion coefficient, 2nd-order
  float k3;  //!< Radial distortion coefficient, 3rd-order
  float k4;  //!< Radial distortion coefficient, 4st-order
  float k5;  //!< Radial distortion coefficient, 5nd-order
  float k6;  //!< Radial distortion coefficient, 6rd-order
};

struct ExtrinsicParameter {
  std::vector<double> rotation;
  std::vector<double> translation;
};

namespace zense {
class PicoZenseManager {
 public:
  PicoZenseManager(uint32_t sensor_idx_);
  ~PicoZenseManager();
  void printCameraParams(CameraParameter cameraParam);
  bool update();
  cv::Mat getIRImage();
  cv::Mat getRGBImage();
  cv::Mat getDepthImage();

  std::string getSerialNumber();
  std::vector<double> getCameraParameter();
  std::vector<std::vector<double>> getExtrinsicParameter();
  std::vector<double> getRGBCameraParameter();

 private:
  PsDeviceInfo* pDeviceListInfo;
  PsDeviceHandle deviceHandle;
  uint32_t sessionIndex;
  uint32_t deviceCount_;

  std::string serialNumber_;
  CameraParameter camera_param_;
  CameraParameter camera_param_rgb_;
  ExtrinsicParameter extrinsic_param_;
  cv::Mat depthImg_;
  cv::Mat irImg_;
  cv::Mat rgbImg_;
};

}  // namespace zense