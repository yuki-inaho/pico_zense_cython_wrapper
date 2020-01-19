#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <omp.h>
#include <functional>
#include <opencv2/opencv.hpp>
#include "PicoZense_api.h"

#define MAX_DEVICECOUNT 1

struct CameraParameter{
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

namespace zense{
  class PicoZenseManager
  {
    public:
      PicoZenseManager();
      ~PicoZenseManager();
      void printCameraParams(CameraParameter cameraParam);
      bool updateDevice();
      cv::Mat getRGBImage();
      cv::Mat getDepthImage();
      std::string getSerialNumber();
      std::vector<double> getCameraParameter();

    private:
      std::string serialNumber_;
      CameraParameter camera_param_;
      cv::Mat depthImg_;
      cv::Mat rgbImg_;
  };

}

