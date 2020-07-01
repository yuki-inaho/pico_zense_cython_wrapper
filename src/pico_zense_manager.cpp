#include "pico_zense_manager.hpp"

using namespace std;

PicoZenseManager::PicoZenseManager() {
<<<<<<< HEAD
  for (int deviceIndex_ = 0; deviceIndex_ < MAX_DEVICECOUNT; deviceIndex_++) {
    deviceState_ = DeviceClosed;
    serialNumber_ = "";
    isWDR_ = false;
    isRGB_ = false;
=======
  for (int deviceIndex = 0; deviceIndex < MAX_DEVICECOUNT; deviceIndex++) {
    deviceState_[deviceIndex] = DeviceClosed;
    serialNumber_[deviceIndex] = "";
    isWDR_[deviceIndex] = false;
    isRGB_[deviceIndex] = false;
>>>>>>> develop
  }

  PsReturnStatus status;

<<<<<<< HEAD
GET:
  deviceCount_ = 0;
  status = Ps2_GetDeviceCount(&deviceCount_);
  if (status != PsReturnStatus::PsRetOK) {
    std::cout << "PsGetDeviceCount failed!" << std::endl;
    exit(EXIT_FAILURE);
  }
  if (0 == deviceCount_) {
    std::cout << "Waiting for device connection ..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));
    goto GET;
  }

  pDeviceListInfo = new PsDeviceInfo[deviceCount_];
  status = Ps2_GetDeviceListInfo(pDeviceListInfo, deviceCount_);
  if (status != PsReturnStatus::PsRetOK) {
    std::cout << "PsGetDeviceCount failed!" << std::endl;
    exit(EXIT_FAILURE);
  }
  std::cout << "Detected " << deviceCount_ << " devices." << std::endl;
  if (deviceCount_ > MAX_DEVICECOUNT) {
    std::cout << "# of devices exceeds maximum of " << MAX_DEVICECOUNT
              << std::endl;
    deviceCount_ = MAX_DEVICECOUNT;
  }

  sessionIndex_ = 0;
=======
  status = PsInitialize();
  if (status != PsReturnStatus::PsRetOK) {
    cout << "PsInitialize failed!" << endl;
    exit(EXIT_FAILURE);
  }

  deviceCount_ = 0;
  status = PsGetDeviceCount(&deviceCount_);
  if (status != PsReturnStatus::PsRetOK) {
    cout << "PsGetDeviceCount failed!" << endl;
    exit(EXIT_FAILURE);
  }
  cout << "Detected " << deviceCount_ << " devices." << endl;
  if (deviceCount_ > MAX_DEVICECOUNT) {
    cout << "# of devices exceeds maximum of " << MAX_DEVICECOUNT << endl;
    deviceCount_ = MAX_DEVICECOUNT;
  }
>>>>>>> develop
}

PicoZenseManager::~PicoZenseManager() {
  PsReturnStatus status;
<<<<<<< HEAD
  status = Ps2_Shutdown();
  std::cout << "Shutdown status: " << status << std::endl;
}

bool PicoZenseManager::openDevice(int32_t deviceIndex) {
  deviceIndex_ = (uint32_t)deviceIndex;
  cout << "Opening device : " << deviceIndex_ << endl;
  if (!(deviceIndex_ >= 0 && deviceIndex_ < deviceCount_)) {
=======
  status = PsShutdown();
  if (status != PsReturnStatus::PsRetOK) {
    cout << "PsShutdown failed!" << endl;
  }
}

/*
int32_t PicoZenseManager::openDeviceBySerial(string strSerial, int32_t
_deviceIndex)
{
    int32_t ret = -1;
    for (int deviceIndex=0; deviceIndex<deviceCount_; deviceIndex++)
    {
        openDevice(deviceIndex);
        if (serialNumber_[deviceIndex] == strSerial)
        {
            ret = deviceIndex;
            break;
        }
        else closeDevice(deviceIndex);
    }
    return ret;
}
*/

// fix

void PicoZenseManager::openDeviceByIdx(int32_t _deviceIndex) {
  openDevice(_deviceIndex);
}

bool PicoZenseManager::openAllDevices() {
  for (int deviceIndex = 0; deviceIndex < deviceCount_; deviceIndex++) {
    if (!openDevice(deviceIndex)) return false;
  }
  return true;
}

bool PicoZenseManager::openDevice(int32_t deviceIndex) {
  cout << "Opening device : " << deviceIndex << endl;
  if (!(deviceIndex >= 0 && deviceIndex < deviceCount_)) {
>>>>>>> develop
    cout << "Device index is out of range!" << endl;
    return false;
  }

<<<<<<< HEAD
  if (deviceState_ != DeviceClosed) {
=======
  if (deviceState_[deviceIndex] != DeviceClosed) {
>>>>>>> develop
    cout << "Device is already opened" << endl;
    return false;
  }

  PsReturnStatus status;
<<<<<<< HEAD
  std::string uri_string = std::string(pDeviceListInfo[deviceIndex_].uri);
  std::cout << "Try to open :" << uri_string << std::endl;
  status = Ps2_OpenDevice(uri_string.c_str(), &deviceHandle);
  if (status != PsReturnStatus::PsRetOK) {
    cout << "PsOpenDevice failed!" << endl;
    return false;
  }

  deviceState_ = DeviceOpened;
  return true;
}

void PicoZenseManager::closeDevice() {
  if (deviceState_ == DeviceClosed) return;

  PsReturnStatus status;
  status = Ps2_StopStream(deviceHandle, sessionIndex_);
  status = Ps2_CloseDevice(deviceHandle);
  if (status != PsReturnStatus::PsRetOK) {
    std::cout << "CloseDevice failed!" << std::endl;
  } else {
    std::cout << "Device Closed: " << deviceIndex_ << std::endl;
  }
  deviceState_ = DeviceClosed;
}

bool PicoZenseManager::startDevice() {
  if (deviceState_ != DeviceOpened) {
    cout << "Device is not opened" << endl;
    return false;
  }
  if (deviceState_ == DeviceStarted) {
    cout << "Device is already started" << endl;
    return false;
  }

  PsReturnStatus status;
  Ps2_StartStream(deviceHandle, sessionIndex_);

  int32_t lenSerial = 100;
  char buffSerial[lenSerial];
  status = Ps2_GetProperty(deviceHandle, sessionIndex_, PsPropertySN_Str,
                           buffSerial, &lenSerial);
  serialNumber_ = buffSerial;
  std::cout << "SERIAL : " << buffSerial << std::endl;

  int32_t lenHW = 100;
  char buffHW[lenHW];
  status = Ps2_GetProperty(deviceHandle, sessionIndex_, PsPropertyHWVer_Str,
                           buffHW, &lenHW);
  std::cout << "HW_VER : " << buffHW << std::endl;

  int32_t lenFW = 100;
  char buffFW[lenFW];
  status = Ps2_GetProperty(deviceHandle, sessionIndex_, PsPropertyFWVer_Str,
                           buffFW, &lenFW);
  std::cout << "FW_VER : " << buffFW << std::endl;
  std::cout << std::endl;

  cout << "Started capturing on device : " << deviceIndex_ << endl;
  cout << endl;

  deviceState_ = DeviceStarted;
  return true;
}

bool PicoZenseManager::setupDevice(int32_t range1, int32_t range2, bool isRGB) {
  cout << "Setting up device : " << deviceIndex_ << endl;
  if (!(deviceIndex_ >= 0 && deviceIndex_ < deviceCount_)) {
=======

  status = PsOpenDevice(deviceIndex);
  if (status != PsReturnStatus::PsRetOK) {
    cout << "PsOpenDevice failed!" << endl;
    return false;
  }

  int32_t lenSerial = 100;
  char buffSerial[lenSerial];
  status = PsGetProperty(deviceIndex, PsPropertySN_Str, buffSerial, &lenSerial);
  serialNumber_[deviceIndex] = buffSerial;
  cout << "SERIAL : " << buffSerial << endl;

  int32_t lenHW = 100;
  char buffHW[lenHW];
  status = PsGetProperty(deviceIndex, PsPropertyHWVer_Str, buffHW, &lenHW);
  cout << "HW_VER : " << buffHW << endl;

  int32_t lenFW = 100;
  char buffFW[lenFW];
  status = PsGetProperty(deviceIndex, PsPropertyFWVer_Str, buffFW, &lenFW);
  cout << "FW_VER : " << buffFW << endl;

  cout << endl;

  deviceState_[deviceIndex] = DeviceOpened;
  return true;
}

void PicoZenseManager::closeDevice(int32_t deviceIndex) {
  if (deviceState_[deviceIndex] == DeviceClosed) return;
  PsReturnStatus status;

  /* Though it is recommended in SDK document, PsStopFrame call causes error and
  SDK sample programs are not using it so commenting out the below lines. if
  (deviceState_[deviceIndex] == DeviceStarted)
      {
          if (isWDR_[deviceIndex])
          {
              status = PsStopFrame(deviceIndex, PsWDRDepthFrame);
          }
          else
          {
              status = PsStopFrame(deviceIndex, PsDepthFrame);
              status = PsStopFrame(deviceIndex, PsIRFrame);
  //            status = PsStopFrame(deviceIndex, PsRGBFrame);
          }
      }
  */
  status = PsCloseDevice(deviceIndex);
  cout << "Closed device : " << deviceIndex << endl;
  deviceState_[deviceIndex] = DeviceClosed;
}

bool PicoZenseManager::setupDevice(int32_t deviceIndex, int32_t range1,
                                   int32_t range2, bool isRGB) {
  cout << "Setting up device : " << deviceIndex << endl;
  if (!(deviceIndex >= 0 && deviceIndex < deviceCount_)) {
>>>>>>> develop
    cout << "Device index is out of range!" << endl;
    return false;
  }

<<<<<<< HEAD
  if (deviceState_ == DeviceClosed) return false;
=======
  if (deviceState_[deviceIndex] == DeviceClosed) return false;

>>>>>>> develop
  PsReturnStatus status;

  // Operating Mode
  int32_t dataMode;
<<<<<<< HEAD
  isWDR_ = !(range2 < PsNearRange);
  isRGB_ = isRGB;
  if (isWDR_) {
    dataMode = PsWDR_Depth;
  } else if (isRGB_) {
    dataMode = PsDepthAndRGB_30;
  } else {
    dataMode = PsDepthAndIR_30;
  }

  status = Ps2_SetDataMode(deviceHandle, sessionIndex_, (PsDataMode)dataMode);
=======
  isWDR_[deviceIndex] = !(range2 < PsNearRange);
  isRGB_[deviceIndex] = isRGB;
  if (isWDR_[deviceIndex]) dataMode = PsWDR_Depth;
  //    else if (isRGB_[deviceIndex]) dataMode = PsDepthAndRGB_30;
  else if (isRGB_[deviceIndex])
    dataMode = PsDepthAndIR_15_RGB_30;
  else
    dataMode = PsDepthAndIR_30;
  status = PsSetDataMode(deviceIndex, (PsDataMode)dataMode);
>>>>>>> develop
  if (status != PsReturnStatus::PsRetOK) {
    cout << "PsSetDataMode failed!" << endl;
    return false;
  }

  // Set depth range
  string strRange[10];
  strRange[PsNearRange] = "near";
  strRange[PsMidRange] = "mid";
  strRange[PsFarRange] = "far";

<<<<<<< HEAD
  if (isWDR_) {
=======
  status = PsSetDepthRange(deviceIndex, (PsDepthRange)range1);
  if (status != PsReturnStatus::PsRetOK) {
    cout << "PsSetDepthRange failed!" << endl;
    return false;
  }
  if (isWDR_[deviceIndex]) {
>>>>>>> develop
    PsWDROutputMode modeWDR = {PsWDRTotalRange_Two,
                               (PsDepthRange)range1,
                               1,
                               (PsDepthRange)range2,
                               1,
                               PsNearRange,
                               1};
<<<<<<< HEAD
    status = Ps2_SetWDROutputMode(deviceHandle, sessionIndex_, &modeWDR);
    if (status != PsReturnStatus::PsRetOK) {
      std::cout << "PsSetWDROutputMode failed!" << std::endl;
      return false;
    }

    status = Ps2_SetWDRStyle(deviceHandle, sessionIndex_, PsWDR_ALTERNATION);
    if (status != PsReturnStatus::PsRetOK) {
      std::cout << "PsSetWDRStyle failed!" << std::endl;
      return false;
    }
    std::cout << "WDR mode : " << strRange[range1] << "-" << strRange[range2]
              << std::endl;
  } else {
    status =
        Ps2_SetDepthRange(deviceHandle, sessionIndex_, (PsDepthRange)range1);
    if (status != PsReturnStatus::PsRetOK) {
      std::cout << "PsSetDepthRange failed!" << std::endl;
      return false;
    }
    std::cout << "Single range mode : " << strRange[range1] << std::endl;
  }

  // Distortion
  Ps2_SetDepthDistortionCorrectionEnabled(deviceHandle, sessionIndex_, true);
  Ps2_SetIrDistortionCorrectionEnabled(deviceHandle, sessionIndex_, true);
  Ps2_SetRGBDistortionCorrectionEnabled(deviceHandle, sessionIndex_, true);

  // Filters
  Ps2_SetComputeRealDepthCorrectionEnabled(deviceHandle, sessionIndex_, true);
  Ps2_SetSpatialFilterEnabled(deviceHandle, sessionIndex_, true);
  Ps2_SetTimeFilterEnabled(deviceHandle, sessionIndex_, true);

  // RGB resolution
  Ps2_SetRGBResolution(deviceHandle, sessionIndex_, PsRGB_Resolution_1920_1080);
  Ps2_SetColorPixelFormat(deviceHandle, sessionIndex_, PsPixelFormatBGR888);
  if (!isRGB) {
    Ps2_SetRgbFrameEnabled(deviceHandle, sessionIndex_, false);
  }

  status = Ps2_SetMapperEnabledRGBToDepth(deviceHandle, sessionIndex_, false);
  if (status != PsRetOK) {
    std::cout << "PsSetMapperEnabledRGBToDepth failed!" << std::endl;
    return false;
  }

  status = Ps2_SetMapperEnabledDepthToRGB(deviceHandle, sessionIndex_, false);
  if (status != PsRetOK) {
    std::cout << "PsSetMapperEnabledDepthToRGB failed!" << std::endl;
    return false;
  }

  camera_param_depth_ = setCameraParameter_(PsDepthSensor);
  camera_param_rgb_ = setCameraParameter_(PsRgbSensor);

  return true;
}

bool PicoZenseManager::updateDevice() {
  bool isSuccess = false;
  PsReturnStatus status;

  if (deviceState_ != DeviceStarted) {
    cout << "Device " << deviceIndex_ << " has not started!" << endl;
    return isSuccess;
  }

  PsFrameReady frameReady = {0};
  status = Ps2_ReadNextFrame(deviceHandle, sessionIndex_, &frameReady);
  if (status != PsRetOK) {
    std::cout << "Could not read next frame from device "
              << (uint32_t)deviceIndex_ << std::endl;
    return isSuccess;
  }

  if (isWDR_ && (1 == frameReady.wdrDepth)) {
    // WDR Depth
    PsFrame depthFrame = {0};
    Ps2_GetFrame(deviceHandle, sessionIndex_, PsWDRDepthFrame, &depthFrame);
    if (depthFrame.pFrameData != NULL) {
      depthRange_ = depthFrame.depthRange;
      depthImg_ = cv::Mat(depthFrame.height, depthFrame.width, CV_16UC1,
                          depthFrame.pFrameData);
      irImg_ = depthImg_;
=======
    status = PsSetWDROutputMode(deviceIndex, &modeWDR);
    if (status != PsReturnStatus::PsRetOK) {
      cout << "PsSetWDROutputMode failed!" << endl;
      return false;
    }
    status = PsSetWDRStyle(deviceIndex, PsWDR_ALTERNATION);
    if (status != PsReturnStatus::PsRetOK) {
      cout << "PsSetWDRStyle failed!" << endl;
      return false;
    }
    cout << "WDR mode : " << strRange[range1] << "-" << strRange[range2]
         << endl;
  } else {
    cout << "Single range mode : " << strRange[range1] << endl;
  }

  // Distortion
  PsSetDepthDistortionCorrectionEnabled(deviceIndex, true);
  PsSetIrDistortionCorrectionEnabled(deviceIndex, true);
  PsSetRGBDistortionCorrectionEnabled(deviceIndex, true);

  // TODO: tune these!!!
  // Filters
  PsSetFilter(deviceIndex, PsComputeRealDepthFilter, true);  // default : true
  PsSetFilter(deviceIndex, PsSmoothingFilter, true);         // default : true
  //    PsSetFilter(deviceIndex, PSSpatialFilter, false);

  // RGB resolution
  PsFrameMode frameMode;
  frameMode.fps = 30;
  frameMode.pixelFormat = PsPixelFormatBGR888;
  frameMode.resolutionWidth = 1920;
  frameMode.resolutionHeight = 1080;
  PsSetFrameMode(deviceIndex, PsRGBFrame, &frameMode);

  PsSetColorPixelFormat(deviceIndex, PsPixelFormatBGR888);

  status = PsSetMapperEnabledRGBToDepth(deviceIndex, false);
  if (status != PsRetOK) {
    cout << "PsSetMapperEnabledRGBToDepth failed!" << endl;
    return false;
  }

  status = PsSetMapperEnabledDepthToRGB(deviceIndex, false);
  if (status != PsRetOK) {
    cout << "PsSetMapperEnabledDepthToRGB failed!" << endl;
    return false;
  }

  //////////////////////////////////////////////////////////////////////////////
  /* TODO memos!!!!!

  PsDepthVector3

  PsSetWDRFusionThreshold

  PsGetThreshold
  PsSetThreshold

  PsGetTimeFilterEnabled
  PsSetTimeFilterEnabled

  PsSetComputeRealDepthCorrectionEnabled
  PsSetSmoothingFilterEnabled
  PsSetSpatialFilterEnabled

  */
  //////////////////////////////////////////////////////////////////////////////

  cameraParams_[deviceIndex][0] =
      updateCameraParameter_(deviceIndex, PsDepthSensor);
  cameraParams_[deviceIndex][1] =
      updateCameraParameter_(deviceIndex, PsRgbSensor);

  return true;
}

bool PicoZenseManager::startDevice(int32_t deviceIndex) {
  if (deviceState_[deviceIndex] == DeviceClosed) return false;
  if (deviceState_[deviceIndex] == DeviceStarted) return false;

  PsReturnStatus status;
  /* Though it is recommended in SDK document, PsStopFrame call causes error and
  SDK sample programs are not using it so commenting out the below lines. if
  (isWDR_[deviceIndex])
      {
          status = PsStartFrame(deviceIndex, PsWDRDepthFrame);
      }
      else
      {
          status = PsStartFrame(deviceIndex, PsDepthFrame);
          status = PsStartFrame(deviceIndex, PsIRFrame);
  //        status = PsStartFrame(deviceIndex, PsRGBFrame);
      }
      if (status != PsRetOK)
      {
          cout << "PsStartFrame failed!" << endl;
          return false;
      }
  */
  cout << "Started capturing on device : " << deviceIndex << endl;
  cout << endl;

  deviceState_[deviceIndex] = DeviceStarted;
  return true;
}

bool PicoZenseManager::updateDevice(int32_t deviceIndex) {
  bool isSuccess = false;
  PsReturnStatus status;

  if (deviceState_[deviceIndex] != DeviceStarted) {
    cout << "Device " << deviceIndex << " has not started!" << endl;
    return isSuccess;
  }
  status = PsReadNextFrame(deviceIndex);

  if (status != PsRetOK) {
    cout << "Could not read next frame from device " << deviceIndex << endl;
    return isSuccess;
  }
  // imageTimestamps_[deviceIndex] = ros::Time::now();

  if (isWDR_[deviceIndex]) {
    // WDR Depth
    PsFrame depthFrame = {0};
    PsGetFrame(deviceIndex, PsWDRDepthFrame, &depthFrame);
    if (depthFrame.pFrameData != NULL) {
      depthRange_[deviceIndex] = depthFrame.depthRange;
      depthImg_[deviceIndex] = cv::Mat(depthFrame.height, depthFrame.width,
                                       CV_16UC1, depthFrame.pFrameData);
      irImg_[deviceIndex] = depthImg_[deviceIndex];
>>>>>>> develop
      isSuccess = true;
    }
  } else {
    // Depth
<<<<<<< HEAD
    if (1 == frameReady.depth) {
      PsFrame depthFrame = {0};
      Ps2_GetFrame(deviceHandle, sessionIndex_, PsDepthFrame, &depthFrame);
      if (depthFrame.pFrameData != NULL) {
        depthRange_ = depthFrame.depthRange;
        depthImg_ = cv::Mat(depthFrame.height, depthFrame.width, CV_16UC1,
                            depthFrame.pFrameData);
        irImg_ = depthImg_;
        isSuccess = true;
      }
    }

    // IR
    if (1 == frameReady.ir) {
      PsFrame irFrame = {0};
      Ps2_GetFrame(deviceHandle, sessionIndex_, PsIRFrame, &irFrame);
      if (irFrame.pFrameData != NULL) {
        irImg_ = cv::Mat(irFrame.height, irFrame.width, CV_16UC1,
                         irFrame.pFrameData);
      }
    }

    // RGB
    if (1 == frameReady.rgb) {
      if (isRGB_) {
        PsFrame rgbFrame = {0};
        Ps2_GetFrame(deviceHandle, sessionIndex_, PsRGBFrame, &rgbFrame);
        if (rgbFrame.pFrameData != NULL) {
          rgbImg_ = cv::Mat(rgbFrame.height, rgbFrame.width, CV_8UC3,
                            rgbFrame.pFrameData);
        }
=======
    PsFrame depthFrame = {0};
    PsGetFrame(deviceIndex, PsDepthFrame, &depthFrame);
    if (depthFrame.pFrameData != NULL) {
      depthRange_[deviceIndex] = depthFrame.depthRange;
      depthImg_[deviceIndex] = cv::Mat(depthFrame.height, depthFrame.width,
                                       CV_16UC1, depthFrame.pFrameData);
      irImg_[deviceIndex] = depthImg_[deviceIndex];
      isSuccess = true;
    }

    // IR
    PsFrame irFrame = {0};
    PsGetFrame(deviceIndex, PsIRFrame, &irFrame);
    if (irFrame.pFrameData != NULL) {
      irImg_[deviceIndex] =
          cv::Mat(irFrame.height, irFrame.width, CV_16UC1, irFrame.pFrameData);
    }

    // RGB
    if (isRGB_[deviceIndex]) {
      PsFrame rgbFrame = {0};
      PsGetFrame(deviceIndex, PsRGBFrame, &rgbFrame);
      if (rgbFrame.pFrameData != NULL) {
        rgbImg_[deviceIndex] = cv::Mat(rgbFrame.height, rgbFrame.width, CV_8UC3,
                                       rgbFrame.pFrameData);
>>>>>>> develop
      }
    }
  }

  if (isSuccess) {
<<<<<<< HEAD
    if (depthImg_.rows == 0) isSuccess = false;
    if (irImg_.rows == 0) isSuccess = false;
=======
    if (depthImg_[deviceIndex].rows == 0) isSuccess = false;
    if (irImg_[deviceIndex].rows == 0) isSuccess = false;
>>>>>>> develop
  }

  return isSuccess;
}

<<<<<<< HEAD
CameraParameter PicoZenseManager::setCameraParameter_(
    PsSensorType sensor_type) {
=======
int32_t PicoZenseManager::getDeviceIndex(string strSerial) {
  int32_t ret = -1;
  for (int deviceIndex = 0; deviceIndex < deviceCount_; deviceIndex++) {
    if (serialNumber_[deviceIndex] == strSerial) {
      ret = deviceIndex;
      break;
    }
  }
  if (ret < 0) {
    cout << "Device with serial# " << strSerial << " is not found!" << endl;
  } else {
    cout << "Device with serial# " << strSerial << " is found at index :" << ret
         << endl;
  }
  return ret;
}

CameraParameter PicoZenseManager::updateCameraParameter_(
    int32_t deviceIndex, PsSensorType sensor_type) {
>>>>>>> develop
  PsReturnStatus status;
  PsCameraParameters cameraParameters;

  std::string sensor_type_str;
<<<<<<< HEAD
  status = Ps2_GetCameraParameters(deviceHandle, sessionIndex_, sensor_type,
                                   &cameraParameters);
=======

  status = PsGetCameraParameters(deviceIndex, sensor_type, &cameraParameters);
>>>>>>> develop

  CameraParameter cameraParam;
  //    cameraParam.image_width = cameraParameters.;
  //    cameraParam.image_height = cameraParameters.;
  cameraParam.fx = cameraParameters.fx;
  cameraParam.fy = cameraParameters.fy;
  cameraParam.cx = cameraParameters.cx;
  cameraParam.cy = cameraParameters.cy;
  cameraParam.p1 = cameraParameters.p1;
  cameraParam.p2 = cameraParameters.p2;
  cameraParam.k1 = cameraParameters.k1;
  cameraParam.k2 = cameraParameters.k2;
  cameraParam.k3 = cameraParameters.k3;
  cameraParam.k4 = cameraParameters.k4;
  cameraParam.k5 = cameraParameters.k5;
  cameraParam.k6 = cameraParameters.k6;

  if (sensor_type == PsDepthSensor) {
    sensor_type_str = "Depth camera";
  } else {
    sensor_type_str = "RGB camera";
  }

<<<<<<< HEAD
  cout << "Intinsic parameters of device : " << deviceIndex_ << endl;
  cout << "serial_no = \"" << serialNumber_ << "\"" << endl;
  cout << "Camera type: " << sensor_type_str << endl;
  // printCameraParams(cameraParam);
=======
  cout << "Intinsic parameters of device : " << deviceIndex << endl;
  cout << "serial_no = \"" << serialNumber_[deviceIndex] << "\"" << endl;
  cout << "Camera type: " << sensor_type_str << endl;
  printCameraParams(cameraParam);
>>>>>>> develop

  return cameraParam;
}

<<<<<<< HEAD
ExtrinsicParameter PicoZenseManager::setExtrinsicParameter_() {
  PsReturnStatus status;
  PsCameraExtrinsicParameters pCameraExtrinsicParameters;
  status = Ps2_GetCameraExtrinsicParameters(deviceHandle, sessionIndex_,
                                            &pCameraExtrinsicParameters);
  if (status != PsReturnStatus::PsRetOK) {
    std::cout << "Ps2_GetCameraExtrinsicParameters failed!" << std::endl;
    std::exit(EXIT_FAILURE);
  }
  std::vector<double> _rotation(std::begin(pCameraExtrinsicParameters.rotation),
                                std::end(pCameraExtrinsicParameters.rotation));
  std::vector<double> _translation(
      std::begin(pCameraExtrinsicParameters.translation),
      std::end(pCameraExtrinsicParameters.translation)
  );
  extrinsic_param_.rotation = _rotation;
  extrinsic_param_.translation = _translation;
  return extrinsic_param_;
}

bool PicoZenseManager::setPulseCount(uint32_t pulse_count) {
  PsReturnStatus status;
  uint16_t _p_pulse_count = static_cast<uint16_t>(pulse_count);
  status = Ps2_SetPulseCount(deviceHandle, sessionIndex_, _p_pulse_count);
  if (status != PsReturnStatus::PsRetOK) {
    std::cerr << "PsSetPulseCount failed!" << std::endl;
=======
bool PicoZenseManager::getPulseCount(int32_t deviceIndex,
                                     uint32_t &pulseCount) {
  PsReturnStatus status;
  uint16_t _pPulseCount;
  status = PsGetPulseCount(deviceIndex, &_pPulseCount);
  pulseCount = static_cast<uint32_t>(_pPulseCount);
  if (status != PsReturnStatus::PsRetOK) {
    cerr << "PsGetPulseCount failed!" << endl;
>>>>>>> develop
    return false;
  }
  return true;
}

<<<<<<< HEAD
bool PicoZenseManager::getPulseCount(uint32_t &pulse_count) {
  PsReturnStatus status;
  uint16_t _p_pulse_count;
  status = Ps2_GetPulseCount(deviceHandle, sessionIndex_, &_p_pulse_count);
  if (status != PsReturnStatus::PsRetOK) {
    std::cerr << "PsGetPulseCount failed!" << std::endl;
    return false;
  }
  pulse_count = static_cast<uint32_t>(_p_pulse_count);
  return true;
}

bool PicoZenseManager::setPulseCountWDR(uint32_t pulse_count_range1,
                                        uint32_t pulse_count_range2) {
  PsReturnStatus status;
  PsWDRPulseCount wdrPulseCount;
  wdrPulseCount.pulseCount1 = static_cast<uint16_t>(pulse_count_range1);
  wdrPulseCount.pulseCount2 = static_cast<uint16_t>(pulse_count_range2);
  status = Ps2_SetWDRPulseCount(deviceHandle, sessionIndex_, wdrPulseCount);
  if (status != PsReturnStatus::PsRetOK) {
    std::cerr << "PsSetWDRPulseCount failed!" << std::endl;
    return false;
  }
  return true;
}

bool PicoZenseManager::getPulseCountWDR(uint32_t &pulse_count_range1,
                                        uint32_t &pulse_count_range2) {
  PsReturnStatus status;
  PsWDRPulseCount pwdrPulseCount;
  status = Ps2_GetWDRPulseCount(deviceHandle, sessionIndex_, &pwdrPulseCount);
  if (status != PsReturnStatus::PsRetOK) {
    std::cerr << "PsGetWDRPulseCount failed!" << std::endl;
    return false;
  }
  pulse_count_range1 = static_cast<uint32_t>(pwdrPulseCount.pulseCount1);
  pulse_count_range2 = static_cast<uint32_t>(pwdrPulseCount.pulseCount2);
  return true;
}


bool PicoZenseManager::setDepthRange(std::string given_depth_range) {
=======
bool PicoZenseManager::setPulseCount(int32_t deviceIndex, uint32_t pulseCount) {
  PsReturnStatus status;
  uint16_t _pPulseCount = static_cast<uint16_t>(pulseCount);
  status = PsSetPulseCount(deviceIndex, _pPulseCount);
  if (status != PsReturnStatus::PsRetOK) {
    cerr << "PsSetPulseCount failed!" << endl;
    return false;
  }
  return true;
}

bool PicoZenseManager::setDepthRange(int32_t deviceIndex,
                                     std::string given_depth_range) {
>>>>>>> develop
  PsDepthRange targetRange;
  if (given_depth_range == "Near") {
    targetRange = PsNearRange;
  } else if (given_depth_range == "Mid") {
    targetRange = PsMidRange;
  } else if (given_depth_range == "Far") {
    targetRange = PsFarRange;
  } else {
<<<<<<< HEAD
    std::cout << "Currently depth range is supported only in [Near, Mid, Far] mode" << std::endl;
=======
    cout << "Currently depth range is supported only in [Near, Mid, Far] mode" << endl;
>>>>>>> develop
    return false;
  }

  PsReturnStatus status;
<<<<<<< HEAD
  status = Ps2_SetDepthRange(deviceHandle, sessionIndex_, targetRange);
  if (status != PsReturnStatus::PsRetOK) {
    std::cerr << "Depth range changing is failed" << std::endl;
=======
  status = PsSetDepthRange(deviceIndex, targetRange);
  if (status != PsReturnStatus::PsRetOK) {
    cerr << "Depth range changing is failed" << endl;
>>>>>>> develop
    exit(EXIT_FAILURE);
  }

  PsDepthRange resetRange;
<<<<<<< HEAD
  status = Ps2_GetDepthRange(deviceHandle, sessionIndex_, &resetRange);
  if (status != PsReturnStatus::PsRetOK || targetRange != resetRange) {
    std::cerr << "Depth range changing is failed ()" << std::endl;
    exit(EXIT_FAILURE);
  }
  depthRange_ = targetRange;

  if (targetRange == PsNearRange)
    std::cout << "Set depth range to Near," << std::endl;
  else if (targetRange == PsMidRange)
    std::cout << "Set depth range to Mid," << std::endl;
  else if (targetRange == PsFarRange)
    std::cout << "Set depth range to Far," << std::endl;
  else {
    std::cerr << "Invarid depth range setting detected" << std::endl;
=======
  status = PsGetDepthRange(deviceIndex, &resetRange);
  if (status != PsReturnStatus::PsRetOK || targetRange != resetRange) {
    cerr << "Depth range changing is failed ()" << endl;
    exit(EXIT_FAILURE);
  }
  depthRange_[deviceIndex] = targetRange;

  if (targetRange == PsNearRange)
    cout << "Set depth range to Near," << endl;
  else if (targetRange == PsMidRange)
    cout << "Set depth range to Mid," << endl;
  else if (targetRange == PsFarRange)
    cout << "Set depth range to Far," << endl;
  else {
    cerr << "Invarid depth range setting detected" << endl;
>>>>>>> develop
    exit(EXIT_FAILURE);
  }

  return true;
}