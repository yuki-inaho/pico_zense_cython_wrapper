#include "pico_zense_manager.hpp"

using namespace std;
namespace zense
{
  PicoZenseManager::PicoZenseManager(uint32_t device_idx)
  {
    sessionIndex = device_idx;
    PsReturnStatus status;
    status = Ps2_Initialize();
    if (status != PsReturnStatus::PsRetOK)
    {
      cout << "PsInitialize failed!" << endl;
      exit(EXIT_FAILURE);
    }

  GET:
    deviceCount_ = 0;
    status = Ps2_GetDeviceCount(&deviceCount_);
    if (status != PsReturnStatus::PsRetOK)
    {
      cout << "PsGetDeviceCount failed!" << endl;
      exit(EXIT_FAILURE);
    }
    cout << "Detected " << deviceCount_ << " devices." << endl;
    if (0 == deviceCount_)
    {
      this_thread::sleep_for(chrono::seconds(1));
      goto GET;
    }

    pDeviceListInfo = new PsDeviceInfo[deviceCount_];
    status = Ps2_GetDeviceListInfo(pDeviceListInfo, deviceCount_);
    if (status != PsReturnStatus::PsRetOK)
    {
      cout << "PsGetDeviceCount failed!" << endl;
      exit(EXIT_FAILURE);
    }
    if (deviceCount_ > MAX_DEVICECOUNT)
    {
      cout << "# of devices exceeds maximum of " << MAX_DEVICECOUNT << endl;
      deviceCount_ = MAX_DEVICECOUNT;
    }

    status = Ps2_OpenDevice(pDeviceListInfo[sessionIndex].uri, &deviceHandle);
    if (status != PsReturnStatus::PsRetOK)
    {
      cout << "PsOpenDevice failed!" << endl;
      exit(EXIT_FAILURE);std::cout << sessionIndex << std::endl;
    }

    //sessionIndex = pDeviceListInfo[device_idx_].SessionCount;
    Ps2_StartStream(deviceHandle, sessionIndex);
    std::cout << "session index :" << sessionIndex << std::endl;

    // get Serial Number
    int32_t lenSerial = 100;
    char buffSerial[lenSerial];
    status = Ps2_GetProperty(deviceHandle, sessionIndex, PsPropertySN_Str,
                             buffSerial, &lenSerial);
    serialNumber_ = buffSerial;
    cout << "SERIAL : " << buffSerial << endl;
    cout << "connection" << pDeviceListInfo[sessionIndex].status << endl;

    // set Depth Camera Parameter
    PsCameraParameters camera_parameters;
    status = Ps2_GetCameraParameters(deviceHandle, sessionIndex, PsDepthSensor, &camera_parameters);

    cout << "serial_no = \"" << serialNumber_ << "\"" << endl;
    camera_param_.fx = camera_parameters.fx;
    camera_param_.fy = camera_parameters.fy;
    camera_param_.cx = camera_parameters.cx;
    camera_param_.cy = camera_parameters.cy;
    camera_param_.p1 = camera_parameters.p1;
    camera_param_.p2 = camera_parameters.p2;
    camera_param_.k1 = camera_parameters.k1;
    camera_param_.k2 = camera_parameters.k2;
    camera_param_.k3 = camera_parameters.k3;
    camera_param_.k4 = camera_parameters.k4;
    camera_param_.k5 = camera_parameters.k5;
    camera_param_.k6 = camera_parameters.k6;

    PsCameraParameters camera_parameters_rgb;
    status = Ps2_GetCameraParameters(deviceHandle, sessionIndex, PsRgbSensor, &camera_parameters_rgb);
    camera_param_rgb_.fx = camera_parameters_rgb.fx;
    camera_param_rgb_.fy = camera_parameters_rgb.fy;
    camera_param_rgb_.cx = camera_parameters_rgb.cx;
    camera_param_rgb_.cy = camera_parameters_rgb.cy;
    camera_param_rgb_.p1 = camera_parameters_rgb.p1;
    camera_param_rgb_.p2 = camera_parameters_rgb.p2;
    camera_param_rgb_.k1 = camera_parameters_rgb.k1;
    camera_param_rgb_.k2 = camera_parameters_rgb.k2;
    camera_param_rgb_.k3 = camera_parameters_rgb.k3;
    camera_param_rgb_.k4 = camera_parameters_rgb.k4;
    camera_param_rgb_.k5 = camera_parameters_rgb.k5;
    camera_param_rgb_.k6 = camera_parameters_rgb.k6;

    PsCameraExtrinsicParameters pCameraExtrinsicParameters;
    Ps2_GetCameraExtrinsicParameters(deviceHandle, sessionIndex, &pCameraExtrinsicParameters);
    std::vector<double> _rotation(std::begin(pCameraExtrinsicParameters.rotation),
                                  std::end(pCameraExtrinsicParameters.rotation));
    std::vector<double> _translation(
        std::begin(pCameraExtrinsicParameters.translation),
        std::end(pCameraExtrinsicParameters.translation));
    extrinsic_param_.rotation = _rotation;
    extrinsic_param_.translation = _translation;

    Ps2_SetDepthDistortionCorrectionEnabled(deviceHandle, sessionIndex, true);
    Ps2_SetIrDistortionCorrectionEnabled(deviceHandle, sessionIndex, true);
    Ps2_SetRGBDistortionCorrectionEnabled(deviceHandle, sessionIndex, true);

    Ps2_SetComputeRealDepthCorrectionEnabled(deviceHandle, sessionIndex, true);
    Ps2_SetSpatialFilterEnabled(deviceHandle, sessionIndex, true);
    Ps2_SetTimeFilterEnabled(deviceHandle, sessionIndex, true);

    status = Ps2_SetMapperEnabledRGBToDepth(deviceHandle, sessionIndex, false);
    if (status != PsRetOK)
    {
      cout << "PsSetMapperEnabledRGBToDepth failed!" << endl;
      exit(EXIT_FAILURE);
    }

    status = Ps2_SetMapperEnabledDepthToRGB(deviceHandle, sessionIndex, false);
    if (status != PsRetOK)
    {
      cout << "PsSetMapperEnabledDepthToRGB failed!" << endl;
      exit(EXIT_FAILURE);
    }

    status = Ps2_SetDataMode(deviceHandle, sessionIndex, (PsDataMode)PsDepthAndIR_15_RGB_30);
    if (status != PsReturnStatus::PsRetOK)
    {
      cout << "PsSetDataMode failed!" << endl;
      ;
    }
    Ps2_SetRGBResolution(deviceHandle, sessionIndex, PsRGB_Resolution_1920_1080);
    Ps2_SetColorPixelFormat(deviceHandle, sessionIndex, PsPixelFormatBGR888);

    status = Ps2_SetDepthRange(deviceHandle, sessionIndex, (PsDepthRange)PsNearRange);
    if (status != PsReturnStatus::PsRetOK)
    {
      cout << "PsSetDepthRange failed!" << endl;
      exit(EXIT_FAILURE);
    }
  }

  PicoZenseManager::~PicoZenseManager()
  {
    PsReturnStatus status;    
    status = Ps2_StopStream(deviceHandle, sessionIndex);
  	status = Ps2_CloseDevice(deviceHandle);
    if (status != PsReturnStatus::PsRetOK)
    {
      cout << "CloseDevice failed!" << endl;
    }else{
	    cout << "Device Closed: " << sessionIndex << endl;
    }
    status = Ps2_Shutdown();
    cout << "Shutdown status: " << status << endl;
  }

  bool PicoZenseManager::update()
  {
    bool isSuccess = false;
    PsReturnStatus status;

    PsFrameReady frameReady = {0};
    status = Ps2_ReadNextFrame(deviceHandle, sessionIndex, &frameReady);
    if (status != PsRetOK)
    {
      cout << "Could not read next frame from device " << sessionIndex << endl;
      return isSuccess;
    }

    // Depth
    if (1 == frameReady.depth) {    
      PsFrame depthFrame = {0};
      Ps2_GetFrame(deviceHandle, sessionIndex, PsDepthFrame, &depthFrame);
      if (depthFrame.pFrameData != NULL)
      {
        depthImg_ = cv::Mat(depthFrame.height, depthFrame.width, CV_16UC1,
                            depthFrame.pFrameData);
        isSuccess = true;
      }
    }

    // IR
    if (1 == frameReady.ir) {
      PsFrame irFrame = {0};
      Ps2_GetFrame(deviceHandle, sessionIndex, PsIRFrame, &irFrame);
      if (irFrame.pFrameData != NULL)
      {
        irImg_ =
            cv::Mat(irFrame.height, irFrame.width, CV_16UC1, irFrame.pFrameData);
        isSuccess = true;
      }
    }

    if (1 == frameReady.rgb) {
      PsFrame rgbFrame = {0};
      Ps2_GetFrame(deviceHandle, sessionIndex, PsRGBFrame, &rgbFrame);
      if (rgbFrame.pFrameData != NULL)
      {
        rgbImg_ =
            cv::Mat(rgbFrame.height, rgbFrame.width, CV_8UC3, rgbFrame.pFrameData);
        isSuccess = true;
      }
    }

    if (isSuccess)
    {
      if (depthImg_.rows == 0)
        isSuccess = false;
      if (irImg_.rows == 0)
        isSuccess = false;
      if (rgbImg_.rows == 0)
        isSuccess = false;
    }

    return isSuccess;
  }

  void PicoZenseManager::printCameraParams(CameraParameter camera_param)
  {
    std::cout << "Camera parameters(Depth):" << std::endl;
    std::cout << "fx = " << camera_param.fx << std::endl;
    std::cout << "fy = " << camera_param.fy << std::endl;
    std::cout << "cx = " << camera_param.cx << std::endl;
    std::cout << "cy = " << camera_param.cy << std::endl;
    std::cout << "p1 = " << camera_param.p1 << std::endl;
    std::cout << "p2 = " << camera_param.p2 << std::endl;
    std::cout << "k1 = " << camera_param.k1 << std::endl;
    std::cout << "k2 = " << camera_param.k2 << std::endl;
    std::cout << "k3 = " << camera_param.k3 << std::endl;
    std::cout << "k4 = " << camera_param.k4 << std::endl;
    std::cout << "k5 = " << camera_param.k5 << std::endl;
    std::cout << "k6 = " << camera_param.k6 << std::endl;
  }

  std::vector<std::vector<double>> PicoZenseManager::getExtrinsicParameter()
  {
    std::vector<std::vector<double>> extrinsic_parameter_vec;
    extrinsic_parameter_vec.push_back(extrinsic_param_.rotation);
    extrinsic_parameter_vec.push_back(extrinsic_param_.translation);
    return extrinsic_parameter_vec;
  }

  std::vector<double> PicoZenseManager::getCameraParameter()
  {
    std::vector<double> camera_parameter_vec;
    camera_parameter_vec.push_back(camera_param_.fx);
    camera_parameter_vec.push_back(camera_param_.fy);
    camera_parameter_vec.push_back(camera_param_.cx);
    camera_parameter_vec.push_back(camera_param_.cy);
    camera_parameter_vec.push_back(camera_param_.p1);
    camera_parameter_vec.push_back(camera_param_.p2);
    camera_parameter_vec.push_back(camera_param_.k1);
    camera_parameter_vec.push_back(camera_param_.k2);
    camera_parameter_vec.push_back(camera_param_.k3);
    camera_parameter_vec.push_back(camera_param_.k4);
    camera_parameter_vec.push_back(camera_param_.k5);
    camera_parameter_vec.push_back(camera_param_.k6);
    return camera_parameter_vec;
  }

  std::vector<double> PicoZenseManager::getRGBCameraParameter()
  {
    std::vector<double> camera_parameter_vec;
    camera_parameter_vec.push_back(camera_param_rgb_.fx);
    camera_parameter_vec.push_back(camera_param_rgb_.fy);
    camera_parameter_vec.push_back(camera_param_rgb_.cx);
    camera_parameter_vec.push_back(camera_param_rgb_.cy);
    camera_parameter_vec.push_back(camera_param_rgb_.p1);
    camera_parameter_vec.push_back(camera_param_rgb_.p2);
    camera_parameter_vec.push_back(camera_param_rgb_.k1);
    camera_parameter_vec.push_back(camera_param_rgb_.k2);
    camera_parameter_vec.push_back(camera_param_rgb_.k3);
    camera_parameter_vec.push_back(camera_param_rgb_.k4);
    camera_parameter_vec.push_back(camera_param_rgb_.k5);
    camera_parameter_vec.push_back(camera_param_rgb_.k6);
    return camera_parameter_vec;
  }

  std::string PicoZenseManager::getSerialNumber() { return serialNumber_; }

  cv::Mat PicoZenseManager::getIRImage() { return irImg_; }

  cv::Mat PicoZenseManager::getDepthImage() { return depthImg_; }

  cv::Mat PicoZenseManager::getRGBImage() { return rgbImg_; }

} // namespace zense
