#include "pico_zense_wrapper_impl.hpp"

namespace zense {

void PicoZenseWrapperImpl::setup(std::string cfgParamPath, std::string camKey,
                                int32_t device_index__) {
  device_index_ = device_index__;
  usleep(5 * 1e6);  // To avoid high frequent sensor open call from
                    // immediately after termination and rebooting

  if (!checkFileExistence(cfgParamPath)) {
    std::cerr << "Setting TOML file does not exist" << std::endl;
    std::exit(EXIT_FAILURE);
  }
  ParameterManager cfgParam(cfgParamPath);

  sensor_id_ = cfgParam.readStringData("General", "sensor_id");

  if (!cfgParam.checkExistanceTable(camKey.c_str())) {
    std::cerr << "Camera name is invalid, please check setting toml file"
              << std::endl;
    std::exit(EXIT_FAILURE);
  }

  std::string camera_name =
      cfgParam.readStringData(camKey.c_str(), "camera_name");
  serial_no_ = cfgParam.readStringData(camKey.c_str(), "serial_no");
  range1 = cfgParam.readIntData(camKey.c_str(), "range1");
  range2 = cfgParam.readIntData(camKey.c_str(), "range2");
  isRGB = cfgParam.readIntData(camKey.c_str(), "rgb_image") == 1;
  isWDR = (range1 >= 0) && (range2 >= 0);
  isIR = isRGB && !isWDR;
  // TODO: merge factory values and tuned values
  std::string camFactKey = camKey + "_Factory";

  CameraParameter camera_factory_param;
  camera_factory_param.fx = cfgParam.readFloatData(camFactKey.c_str(), "fx");
  camera_factory_param.fy = cfgParam.readFloatData(camFactKey.c_str(), "fy");
  camera_factory_param.cx = cfgParam.readFloatData(camFactKey.c_str(), "cx");
  camera_factory_param.cy = cfgParam.readFloatData(camFactKey.c_str(), "cy");
  camera_factory_param.p1 = cfgParam.readFloatData(camFactKey.c_str(), "p1");
  camera_factory_param.p2 = cfgParam.readFloatData(camFactKey.c_str(), "p2");
  camera_factory_param.k1 = cfgParam.readFloatData(camFactKey.c_str(), "k1");
  camera_factory_param.k2 = cfgParam.readFloatData(camFactKey.c_str(), "k2");
  camera_factory_param.k3 = cfgParam.readFloatData(camFactKey.c_str(), "k3");
  camera_factory_param.k4 = cfgParam.readFloatData(camFactKey.c_str(), "k4");
  camera_factory_param.k5 = cfgParam.readFloatData(camFactKey.c_str(), "k5");
  camera_factory_param.k6 = cfgParam.readFloatData(camFactKey.c_str(), "k6");

  std::string id_str;
  std::cout << "Serial number allocated to Zense Manager : " << serial_no_
            << std::endl;

  std::string distortionKey = camKey + "_Undistortion";

  // If TOML configuration file doesn't contain any undistortion
  // description(table), undistortion process will be skip
  if (cfgParam.checkExistanceTable(distortionKey)) {
    undistortion_flag = cfgParam.readBoolData(distortionKey, "flag");
  } else {
    undistortion_flag = false;
  }
  if (undistortion_flag == true) {
    undistorter = PicoZenseUndistorter(cfgParam, distortionKey);
  }

  manager_.openDevice(device_index_);
  if (!manager_.setupDevice(device_index_, range1, range2, isRGB)) {
    close();
    std::cerr << "Could not setup device" << std::endl;
    std::exit(EXIT_FAILURE);
  }
  if (range2 < 0) range2 = range1;

  camera_param_ = manager_.getCameraParameter(device_index_, 0);
  if (!(isWithinError(camera_param_.k5, camera_factory_param.k5) &&
        isWithinError(camera_param_.k6, camera_factory_param.k6))) {
    close();
    std::cerr << "Erroneous internal parameters. Exiting..." << std::endl;
    std::exit(EXIT_FAILURE);
  }

  if (!manager_.startDevice(device_index_)) {
    close();
    std::cerr << "Could not start device" << std::endl;
    std::exit(EXIT_FAILURE);
  }

  std::cout << "Camera setup is finished!" << std::endl;
}

void PicoZenseWrapperImpl::setup(int32_t device_index__) {
  device_index_ = device_index__;
  usleep(5 * 1e6);  // To avoid high frequent sensor open call from
                    // immediately after termination and rebooting
  range1 = 0;
  range2 = -1;
  isRGB = 1;
  isWDR = (range1 >= 0) && (range2 >= 0);
  isIR = isRGB && !isWDR;

  manager_.openDevice(device_index_);
  if (!manager_.setupDevice(device_index_, range1, range2, isRGB)) {
    close();
    std::cerr << "Could not setup device" << std::endl;
    std::exit(EXIT_FAILURE);
  }
  if (range2 < 0) range2 = range1;

  std::string camera_name = "Camera0";
  int32_t lenSerial = 100;
  char buffSerial[lenSerial];
  PsReturnStatus status;
  status = PsGetProperty(device_index_, PsPropertySN_Str, buffSerial, &lenSerial);
  if (status != PsReturnStatus::PsRetOK) {
    std::cout << "Aquisition of Device Serial Number failed !" << std::endl;
  }
  serial_no_ = std::string(buffSerial);
  std::cout << "Serial number allocated to Zense Manager : " << serial_no_
            << std::endl;

  //TODO: rewrite RGB flag expricitly
  camera_param_ = manager_.getCameraParameter(device_index_, 0);
  camera_param_rgb_ = manager_.getCameraParameter(device_index_, 1);
  extrinsic_param_ = manager_.getExtrinsicParameter(device_index_);

  if (!manager_.startDevice(device_index_)) {
    close();
    std::cerr << "Could not start device" << std::endl;
    std::exit(EXIT_FAILURE);
  }

  std::cout << "Camera setup is finished!" << std::endl;
}

void PicoZenseWrapperImpl::setup_debug(
      int32_t device_index__,
      bool EnableDepthDistCorrection,
      bool EnableIRDistCorrection,
      bool EnableRGBDistCorrection,
      bool EnableComputeRealDepthFilter,
      bool EnableSmoothingFilter,
      bool EnableTimeFilter,
      bool EnableSpatialFilter,
      bool EnabledRGBToDepth,
      bool EnabledDepth2RGB
    )
{
  device_index_ = device_index__;
  usleep(5 * 1e6);  // To avoid high frequent sensor open call from
                    // immediately after termination and rebooting
  range1 = 0;
  range2 = -1;
  isRGB = 1;
  isWDR = (range1 >= 0) && (range2 >= 0);
  isIR = isRGB && !isWDR;

  manager_.openDevice(device_index_);
  if (!manager_.setupDeviceDebug(
      device_index_, range1, range2, isRGB,
      EnableDepthDistCorrection,
      EnableIRDistCorrection,
      EnableRGBDistCorrection,
      EnableComputeRealDepthFilter,
      EnableSmoothingFilter,
      EnableTimeFilter,
      EnableSpatialFilter,
      EnabledRGBToDepth,
      EnabledDepth2RGB
    )) {
    close();
    std::cerr << "Could not setup device" << std::endl;
    std::exit(EXIT_FAILURE);
  }
  if (range2 < 0) range2 = range1;

  std::string camera_name = "Camera0";
  int32_t lenSerial = 100;
  char buffSerial[lenSerial];
  PsReturnStatus status;
  status = PsGetProperty(device_index_, PsPropertySN_Str, buffSerial, &lenSerial);
  if (status != PsReturnStatus::PsRetOK) {
    std::cout << "Aquisition of Device Serial Number failed !" << std::endl;
  }
  serial_no_ = std::string(buffSerial);
  std::cout << "Serial number allocated to Zense Manager : " << serial_no_
            << std::endl;

  //TODO: rewrite RGB flag expricitly
  camera_param_ = manager_.getCameraParameter(device_index_, 0);
  camera_param_rgb_ = manager_.getCameraParameter(device_index_, 1);
  extrinsic_param_ = manager_.getExtrinsicParameter(device_index_);

  if(!is_started){
    if (!manager_.startDevice(device_index_)) {
      close();
      std::cerr << "Could not start device" << std::endl;
      std::exit(EXIT_FAILURE);
    }
    is_started = true;
  }

  std::cout << "Camera setup is finished!" << std::endl;
}

void PicoZenseWrapperImpl::close() { manager_.closeDevice(device_index_); }

int PicoZenseWrapperImpl::getDepthRange() { return depth_range1; }

std::vector<int> PicoZenseWrapperImpl::getDepthRangeWDR() {
  return std::vector<int>{(int)depth_range1, (int)depth_range2};
}

bool PicoZenseWrapperImpl::monitoring_skip() {
  skip_counter_[range1]++;
  skip_counter_[range2]++;
  if (skip_counter_[range1] > MAX_SKIP_COUNTER ||
      skip_counter_[range2] > MAX_SKIP_COUNTER) {
    close();
    std::cerr << "Device not responding. Exiting..." << std::endl;
    std::exit(EXIT_FAILURE);
  }

  if (!manager_.updateDevice(device_index_)) {
    std::cout << "Device not updated. Skipping..." << std::endl;
    usleep(33333);
    return false;
  }
  return true;
}

template <>
bool PicoZenseWrapperImpl::_update<ZenseMode::RGBD>() {
  bool is_success = true;
  if (!monitoring_skip()) return false;
  rgb_image = manager_.getRgbImage(device_index_).clone();
  depth_range1 = (DepthRange)manager_.getDepthRange(device_index_);
  depth_image_range1 = manager_.getDepthImage(device_index_).clone();
  if (rgb_image.cols == 0 || depth_image_range1.cols == 0) is_success = false;
  skip_counter_[depth_range1] = 0;
  return is_success;
}

template <>
bool PicoZenseWrapperImpl::_update<ZenseMode::RGBDIR>() {
  bool is_success = true;
  if (!monitoring_skip()) return false;
  rgb_image = manager_.getRgbImage(device_index_).clone();
  ir_image = manager_.getIRImage(device_index_).clone();
  depth_image_range1 = manager_.getDepthImage(device_index_).clone();
  depth_range1 = (DepthRange)manager_.getDepthRange(device_index_);
  flag_wdr_range_updated_[depth_range1];
  if (is_success && (ir_image.cols == 0 || depth_image_range1.cols == 0))
    is_success = false;
  skip_counter_[depth_range1] = 0;
  return is_success;
}

template <>
bool PicoZenseWrapperImpl::_update<ZenseMode::WDR>() {
  bool is_success = true;
  if (!monitoring_skip()) return false;
  DepthRange _depth_range = (DepthRange)manager_.getDepthRange(device_index_);
  cv::Mat _depth_image = manager_.getDepthImage(device_index_).clone();
  if (_depth_range == range1) {
    depth_range1 = _depth_range;
    depth_image_range1 = _depth_image;
  } else if (_depth_range == range2) {
    depth_range2 = _depth_range;
    depth_image_range2 = _depth_image;
  } else {
    throw std::runtime_error("Unconfigurated depth informaiton aquired");
  }

  if (_depth_image.cols == 0) {
    is_success = false;
  } else {
    flag_wdr_range_updated_[_depth_range] = true;
  }
  skip_counter_[_depth_range] = 0;

  // if only double range depth image are updated, return true
  bool is_success_wdr;
  is_success_wdr =
      flag_wdr_range_updated_[range1] && flag_wdr_range_updated_[range2];
  if (is_success_wdr) {
    // if double depth info is correctly updated, reflesh
    flag_wdr_range_updated_[range1] = false;
    flag_wdr_range_updated_[range2] = false;
  }

  if (is_success_wdr && !is_success) {
    throw "Undefined situation";
  }

  return is_success_wdr;
}

bool PicoZenseWrapperImpl::update() {
  // ToDo: Check coverage
  bool status = false;

  while (!status) {
    if (isRGB) {
      if (isIR) {
        status = _update<ZenseMode::RGBDIR>();
      } else {
        status = _update<ZenseMode::RGBD>();
      }
    } else {
      if (isWDR) {
        status = _update<ZenseMode::WDR>();
      } else {
        status = _update<ZenseMode::DepthIR>();
      }
    }
  }

  return status;
}

bool PicoZenseWrapperImpl::setDepthRange(std::string given_range) {
  if (!isWDR) {
    bool status = manager_.setDepthRange(device_index_, given_range);
    range1 = manager_.getDepthRange(device_index_);
    range2 = range1;
  } else {
    std::cout << "Currently depth range change function supports not WDR mode"
              << std::endl;
  }
}

std::vector<double> PicoZenseWrapperImpl::getCameraParameter() {
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

std::vector<double> PicoZenseWrapperImpl::getRGBCameraParameter() {
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

std::vector<std::vector<double>> PicoZenseWrapperImpl::getExtrinsicParameter() {
  std::vector<std::vector<double>> extrinsic_parameter_vec;
  extrinsic_parameter_vec.push_back(extrinsic_param_.rotation);
  extrinsic_parameter_vec.push_back(extrinsic_param_.translation);
  return extrinsic_parameter_vec;
}

}  // namespace zense