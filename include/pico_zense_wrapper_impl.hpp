#pragma once
#include <chrono>
#include <unordered_map>

#include "common.hpp"
#include "parameter_manager.hpp"
#include "pico_zense_manager.hpp"

#define INIT_SKIP_COUNTER -200
#define MAX_SKIP_COUNTER 60
#define MAX_HEARTBEAT_COUNTER 10

namespace zense {

enum DepthRange { Near, Mid, Far };

typedef std::vector<DepthRange> WDRDepthRange;

// not_WDR is just convenience notification for getDepthRange()
enum ZenseMode { RGBD, RGBDIR, RGBIR, DepthIR, WDR }; // Currently RGBDIR mode is disable by manufacture

class PicoZenseWrapperImpl {
 public:
  ~PicoZenseWrapperImpl() { close(); }

  void setup(std::string cfgParamPath, std::string camKey,
             int32_t device_index__);
  void setup(int32_t device_index__);
  void setup(std::string uri_string);
  bool isWithinError(float val, float ref) {
    int32_t depth_range = manager_.getDepthRange();
    const double fract_err = 1e-5;
    return (std::fabs(val - ref) <= fract_err * std::fabs(ref));
  };
  bool setDeviceMode(const int32_t & range1_to_set, const int32_t &range2_to_set, const int32_t &rgb_setting);
  bool setDeviceModeFromConfig(std::string cfgParamPath, std::string camKey);
  void close();
  bool update();

  std::string getSerialNumber() { return serial_no_; };
  std::vector<double> getCameraParameter();
  std::vector<std::vector<double>> getExtrinsicParameter();
  std::vector<double> getRGBCameraParameter();

  cv::Mat getRGBImage() { return rgb_image; };
  cv::Mat getIRImage() { return ir_image; };
  cv::Mat getDepthImage() { return depth_image_range1; };
  std::vector<cv::Mat> getWDRDepthImage() {
    return std::vector<cv::Mat>{depth_image_range1, depth_image_range2};
  }

  // template<typename range_output_type>
  // range_output_type getDepthRange(){ throw std::runtime_error("Undefined Type
  // : getDepthRange()");}; // template specification later : DepthRange,
  // WDRDepthRange

  int getDepthRange();
  std::vector<int> getDepthRangeWDR();

  bool getPulseCount(uint32_t &pulseCount) {
    return manager_.getPulseCount(pulseCount);
  }
  bool setPulseCount(uint32_t pulseCount) {
    return manager_.setPulseCount(pulseCount);
  }
  bool getPulseCountWDR(uint32_t &pulse_count_range1, uint32_t &pulse_count_range2);
  bool setPulseCountWDR(uint32_t pulse_count_range1, uint32_t pulse_count_range2);
  bool setDepthRange(std::string given_range);

  bool is_rgb() { return isRGB; };
  bool is_ir() { return isIR; };
  bool is_wdr() { return isWDR; };
  bool is_rgbir() { return isRGBIR; };

 private:
  // PicoZense custom API
  PicoZenseManager manager_;

  int range1;
  int range2;
  int skip_counter_[3] = {INIT_SKIP_COUNTER, INIT_SKIP_COUNTER,
                          INIT_SKIP_COUNTER};
  bool flag_wdr_range_updated_[3] = {
      false, false, false};  // assumed near = 0, mid = 1, far = 2

  int32_t device_index_;
  std::string sensor_id_;
  std::string serial_no_;
  bool isRGB, isWDR, isIR, isRGBIR;

  cv::Mat rgb_image;
  cv::Mat ir_image;
  cv::Mat depth_image_range1;
  cv::Mat depth_image_range2;
  DepthRange depth_range1;
  DepthRange depth_range2;

  CameraParameter camera_param_;
  CameraParameter camera_param_rgb_;
  ExtrinsicParameter extrinsic_param_;

  // For template speciallization, defined actual process is written in .cpp
  bool monitoring_skip();
  template <ZenseMode T>
  bool _update() {
    throw std::runtime_error("Undefined Type : _update");
  };
};

}  // namespace zense