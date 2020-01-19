#include "pico_zense_manager.hpp"

using namespace std;
namespace zense{
    PicoZenseManager::PicoZenseManager()
    {
        PsReturnStatus status;
        status = PsInitialize();
        if (status != PsReturnStatus::PsRetOK)
        {
            cout << "PsInitialize failed!" << endl;
            exit(EXIT_FAILURE);
        }

        int32_t deviceCount_ = 0;
        status = PsGetDeviceCount(&deviceCount_);
        if (status != PsReturnStatus::PsRetOK)
        {
            cout << "PsGetDeviceCount failed!" << endl;
            exit(EXIT_FAILURE);
        }
        cout << "Detected " << deviceCount_ << " devices." << endl;
        
        if (deviceCount_ > MAX_DEVICECOUNT)
        {
            cout << "# of devices exceeds maximum of " << MAX_DEVICECOUNT << endl;
            deviceCount_ = MAX_DEVICECOUNT;
        }

        status = PsOpenDevice(0);
        if (status != PsReturnStatus::PsRetOK)
        {
            cout << "PsOpenDevice failed!" << endl;
            exit(EXIT_FAILURE);
        }

        //get Serial Number
        int32_t lenSerial = 100;
        char buffSerial[lenSerial];
        status = PsGetProperty(0, PsPropertySN_Str, buffSerial, &lenSerial);
        serialNumber_ = buffSerial;
        cout << "SERIAL : " << buffSerial << endl;

        //set Depth Camera Parameter
        PsCameraParameters camera_parameters;
        status = PsGetCameraParameters(0, PsDepthSensor, &camera_parameters);

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

        status = PsSetDataMode(0, PsDepthAndRGB_30);
        if (status != PsReturnStatus::PsRetOK)
        {
            cout << "PsSetDataMode failed!" << endl;
            ;
        }        

        status = PsSetResolution(0, PsRGB_Resolution_1920_1080);

        status = PsSetDepthRange(0, PsNearRange);
        if (status != PsReturnStatus::PsRetOK)
        {
            cout << "PsSetDepthRange failed!" << endl;
            exit(EXIT_FAILURE);
        }

        PsSetDepthDistortionCorrectionEnabled(0, true);
        PsSetIrDistortionCorrectionEnabled(0, true);
        PsSetRGBDistortionCorrectionEnabled(0, true);

        PsSetFilter(0, PsComputeRealDepthFilter, true); // default : true
        PsSetFilter(0, PsSmoothingFilter, true); // default : true

        PsSetSpatialFilterEnabled(0, true); // default : true
        PsSetTimeFilterEnabled(0, true); // default : true

        PsSetColorPixelFormat(0, PsPixelFormatBGR888);

        status = PsSetMapperEnabledRGBToDepth(0, false);
        if (status != PsRetOK)
        {
            cout << "PsSetMapperEnabledRGBToDepth failed!" << endl;
            exit(EXIT_FAILURE);
        }

        status = PsSetMapperEnabledDepthToRGB(0, true);
        if (status != PsRetOK)
        {
            cout << "PsSetMapperEnabledDepthToRGB failed!" << endl;
            exit(EXIT_FAILURE);
        }
        
        status = PsSetSynchronizeEnabled(0, true);
    }

    PicoZenseManager::~PicoZenseManager()
    {
        PsReturnStatus status;
        status = PsShutdown();
        cout << "Shutdown : " << status << endl;
    }

    bool PicoZenseManager::updateDevice()
    {
        bool isSuccess = false;
        PsReturnStatus status;
        status = PsReadNextFrame(0);
        if (status != PsRetOK)
        {
            cout << "Could not read next frame from device " << 0 << endl;
            return isSuccess;
        }
        
        // Depth
        PsFrame depthFrame = {0};
        PsGetFrame(0, PsDepthFrame, &depthFrame);
        if (depthFrame.pFrameData != NULL)
        {
            depthImg_ = cv::Mat(
                depthFrame.height, depthFrame.width, CV_16UC1, depthFrame.pFrameData);
            isSuccess = true;
        }

        // RGB
        PsFrame rgbFrame = {0};
        //PsGetFrame(0, PsRGBFrame, &rgbFrame);
        PsGetFrame(0, PsMappedRGBFrame, &rgbFrame);
        if (rgbFrame.pFrameData != NULL)
        {
            rgbImg_ = cv::Mat(
                rgbFrame.height, rgbFrame.width, CV_8UC3, rgbFrame.pFrameData);
        }

        if (isSuccess)
        {
            if (depthImg_.rows == 0) isSuccess = false;
            if (rgbImg_.rows == 0) isSuccess = false;
        }

        return isSuccess;
    }


    void PicoZenseManager::printCameraParams(CameraParameter camera_param) {
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

    std::vector<double> PicoZenseManager::getCameraParameter(){
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

    std::string PicoZenseManager::getSerialNumber() {
        return serialNumber_;
    }

    cv::Mat PicoZenseManager::getRGBImage() {
        return rgbImg_;
    }

    cv::Mat PicoZenseManager::getDepthImage() {
        return depthImg_;
    }

}
