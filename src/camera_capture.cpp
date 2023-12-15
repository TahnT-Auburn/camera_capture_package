//STD
#include <cstring>
#include <string>

//ROS2
#include "rmw/types.h"

//ArenaSDK
#include "camera_capture.h"
#include "rclcpp_adapter/pixelformat_translation.h"

void CameraCapture::parse_parameters_()
{
    //Declare ROS2 stream parameters
    this->declare_parameter<std::string>("serial", "");
    this->declare_parameter("pixelformat", "");
    this->declare_parameter("width", 0);
    this->declare_parameter("height", 0);
    this->declare_parameter("gain", 0.0);
    this->declare_parameter("exposure_time", 4000.0);
    this->declare_parameter("stream_auto_negotiate_packet_size", true);
    this->declare_parameter("stream_packet_resend_enable", true);

    //Declare ROS2 image capture parameters
    this->declare_parameter("image_save_path");
    this->declare_parameter("image_save_name");
    this->declare_parameter("image_save_ext");

    //Get ROS2 stream parameters
    this->get_parameter("serial", serial_);
    this->get_parameter("pixelformat", pixelformat_ros_);
    this->get_parameter("width", width_);
    this->get_parameter("height", height_);
    this->get_parameter("gain", gain_);
    this->get_parameter("exposure_time", exposure_time_);
    this->get_parameter("stream_auto_negotiate_packet_size", stream_auto_negotiate_packet_size_);
    this->get_parameter("stream_packet_resend_enable", stream_packet_resend_enable_);

    //Get ROS2 image capture parameters
    this->get_parameter("image_save_path", image_save_path_);
    this->get_parameter("image_save_name", image_save_name_);
    this->get_parameter("image_save_ext", image_save_ext_);
}

void CameraCapture::capture_()
{
    //Open system and retrieve device list
    Arena::ISystem* pSystem = Arena::OpenSystem();
    pSystem->UpdateDevices(100);
    std::vector<Arena::DeviceInfo> deviceInfos = pSystem->GetDevices();
    if (deviceInfos.size() == 0 )
    {
        RCLCPP_ERROR(this->get_logger(), "No camera(s) detected");
        rclcpp::shutdown();
    }
    
    //Log device info
    for (size_t i = 0; i < deviceInfos.size(); i++)
    {

        //Display device information
        GenICam::gcstring vendor = deviceInfos[i].VendorName();
        GenICam::gcstring model = deviceInfos[i].ModelName();
        GenICam::gcstring serial = deviceInfos[i].SerialNumber();
        GenICam::gcstring macStr = deviceInfos[i].MacAddressStr();
        GenICam::gcstring ipStr = deviceInfos[i].IpAddressStr();
        RCLCPP_INFO(this->get_logger(), "\nDevice %d:\n %s\n Model: %s\n Serial: %s\n Mac: %s\n IP: %s\n",
            i+1,
            vendor.c_str(),
            model.c_str(),
            serial.c_str(),
            macStr.c_str(),
            ipStr.c_str());
    }
    
    //Create device (one device)
    Arena::IDevice* pDevice = pSystem->CreateDevice(deviceInfos[0]);
    RCLCPP_INFO(this->get_logger(), "Device created\n");

    //Reset nodes to default to avoid issues when setting new nodes
    reset_nodes_(pDevice);

    //Set nodes
    RCLCPP_INFO(this->get_logger(), "Setting nodes ...");
    auto nodemap = pDevice->GetNodeMap();
    auto stream_nodemap = pDevice->GetTLStreamNodeMap();

    Arena::SetNodeValue<int64_t>(nodemap, "Width", width_);
    Arena::SetNodeValue<int64_t>(nodemap, "Height", height_);

    //Adjust offset from input width and height
    //TODO: Have Arena retrieve max values instead of setting them manually
    
    //Set offset if camera ROI is not max
    size_t width_max_ = 1936;
    size_t height_max_ = 1464;
    size_t offset_x_ = 0;
    size_t offset_y_ = 0;

    if (width_ != width_max_)
    {
        offset_x_ = (width_max_ - width_)/2;
        Arena::SetNodeValue<int64_t>(nodemap, "OffsetX", offset_x_);
    }
    
    
    if (height_ != height_max_)
    {
        offset_y_ = (height_max_ - height_)/2;
        Arena::SetNodeValue<int64_t>(nodemap, "OffsetY", offset_y_);
    }
    
    Arena::SetNodeValue<double>(nodemap, "Gain", gain_);
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "ExposureAuto", "Off");
    Arena::SetNodeValue<double>(nodemap, "ExposureTime", exposure_time_);
    pixelformat_pfnc_ = K_ROS2_PIXELFORMAT_TO_PFNC[pixelformat_ros_];
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "PixelFormat", pixelformat_pfnc_.c_str());
    Arena::SetNodeValue<bool>(stream_nodemap, "StreamAutoNegotiatePacketSize", stream_auto_negotiate_packet_size_);
    Arena::SetNodeValue<bool>(stream_nodemap, "StreamPacketResendEnable", stream_packet_resend_enable_);

    //Log params
    RCLCPP_INFO(this->get_logger(), "Image settings:");
    RCLCPP_INFO(this->get_logger(), "pixelformat: %s", pixelformat_ros_.c_str());
    RCLCPP_INFO(this->get_logger(), "width: %d", width_);
    RCLCPP_INFO(this->get_logger(), "height: %d", height_);
    RCLCPP_INFO(this->get_logger(), "OffsetX: %d", offset_x_);
    RCLCPP_INFO(this->get_logger(), "OffsetY: %d", offset_y_);
    RCLCPP_INFO(this->get_logger(), "gain: %f", gain_);
    RCLCPP_INFO(this->get_logger(), "exposure_time: %f", exposure_time_);
    RCLCPP_INFO(this->get_logger(), "stream_auto_negotiate_packet_size: %d", stream_auto_negotiate_packet_size_);
    RCLCPP_INFO(this->get_logger(), "stream_packet_resend_enable: %d", stream_packet_resend_enable_);

    //Start Stream
    pDevice->StartStream();

    int save_count_ = 0;
    while (rclcpp::ok())
    {

        //Get images from Arena SDK
        Arena::IImage *pImage = pDevice->GetImage(2000);

        //Convert to OpenCV image
        //TODO: Find a more efficient way to differentiate among
        //      different camera specs for coversion

        cv::Mat cvImage = cv::Mat(
        (int)pImage->GetHeight(),
        (int)pImage->GetWidth(),
        CV_8UC3,
        (void *)pImage->GetData());
        cv::cvtColor(cvImage, cvImage, cv::COLOR_BGR2RGB); //Color conversion

        //Display stream
        cv::imshow("OpenCV Stream", cvImage);
        RCLCPP_INFO_ONCE(this->get_logger(), "Streaming with OpenCV ...\n press 's' to save image\n press 'esc' to exit");
        char key = (char) cv::waitKey(1);
        
        //Save frame
        if (key == 's')
        {   
            //Generate write file path and name for cv::imwrite
            save_count_ ++;
            std::string str_count_ = std::to_string(save_count_);
            std::string write_file_ = image_save_path_ + image_save_name_ + str_count_ + image_save_ext_;
            cv::imwrite(write_file_, cvImage);
            RCLCPP_INFO(this->get_logger(), "%s image(s) saved to %s", 
            str_count_.c_str(),
            write_file_.c_str());
        }
        else if (key == 27)
        {   
            RCLCPP_INFO(this->get_logger(), "esc key pressed ...");
            break;
        }
        pDevice->RequeueBuffer(pImage);
    }
    //shutdown
    RCLCPP_INFO(this->get_logger(), "Shutting down");
    cv::destroyAllWindows();
    pDevice->StopStream();
    pSystem->DestroyDevice(pDevice);
    Arena::CloseSystem(pSystem);
}

void CameraCapture::reset_nodes_(Arena::IDevice* pDevice)
{
  auto nodemap = pDevice->GetNodeMap();
  // device run on default profile all the time if no args are passed
  // otherwise, overwise only these params
  Arena::SetNodeValue<GenICam::gcstring>(nodemap, "UserSetSelector", "Default");
  // execute the profile
  Arena::ExecuteNode(nodemap, "UserSetLoad");
  RCLCPP_INFO(this->get_logger(),"Default profile is loaded\n");
}