#pragma once

//STD
#include <chrono>
#include <functional>

//ROS2
#include <rclcpp/rclcpp.hpp>

//OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

//ArenaSDK
#include "ArenaApi.h"

//TODO:
    /*
    CameraCapture will only allow user to start stream and save images
    but not calibrate the camera without launching the calibration node
    You must also relaunch the node and reconfigure if capturing images 
    for a new camera. Find someway to make this process more automated. 
    Start with figuring out how to create multiple device streams through
    ArenaSDK!
    */

class CameraCapture : public rclcpp::Node
{
public:
    CameraCapture()
    : Node("camera_capture")
    {   
        parse_parameters_();
        capture_();
    }

private:

    //Initialize ROS2 stream parameters
    std::string serial_;
    size_t width_;
    size_t height_;
    double gain_;
    double exposure_time_;
    std::string pixelformat_ros_, pixelformat_pfnc_;
    bool stream_auto_negotiate_packet_size_;
    bool stream_packet_resend_enable_;

    //Initialize ROS2 capture parameters
    std::string image_save_path_,
                image_save_name_,
                image_save_ext_;

    //Class Methods
    void parse_parameters_();
    void capture_();
    void reset_nodes_(Arena::IDevice* pDevice);
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraCapture>();
    rclcpp::shutdown();
    return 0;
}