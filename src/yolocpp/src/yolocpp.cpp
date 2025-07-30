#include "yolocpp/yolocpp.hpp"
#include <rclcpp/rclcpp.hpp>

YoloCpp::YoloCpp() : rclcpp::Node("yolo_cpp")
{
    RCLCPP_INFO(this->get_logger(), "Create YoloCPP OK!");

    gst_pipeline = 
        "v4l2src device=/dev/video0 ! image/jpeg,format=MJPG,width=640,height=480,framerate=30/1 "
        "! nvv4l2decoder mjpeg=1 ! nvvidconv ! video/x-raw,format=BGRx "
        "! videoconvert ! video/x-raw,format=BGR ! appsink";

}