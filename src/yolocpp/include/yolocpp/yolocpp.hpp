#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <opencv2/opencv.hpp>

class YoloCpp : public rclcpp::Node
{
public:
    YoloCpp();

private:
    std::string gst_pipeline;
    cv::VideoCapture cap;
};