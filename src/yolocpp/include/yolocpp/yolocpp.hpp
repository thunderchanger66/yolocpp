#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <opencv2/opencv.hpp>
#include "infer.h"
#include "BYTETracker.h"

class YoloCpp : public rclcpp::Node
{
public:
    YoloCpp();

private:
    std::string gst_pipeline;
    cv::VideoCapture cap;
    std::string engine_path;
    int fps;
    std::unique_ptr<YoloDetector> detector;
    std::unique_ptr<BYTETracker> tracker;
    cv::Mat frame;
};