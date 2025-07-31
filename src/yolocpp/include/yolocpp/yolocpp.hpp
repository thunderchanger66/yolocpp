#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <opencv2/opencv.hpp>
#include "infer.h"
#include "BYTETracker.h"
#include <thread>
#include <atomic>

class YoloCpp : public rclcpp::Node
{
public:
    YoloCpp();
    ~YoloCpp();
    void yolorun();

private:
    std::string gst_pipeline;
    cv::VideoCapture cap;
    std::string engine_path;
    int fps;
    std::unique_ptr<YoloDetector> detector;
    std::unique_ptr<BYTETracker> tracker;
    cv::Mat frame;

    std::thread loop_thread;
    //std::atomic<bool> running;//线程运行
};