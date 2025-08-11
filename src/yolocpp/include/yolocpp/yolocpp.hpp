#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <opencv2/opencv.hpp>
#include "infer.h"
#include "BYTETracker.h"
#include <thread>
#include <atomic>
#include "serial/serial.h"
#include "PID.hpp"
#include <chrono>

class YoloCpp : public rclcpp::Node
{
public:
    YoloCpp();
    ~YoloCpp();
    void yolorun();

private:
    std::string gst_pipeline;
    cv::VideoCapture cap;
    std::string engine_path;//模型地址参数
    int fps;
    std::unique_ptr<YoloDetector> detector;
    std::unique_ptr<BYTETracker> tracker;
    cv::Mat frame;

    std::thread loop_thread;
    //std::atomic<bool> running;//线程运行
    std::unique_ptr<serial::Serial> my_serial;//串口
    std::string serial_port;//串口参数

    //系统延迟
    float system_delay;
    double kp, ki, kd, kf, alpha, omin, omax;//PID参数
    //PID对象指针
    std::unique_ptr<PIDFF> my_pid;
    //防止中心抖动的低通滤波器
    std::unique_ptr<LowPassFilter> lowpass;

    // 用于速度计算
    std::chrono::steady_clock::time_point last_pos_time_ = std::chrono::steady_clock::now();
    float last_center_x_ = 0.0f;
    bool last_center_x_valid_ = false;
};