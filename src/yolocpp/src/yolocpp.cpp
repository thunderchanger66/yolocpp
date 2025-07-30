#include "yolocpp/yolocpp.hpp"
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include "infer.h"
#include "BYTETracker.h"

YoloCpp::YoloCpp() : rclcpp::Node("yolo_cpp")
{
    RCLCPP_INFO(this->get_logger(), "Create YoloCPP OK!");

    //模型的地址，传参
    this->declare_parameter("engine_path", "/home/thunder/yolocpp/models/com.plan");
    this->get_parameter("engine_path", engine_path);

    gst_pipeline = 
        "v4l2src device=/dev/video0 ! image/jpeg,format=MJPG,width=640,height=480,framerate=30/1 "
        "! nvv4l2decoder mjpeg=1 ! nvvidconv ! video/x-raw,format=BGRx "
        "! videoconvert ! video/x-raw,format=BGR ! appsink";

    cap.open(gst_pipeline, cv::CAP_GSTREAMER);
    if(!cap.isOpened())
    {
        std::cerr << "Cannot open camera!" << std::endl;
        return;
    }
    fps = cap.get(cv::CAP_PROP_FPS);

    //初始化YOLOv11推理器
    detector = std::make_unique<YoloDetector>(engine_path, 0, 0.45, 0.5);// device_id=0, conf_thresh=0.45, iou_thresh=0.5
    //初始化ByteTrack跟踪器
    tracker = std::make_unique<BYTETracker>(fps, 30);

}