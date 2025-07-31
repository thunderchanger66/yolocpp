#include "yolocpp/yolocpp.hpp"
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include "infer.h"
#include "BYTETracker.h"
#include <chrono>
#include <vector>
#include <string>

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

    //running = true;
    //启动循环线程
    loop_thread = std::thread(&YoloCpp::yolorun, this);
}

YoloCpp::~YoloCpp()
{
    //running = false;
    if(loop_thread.joinable())
        loop_thread.join();

    cap.release();
    
    RCLCPP_INFO(this->get_logger(), "YoloCpp shutdown!");
}

void YoloCpp::yolorun()
{
    while(rclcpp::ok())
    {
        cap >> frame;
        if(frame.empty()) break;

        auto start = std::chrono::system_clock::now();

        //YOLO推理
        std::vector<Detection> detections = detector->inference(frame);

        //跟踪
        std::vector<Object> objects;
        if(!detections.empty())
        {
            // 寻找置信度最高的检测框
            const auto& best_det = *std::max_element(
                detections.begin(), detections.end(),
                [](const Detection& a, const Detection& b) {
                    return a.conf < b.conf;
                }
            );

            //转换为跟踪对象
            cv::Rect_<float> rect(best_det.bbox[0], best_det.bbox[1],
                                best_det.bbox[2] - best_det.bbox[0],
                                best_det.bbox[3] - best_det.bbox[1]);
            objects.emplace_back(Object{rect, best_det.classId, best_det.conf});
        }
        //更新追踪
        std::vector<STrack> tracks = tracker->update(objects);
        // 后续串口发送、绘图、坐标计算都只处理 tracks[0]
        if(!tracks.empty())
        {
            auto tlwh = tracks[0].tlwh;
            float center_x = tlwh[0] + tlwh[2] / 2.0f;
            float center_y = tlwh[1] + tlwh[3] / 2.0f;
            // char buffer[64];
            // int len = snprintf(buffer, sizeof(buffer), "C %.2f, %.2f", center_x, center_y);
            //发送
        }

        //显示帧率
        auto end = std::chrono::system_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        RCLCPP_INFO(this->get_logger(), "Delay: %ldms", duration);
    }
}