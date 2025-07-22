#include <iostream>
#include <string>
#include "infer.h"          // YOLOv11 推理类
#include "BYTETracker.h"    // ByteTrack 跟踪器
#include <opencv2/opencv.hpp>
#include "serial.hpp"       // 串口通讯类

// 跟踪类别（COCO 索引），这里只跟踪 person
std::vector<int> trackClasses {0};  // 0 = person

bool isTrackingClass(int class_id){
    return std::find(trackClasses.begin(), trackClasses.end(), class_id) != trackClasses.end();
}

int main() {
    // GStreamer 管道：USB 摄像头（MJPG 格式）
    std::string gst_pipeline =
        "v4l2src device=/dev/video0 ! image/jpeg,format=MJPG,width=640,height=480,framerate=30/1 "
        "! nvv4l2decoder mjpeg=1 ! nvvidconv ! video/x-raw,format=BGRx "
        "! videoconvert ! video/x-raw,format=BGR ! appsink";

    cv::VideoCapture cap(gst_pipeline, cv::CAP_GSTREAMER);
    if (!cap.isOpened()) {
        std::cerr << "无法打开 USB 摄像头 GStreamer 流" << std::endl;
        return -1;
    }

    int img_w = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    int img_h = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    int fps = cap.get(cv::CAP_PROP_FPS);

    // 初始化 YOLOv11 推理器
    std::string engine_path = "/home/thunder/yolocpp/models/nice.plan";  // 替换为你的 engine 路径
    YoloDetector detector(engine_path, 0, 0.45, 0.01);  // device_id=0, conf_thresh=0.45, iou_thresh=0.01

    // 初始化 ByteTrack 跟踪器
    BYTETracker tracker(fps, 30);  // 允许丢失 30 帧

    cv::Mat frame;
    int frame_id = 0;
    int total_us = 0;

    SerialPort serial("/dev/ttyCH341USB0", 115200);//串口通讯
    if (!serial.openPort()) 
    {
        std::cerr << "串口打开失败！" << std::endl;
        return 0;
    }

    while (true) {
        cap >> frame;
        if (frame.empty()) break;
        frame_id++;

        auto start = std::chrono::system_clock::now();

        // YOLO 推理
        std::vector<Detection> detections = detector.inference(frame);

        // 筛选目标并转换为 ByteTrack 格式
        // std::vector<Object> objects;
        // for (const auto& det : detections) {
        //     if (!isTrackingClass(det.classId)) continue;
        //     cv::Rect_<float> rect(det.bbox[0], det.bbox[1], det.bbox[2] - det.bbox[0], det.bbox[3] - det.bbox[1]);
        //     objects.emplace_back(Object{rect, det.classId, det.conf});
        // }

        // // 跟踪
        // std::vector<STrack> tracks = tracker.update(objects);
        if (!detections.empty()) {
            // 寻找置信度最高的检测框
            const auto& best_det = *std::max_element(
                detections.begin(), detections.end(),
                [](const Detection& a, const Detection& b) {
                    return a.conf < b.conf;
                }
            );

            // 转换为跟踪对象
            cv::Rect_<float> rect(best_det.bbox[0], best_det.bbox[1],
                                best_det.bbox[2] - best_det.bbox[0],
                                best_det.bbox[3] - best_det.bbox[1]);
            std::vector<Object> objects{ Object{rect, best_det.classId, best_det.conf} };
            
            // 更新跟踪
            std::vector<STrack> tracks = tracker.update(objects);
            // 后续串口发送、绘图、坐标计算都只处理 tracks[0]
            if (!tracks.empty()) {
                auto tlwh = tracks[0].tlwh;
                float center_x = tlwh[0] + tlwh[2] / 2.0f;
                char buffer[64];
                int len = snprintf(buffer, sizeof(buffer), "C %.2f\n", center_x);
                serial.writeData(buffer, len);
            }
        }

        // for(const auto& track : tracks)
        // {
        //     auto tlwh = track.tlwh;
        //     center_x = tlwh[0] + tlwh[2] / 2.0f;
        //     //float center_y = tlwh[1] + tlwh[3] / 2;
        // }

        //显示帧率和目标数
        auto end = std::chrono::system_clock::now();
        //total_us += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        //int fps_now = frame_id * 1000000 / total_us;
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        int fps_now = (duration > 0) ? 1000 / duration : 0;
        
        // //绘制结果
        // for (const auto& track : tracks) {
        //     auto tlwh = track.tlwh;
        //     if (tlwh[2] * tlwh[3] > 20) {
        //         cv::Scalar color = tracker.get_color(track.track_id);
        //         cv::rectangle(frame, cv::Rect(tlwh[0], tlwh[1], tlwh[2], tlwh[3]), color, 2);
        //         cv::putText(frame, std::to_string(track.track_id), cv::Point(tlwh[0], tlwh[1] - 5),
        //                     cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);
        //     }
        // }

        // cv::putText(frame, cv::format("frame: %d fps: %d num: %ld", frame_id, fps_now, tracks.size()),
        //             cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);
        // cv::imshow("YOLOv11 + ByteTrack 实时跟踪", frame);
        // if (cv::waitKey(1) == 27) break;  // ESC 退出

        std::cout << "Processing frame " << frame_id << " (" << fps_now << " fps)" << " delay:" << duration << " ms" << std::endl;
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}
