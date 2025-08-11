#include "yolocpp/yolocpp.hpp"
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include "infer.h"
#include "BYTETracker.h"
#include <chrono>
#include <vector>
#include <string>
#include "serial/serial.h"
#include <mutex>

// ===================== PID + 前馈 + 滤波 + 延迟补偿 工具类 =====================
class LowPassFilter {
public:
    LowPassFilter(float alpha = 0.7f) : alpha_(alpha), initialized_(false), state_(0.0f) {}
    float filter(float x) {
        if(!initialized_) { state_ = x; initialized_ = true; return state_; }
        state_ = alpha_ * x + (1.0f - alpha_) * state_;
        return state_;
    }
    void reset() { initialized_ = false; }
    void setAlpha(float a) { alpha_ = a; }
private:
    float alpha_;
    bool initialized_;
    float state_;
};

class PIDFFController {
public:
    PIDFFController(float kp = 0.1f, float ki = 0.0f, float kd = 0.0f, float kf = 0.0f,
                    float min_out = -100.0f, float max_out = 100.0f)
        : kp_(kp), ki_(ki), kd_(kd), kf_(kf),
          min_out_(min_out), max_out_(max_out),
          prev_error_(0.0f), integral_(0.0f), prev_measurement_(0.0f), prev_setpoint_(0.0f), first_(true) {}

    float compute(float setpoint, float measurement, float measured_velocity, float dt, float system_delay) {
        if(dt <= 0.0f) return 0.0f;

        // 延迟补偿：预测测量在系统延迟后的值（线性预测）
        // 这里用测量速度（像素/s）乘延迟进行预测
        float predicted_measurement = measurement + measured_velocity * system_delay;

        // 误差基于预测值（或者你也可以基于预测 setpoint）
        float error = setpoint - predicted_measurement;

        // 积分
        integral_ += error * dt;

        // 微分（用测量值差分来降低噪声使用测量值的负导数）
        float derivative = 0.0f;
        if(!first_) derivative = -(predicted_measurement - prev_measurement_) / dt;

        float pid_out = kp_ * error + ki_ * integral_ + kd_ * derivative;

        // 前馈：使用目标速率（这里用 setpoint 变化率近似）
        // 前馈目的：当目标在移动时，直接施加额外控制以减少误差
        float setpoint_velocity = (setpoint - prev_setpoint_) / dt; // 像素/s
        float ff = kf_ * setpoint_velocity;

        float out = pid_out + ff;
        out = std::clamp(out, min_out_, max_out_);

        // 保存历史
        prev_error_ = error;
        prev_measurement_ = predicted_measurement;
        prev_setpoint_ = setpoint;
        first_ = false;

        return out;
    }

    void reset() {
        integral_ = 0.0f; prev_error_ = 0.0f; prev_measurement_ = 0.0f; prev_setpoint_ = 0.0f; first_ = true;
    }

    void setGains(float kp, float ki, float kd, float kf) { kp_ = kp; ki_ = ki; kd_ = kd; kf_ = kf; }
    void setOutputLimits(float lo, float hi) { min_out_ = lo; max_out_ = hi; }

private:
    float kp_, ki_, kd_, kf_;
    float min_out_, max_out_;
    float prev_error_, integral_, prev_measurement_, prev_setpoint_;
    bool first_;
};

// ===================== 集成到 ROS2 节点 =====================
class YoloCppNode : public rclcpp::Node {
public:
    YoloCppNode()
    : Node("yolo_cpp_pid_ff")
    {
        RCLCPP_INFO(this->get_logger(), "Create YoloCPP with PID+FF OK!");

        // ---------- ROS 参数 ----------
        this->declare_parameter<std::string>("engine_path", "/home/thunder/yolocpp/models/com.plan");
        this->declare_parameter<std::string>("serial_port", "/dev/ttyCH341USB0");
        this->declare_parameter<double>("kp", 0.15);
        this->declare_parameter<double>("ki", 0.0);
        this->declare_parameter<double>("kd", 0.02);
        this->declare_parameter<double>("kf", 0.05);
        this->declare_parameter<double>("filter_alpha", 0.6); // 低通滤波系数
        this->declare_parameter<double>("system_delay", 0.05); // 估计系统总延迟 (s)
        this->declare_parameter<double>("control_rate", 30.0); // 控制环频率 (Hz)
        this->declare_parameter<double>("output_min", -100.0);
        this->declare_parameter<double>("output_max", 100.0);

        this->get_parameter("engine_path", engine_path_);
        this->get_parameter("serial_port", serial_port_);

        double kp, ki, kd, kf, alpha, delay, rate, omin, omax;
        this->get_parameter("kp", kp);
        this->get_parameter("ki", ki);
        this->get_parameter("kd", kd);
        this->get_parameter("kf", kf);
        this->get_parameter("filter_alpha", alpha);
        this->get_parameter("system_delay", delay);
        this->get_parameter("control_rate", rate);
        this->get_parameter("output_min", omin);
        this->get_parameter("output_max", omax);

        pid_.setGains(kp, ki, kd, kf);
        pid_.setOutputLimits((float)omin, (float)omax);
        filter_.setAlpha((float)alpha);
        system_delay_ = (float)delay;
        control_rate_ = rate;

        gst_pipeline = 
            "v4l2src device=/dev/video0 ! image/jpeg,format=MJPG,width=640,height=480,framerate=30/1 "
            "! nvv4l2decoder mjpeg=1 ! nvvidconv ! video/x-raw,format=BGRx "
            "! videoconvert ! video/x-raw,format=BGR ! appsink";

        cap.open(gst_pipeline, cv::CAP_GSTREAMER);
        if(!cap.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Cannot open camera!");
            throw std::runtime_error("Cannot open camera");
        }

        fps_ = cap.get(cv::CAP_PROP_FPS);
        if(fps_ <= 0) fps_ = 30.0;

        // 初始化 detector && tracker (使用你已有的类)
        detector = std::make_unique<YoloDetector>(engine_path_, 0, 0.45, 0.5);
        tracker = std::make_unique<BYTETracker>(fps_, 30);

        // 初始化串口
        my_serial = std::make_unique<serial::Serial>(serial_port_, 115200, serial::Timeout::simpleTimeout(1000));
        if(!my_serial->isOpen()) {
            RCLCPP_ERROR(this->get_logger(), "Serial port open failed!");
            // 仍然允许运行，但不发送
            serial_ok_ = false;
        } else serial_ok_ = true;

        // 启动摄像头处理线程
        cam_thread_ = std::thread(&YoloCppNode::yolorun, this);

        // 启动固定频率的控制定时器（在 ROS2 线程上下文中）
        auto period = std::chrono::milliseconds((int)(1000.0 / control_rate_));
        control_timer_ = this->create_wall_timer(period, std::bind(&YoloCppNode::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "Node initialized: fps=%.1f, control_rate=%.1f Hz", fps_, control_rate_);
    }

    ~YoloCppNode() override {
        rclcpp::shutdown();
        if(cam_thread_.joinable()) cam_thread_.join();
        if(my_serial && my_serial->isOpen()) my_serial->close();
    }

private:
    // 摄像头和检测线程：只做检测与跟踪，更新共享的目标信息
    void yolorun() {
        cv::Mat frame;
        rclcpp::Time last_time = this->now();
        while(rclcpp::ok()) {
            cap >> frame;
            if(frame.empty()) break;

            auto start = std::chrono::steady_clock::now();

            // YOLO 推理获得检测
            std::vector<Detection> detections = detector->inference(frame);

            // 选择置信度最高的检测并转换为跟踪对象
            std::vector<Object> objects;
            if(!detections.empty()) {
                const auto& best_det = *std::max_element(detections.begin(), detections.end(),
                    [](const Detection& a, const Detection& b){ return a.conf < b.conf; });

                cv::Rect_<float> rect(best_det.bbox[0], best_det.bbox[1],
                                      best_det.bbox[2] - best_det.bbox[0],
                                      best_det.bbox[3] - best_det.bbox[1]);
                objects.emplace_back(Object{rect, best_det.classId, best_det.conf});
            }

            // 更新跟踪
            std::vector<STrack> tracks = tracker->update(objects);

            // 只取 tracks[0] 作为目标
            if(!tracks.empty()) {
                auto tlwh = tracks[0].tlwh;
                float center_x = tlwh[0] + tlwh[2] / 2.0f;
                float center_y = tlwh[1] + tlwh[3] / 2.0f;

                // 计算速度（像素/秒），需要时间间隔
                auto now = std::chrono::steady_clock::now();
                double dt = std::chrono::duration_cast<std::chrono::duration<double>>(now - last_pos_time_).count();
                if(dt <= 0) dt = 1.0 / fps_;
                float vx = 0.0f;
                {
                    std::lock_guard<std::mutex> lk(data_mutex_);
                    if(last_center_x_valid_) {
                        vx = (center_x - last_center_x_) / dt;
                    } else {
                        vx = 0.0f; // 初始时速度为0
                        last_center_x_valid_ = true;
                    }
                    // 更新历史量（用于速度计算与滤波）
                    last_center_x_ = center_x;
                    last_pos_time_ = now;
                }

                // 对测得的中心做低通滤波，降低检测抖动
                float center_x_filtered = filter_.filter(center_x);

                // 写入共享结构供控制环读取
                {
                    std::lock_guard<std::mutex> lk(data_mutex_);
                    shared_center_x_ = center_x_filtered;
                    shared_velocity_x_ = vx;
                    shared_frame_width_ = frame.cols;
                    target_valid_ = true;
                }
            } else {
                // 目标丢失，重置一些量
                filter_.reset();
                std::lock_guard<std::mutex> lk(data_mutex_);
                target_valid_ = false;
            }

            auto end = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            RCLCPP_DEBUG(this->get_logger(), "Detection loop time: %ld ms", duration);

            // 睡一小会，避免忙等待（如果摄像头帧率受限则可不用）
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    // 控制环：以固定频率读取共享的目标信息，计算 PID+FF，串口发送控制
    void controlLoop() {
        float center_x = 0.0f;
        float vx = 0.0f;
        int frame_w = 640;
        bool valid = false;
        {
            std::lock_guard<std::mutex> lk(data_mutex_);
            center_x = shared_center_x_;
            vx = shared_velocity_x_;
            frame_w = shared_frame_width_;
            valid = target_valid_;
        }

        auto now = this->now();
        double dt = 0.0;
        if(last_control_time_.nanoseconds() == 0) {
            dt = 1.0 / control_rate_;
        } else {
            dt = (now - last_control_time_).seconds();
            if(dt <= 0) dt = 1.0 / control_rate_;
        }
        last_control_time_ = now;

        float control_output = 0.0f;
        if(valid) {
            // 目标位置的 setpoint 为图像中心
            float setpoint = frame_w / 2.0f;

            // 调用 PID+前馈（传入测得速度与估计延迟）
            control_output = pid_.compute(setpoint, center_x, vx, (float)dt, system_delay_);

            // 串口发送（格式可按你协议修改）
            if(serial_ok_) {
                char buffer[64];
                // 这里发送控制量与检测的中心，用于下位机调试
                std::snprintf(buffer, sizeof(buffer), "CTRL %.2f %.2f\n", control_output, center_x);
                my_serial->write(buffer);
            }
        } else {
            // 目标丢失时，视策略决定：发送零控制/保持上一次/或发停止命令
            pid_.reset();
            if(serial_ok_) {
                char buffer[32];
                std::snprintf(buffer, sizeof(buffer), "STOP\n");
                my_serial->write(buffer);
            }
        }

        // 可选：日志、RCLCPP_INFO_TOO_MUCH
        RCLCPP_DEBUG(this->get_logger(), "control: valid=%d output=%.2f center=%.2f vx=%.2f dt=%.3f",
                     valid, control_output, center_x, vx, dt);
    }

private:
    // ------- hardware / algorithm objects -------
    cv::VideoCapture cap;
    std::unique_ptr<YoloDetector> detector;
    std::unique_ptr<BYTETracker> tracker;
    std::unique_ptr<serial::Serial> my_serial;

    // ------- Parameters & state -------
    std::string engine_path_;
    std::string serial_port_;
    std::string gst_pipeline;
    double fps_ = 30.0;
    double control_rate_ = 30.0;
    float system_delay_ = 0.05f; // s

    // PID + filter
    PIDFFController pid_;
    LowPassFilter filter_;

    // 共享数据（detection -> control）
    std::mutex data_mutex_;
    float shared_center_x_ = 0.0f;
    float shared_velocity_x_ = 0.0f;
    int shared_frame_width_ = 640;
    bool target_valid_ = false;

    // 用于速度计算
    std::chrono::steady_clock::time_point last_pos_time_ = std::chrono::steady_clock::now();
    float last_center_x_ = 0.0f;
    bool last_center_x_valid_ = false;

    // control timer
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::Time last_control_time_;

    // serial ok
    bool serial_ok_ = false;

    // threads
    std::thread cam_thread_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<YoloCppNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
