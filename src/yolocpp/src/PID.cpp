#include "PID.hpp"
#include <algorithm>

float PIDFF::compute(float setpoint, float measurement, float system_delay) 
{
    float dt = 0.03;

    // 延迟补偿：预测测量在系统延迟后的值（线性预测）
    // 这里用测量速度（像素/s）乘延迟进行预测
    float predicted_measurement = measurement + 0 * system_delay;

    // 误差基于预测值（或者measurement）
    //float error = setpoint - predicted_measurement;
    float error = setpoint - measurement;

    // 积分
    integral_ += error * dt;

    // 微分（用测量值差分来降低噪声使用测量值的负导数）
    float derivative = 0.0f;
    if(!first_) derivative = -(predicted_measurement - prev_measurement_) / dt;

    float pid_out = kp_ * error + ki_ * integral_ + kd_ * derivative;

    // 前馈：使用目标速率（这里用 setpoint 变化率近似）
    // 前馈目的：当目标在移动时，直接施加额外控制以减少误差
    //float setpoint_velocity = (setpoint - prev_setpoint_) / dt; // 像素/s
    float setpoint_velocity = 0;
    float ff = kf_ * setpoint_velocity;

    //float out = pid_out + ff;
    float out = pid_out;
    out = std::clamp(out, min_out_, max_out_);

    // 保存历史
    prev_error_ = error;
    prev_measurement_ = predicted_measurement;
    prev_setpoint_ = setpoint;
    first_ = false;

    return out;
}