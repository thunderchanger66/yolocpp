#pragma once

//低通滤波器
class LowPassFilter
{
public:
    LowPassFilter(float alpha = 0.7f):
        alpha_(alpha), initialized_(false), state_(0.0f) {}

    float filter(float x)
    {
        if(!initialized_) { state_ = x; initialized_ = true; return state_; }
        state_ = alpha_ * x + (1.0f - alpha_) * state_;
        return state_;
    }
    void reset() { initialized_ = false; }

private:
    float alpha_;
    bool initialized_;
    float state_;
};

class PIDFF
{
public:
    PIDFF(float kp = 0.01f, float ki = 0.0f, float kd = 0.0f, float kf = 0.0f, 
        float min_out = -100.0f, float max_out = 100.0f):
        kp_(kp), ki_(ki), kd_(kd), kf_(kf), min_out_(min_out), max_out_(max_out),
        prev_error_(0.0f), integral_(0.0f), prev_measurement_(0.0f), prev_setpoint_(0.0f),
        first_(true) {}

    void reset() 
    {
        integral_ = 0.0f; prev_error_ = 0.0f; prev_measurement_ = 0.0f; prev_setpoint_ = 0.0f; first_ = true;
    }

    float compute(float setpoint, float measurement, float system_delay);

private:
    float kp_, ki_, kd_, kf_;
    float min_out_, max_out_;
    float prev_error_, integral_, prev_measurement_, prev_setpoint_;
    bool first_;

    LowPassFilter lowpass;//先放个低通滤波器
};