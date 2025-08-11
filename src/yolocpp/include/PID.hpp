#pragma once

class PID
{
private:
    float kp; // Proportional gain
    float ki; // Integral gain
    float kd; // Derivative gain

    float integral; // Integral of error
    //float dt; // Time step

    float prev_error; // Previous error
    float prev_derivative; // Previous derivative

    float integral_limit; // Limit for integral windup

    float alpha; // Low-pass filter coefficient

public:
    PID(float kp, float ki, float kd, float integral_limit = 10.0f, float alpha = 0.1f)
        : kp(kp), ki(ki), kd(kd), integral_limit(integral_limit), alpha(alpha),
          integral(0.0f), prev_error(0.0f), prev_derivative(0.0f) {}
    
    float compute(float& setpoint, float& measured_value, double dt);

    void reset()
    {
        integral = 0.0f;
        prev_error = 0.0f;
        prev_derivative = 0.0f;
    }
};