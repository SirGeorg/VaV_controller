#pragma once
#include <Arduino.h>

// ============================================================
//  Простой PID-контроллер общего назначения (header-only).
//  Используется для: перепада давления вентилятора,
//  а также как база для PI-регулирования CO2 по заслонкам.
// ============================================================

class PIDController {
public:
    PIDController(float kp = 1.0f, float ki = 0.0f, float kd = 0.0f,
                  float outMin = 0.0f, float outMax = 100.0f)
        : kp_(kp), ki_(ki), kd_(kd), outMin_(outMin), outMax_(outMax) {}

    void setTunings(float kp, float ki, float kd) { kp_ = kp; ki_ = ki; kd_ = kd; }
    float kp() const { return kp_; }
    float ki() const { return ki_; }
    float kd() const { return kd_; }
    void setOutputLimits(float mn, float mx) { outMin_ = mn; outMax_ = mx; }
    void reset() { integral_ = 0; lastError_ = 0; firstRun_ = true; }

    // dtSeconds — период вызова в секундах
    float compute(float setpoint, float measured, float dtSeconds) {
        float error = setpoint - measured;
        integral_ += error * dtSeconds;

        // простое ограничение интегральной составляющей (anti-windup)
        float maxIntegral = (outMax_ - outMin_) / (ki_ > 0.0001f ? ki_ : 1.0f);
        integral_ = constrain(integral_, -maxIntegral, maxIntegral);

        float deriv = 0;
        if (!firstRun_ && dtSeconds > 0.0001f) {
            deriv = (error - lastError_) / dtSeconds;
        }
        firstRun_ = false;
        lastError_ = error;

        float out = kp_ * error + ki_ * integral_ + kd_ * deriv;
        out = constrain(out, outMin_, outMax_);
        return out;
    }

private:
    float kp_, ki_, kd_;
    float outMin_, outMax_;
    float integral_ = 0;
    float lastError_ = 0;
    bool firstRun_ = true;
};
