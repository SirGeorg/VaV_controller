#pragma once
#include <Arduino.h>
#include "Config.h"
#include "Storage.h"
#include "Sensors.h"
#include "Dampers.h"
#include "PIDController.h"
#include "EventLog.h"

// ============================================================
//  Fan — вентилятор + контур давления.
//   Последовательность пуска:
//     STARTING (сбор данных, позиционирование заслонок, задержка
//               damper_settle_time_s) -> RAMPING (плавный разгон,
//               fan_ramp_time_s) -> RUNNING (PID по перепаду давления)
//   Последовательность останова:
//     STOPPING (выбег fan_coastdown_time_s) -> OFF
//   dp_setpoint вычисляется автоматически:
//     dp_setpoint = dp_min + (dp_max-dp_min) * maxPosAcrossRooms()/100
//   Аварии (fan_fault, отдельный от heater_fault, latched):
//     dp < dp_alarm_low  дольше dp_low_timeout  -> FAULT
//     dp > dp_alarm_high дольше dp_high_timeout -> FAULT, мгновенный
//                        стоп (PWM=0) без выбега
// ============================================================

enum class FanPhase : uint8_t { OFF, STARTING, RAMPING, RUNNING, STOPPING, FAULT };
enum class FanFaultCode : uint8_t { NONE = 0, LOW_PRESSURE = 1, HIGH_PRESSURE = 2 };

class Fan {
public:
    void begin();
    void update(float dtSeconds);

    // Пересчёт dp_setpoint от положения заслонок — вызывается извне (main.cpp)
    // синхронно с циклом Dampers::update(), раз в 10 секунд (см. обсуждение).
    void recomputeSetpoint();

    void requestOn();     // power=1
    void requestOff();    // power=0

    // мгновенное аварийное отключение (вызывается также извне при необходимости)
    void triggerFault(FanFaultCode code);
    void resetFault();    // ручной сброс fan_fault

    // --- геттеры состояния ---
    FanPhase phase() const { return phase_; }
    float pwmPct() const { return currentPwmPct_; }
    float dpSetpoint() const { return dpSetpoint_; }
    float dpMeasured() const { return sensors.pressurePa(); }
    bool  faultLatched() const { return storage.getFanFaultLatched(); }
    uint8_t faultCode() const { return storage.getFanFaultCode(); }
    float   faultPressureAtTrip() const { return storage.getFanFaultTemp(); }

    // используется Heater'ом для проверки условий разрешения нагрева
    bool fanRuntimeOk() const;                 // фан работает >= heater_min_fan_runtime_s
    bool dpWithinSetpointOk() const;            // |dp - dp_setpoint| <= dp_within_setpoint_pct
    bool isRunning() const { return phase_ == FanPhase::RAMPING || phase_ == FanPhase::RUNNING; }

private:
    FanPhase phase_ = FanPhase::OFF;
    unsigned long phaseStartMs_ = 0;
    unsigned long fanRunStartMs_ = 0;
    float currentPwmPct_ = 0;
    float dpSetpoint_ = 0;
    PIDController dpPid_{Defaults::dp_kp, Defaults::dp_ki, Defaults::dp_kd, 0.0f, 100.0f};

    unsigned long lowTimerStartMs_ = 0;
    unsigned long highTimerStartMs_ = 0;

    void applyPwm(float pct);
    void checkPressureAlarms(float dtSeconds);
};

extern Fan fan;
