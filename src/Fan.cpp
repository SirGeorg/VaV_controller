#include "Fan.h"
#include "Dampers.h"

Fan fan;

void Fan::begin() {
    ledcSetup(FAN_PWM_CHANNEL, FAN_PWM_FREQ_HZ, FAN_PWM_RES_BITS);
    ledcAttachPin(PIN_FAN_PWM, FAN_PWM_CHANNEL);
    applyPwm(0);
    dpSetpoint_ = storage.getDpMin();
    // Коэффициенты PID хранятся в NVS (управляются через веб/MQTT)
    dpPid_.setTunings(storage.getDpKp(), storage.getDpKi(), storage.getDpKd());

    // При старте ESP32, если аварии сохранились в NVS как активные -> остаёмся в FAULT (п.3, п.4)
    if (storage.getFanFaultLatched()) {
        phase_ = FanPhase::FAULT;
    }
}

void Fan::setDpTunings(float kp, float ki, float kd) {
    storage.setDpKp(kp);
    storage.setDpKi(ki);
    storage.setDpKd(kd);
    dpPid_.setTunings(kp, ki, kd);
}

void Fan::applyPwm(float pct) {
    pct = constrain(pct, 0.0f, 100.0f);
    currentPwmPct_ = pct;
    uint32_t maxDuty = (1 << FAN_PWM_RES_BITS) - 1;
    uint32_t duty = (uint32_t)(pct / 100.0f * maxDuty);
    ledcWrite(FAN_PWM_CHANNEL, duty);
}

void Fan::recomputeSetpoint() {
    float dpMin = storage.getDpMin();
    float dpMax = storage.getDpMax();
    float maxPos = dampers.maxPosAcrossRooms();   // включает ручной режим и CO2-форсаж
    float newSetpoint = dpMin + (dpMax - dpMin) * (maxPos / 100.0f);

    float minStep = storage.getMinDpStep();
    if (fabs(newSetpoint - dpSetpoint_) >= minStep) {
        dpSetpoint_ = newSetpoint;
    }
}

void Fan::requestOn() {
    if (phase_ == FanPhase::FAULT) return;   // нужен явный сброс аварии
    if (phase_ != FanPhase::OFF) return;
    phase_ = FanPhase::STARTING;
    phaseStartMs_ = millis();
    eventLog.add("Fan: старт последовательности включения");
}

void Fan::requestOff() {
    if (phase_ == FanPhase::OFF || phase_ == FanPhase::STOPPING) return;
    if (phase_ == FanPhase::FAULT) return;   // авария уже остановила всё мгновенно
    phase_ = FanPhase::STOPPING;
    phaseStartMs_ = millis();
    lowTimerStartMs_ = 0;
    highTimerStartMs_ = 0;
    eventLog.add("Fan: начат выбег перед остановкой");
}

void Fan::triggerFault(FanFaultCode code) {
    applyPwm(0);   // мгновенный стоп без выбега (п. "отключаемся с ошибкой сразу в 0")
    phase_ = FanPhase::FAULT;
    storage.setFanFaultLatched(true);
    storage.setFanFaultCode((uint8_t)code, dpMeasured());
    eventLog.add(String("Fan FAULT code=") + (int)code + " dp=" + String(dpMeasured(), 1));
}

void Fan::resetFault() {
    storage.setFanFaultLatched(false);
    storage.setFanFaultCode(0, 0);
    phase_ = FanPhase::OFF;
    lowTimerStartMs_ = 0;
    highTimerStartMs_ = 0;
    inServiceMode_ = false;
    servicePwm_ = 0;
    eventLog.add("Fan: авария сброшена вручную");
}

void Fan::setServicePwm(float pct) {
    inServiceMode_ = true;
    servicePwm_ = constrain(pct, 0.0f, 100.0f);
    applyPwm(servicePwm_);
}

bool Fan::fanRuntimeOk() const {
    if (!isRunning()) return false;
    return (millis() - fanRunStartMs_) >= (storage.getHeaterMinFanRuntime() * 1000UL);
}

bool Fan::dpWithinSetpointOk() const {
    float pct = storage.getDpWithinSetpointPct();
    float tol = dpSetpoint_ * (pct / 100.0f);
    return fabs(dpMeasured() - dpSetpoint_) <= tol;
}

void Fan::checkPressureAlarms(float dtSeconds) {
    if (phase_ != FanPhase::RAMPING && phase_ != FanPhase::RUNNING) {
        lowTimerStartMs_ = 0;
        highTimerStartMs_ = 0;
        return;
    }
    float dp = dpMeasured();
    float alarmLow = storage.getDpAlarmLow();
    float alarmHigh = storage.getDpAlarmHigh();

    // --- низкое давление: не поднимается выше аварийной нижней границы ---
    if (dp < alarmLow) {
        if (lowTimerStartMs_ == 0) lowTimerStartMs_ = millis();
        if (millis() - lowTimerStartMs_ >= storage.getDpLowTimeout() * 1000UL) {
            triggerFault(FanFaultCode::LOW_PRESSURE);
            return;
        }
    } else {
        lowTimerStartMs_ = 0;
    }

    // --- высокое давление: не опускается ниже аварийной верхней границы ---
    if (dp > alarmHigh) {
        if (highTimerStartMs_ == 0) highTimerStartMs_ = millis();
        if (millis() - highTimerStartMs_ >= storage.getDpHighTimeout() * 1000UL) {
            triggerFault(FanFaultCode::HIGH_PRESSURE);
            return;
        }
    } else {
        highTimerStartMs_ = 0;
    }

    // --- критическое загрязнение фильтра: давление > 120% от порога фильтра ---
    if (sensors.filterBlockedCritical()) {
        triggerFault(FanFaultCode::FILTER_BLOCKED);
        return;
    }
}

void Fan::checkPressureSensor(unsigned long graceMs) {
    // Отсутствие показаний датчика давления (обрыв 4-20мА/NaN) на рабочих фазах 1-4
    // — аварийный случай. Активируем только при активном управлении вентилятором.
    bool inControl = phase_ == FanPhase::STARTING || phase_ == FanPhase::RAMPING ||
                     phase_ == FanPhase::RUNNING || phase_ == FanPhase::STOPPING;
    if (!inControl) { pressureSensorGraceMs_ = 0; return; }

    if (!sensors.pressureSensorOk()) {
        // Датчик не даёт показаний: запускаем короткий грэйс, если ещё не запущен
        if (pressureSensorGraceMs_ == 0) pressureSensorGraceMs_ = millis();
        if (millis() - pressureSensorGraceMs_ >= graceMs) {
            pressureSensorGraceMs_ = 0;
            triggerFault(FanFaultCode::SENSOR_LOOP);
        }
    } else {
        // Показания вернулись — сбрасываем грэйс (случайный глитч не создаёт аварию)
        pressureSensorGraceMs_ = 0;
    }
}

void Fan::update(float dtSeconds) {
    // Проверка наличия датчика давления — фазы 1-4 (до value-аварий давления)
    checkPressureSensor(Defaults::sensor_grace_s * 1000UL);
    checkPressureAlarms(dtSeconds);

    if (phase_ == FanPhase::FAULT) {
        applyPwm(0);
        return;
    }

    // Сервисный режим: прямое управление PWM, байпас стандартной логики
    if (inServiceMode_) {
        applyPwm(servicePwm_);
        return;
    }

    switch (phase_) {
        case FanPhase::OFF:
            applyPwm(0);
            break;

        case FanPhase::STARTING: {
            applyPwm(0);
            // В этот период Dampers::update() (вызывается отдельно в main.cpp) уже
            // собирает данные датчиков/MQTT и позиционирует заслонки в целевое положение.
            if (millis() - phaseStartMs_ >= Defaults::damper_settle_time_s * 1000UL) {
                phase_ = FanPhase::RAMPING;
                phaseStartMs_ = millis();
                fanRunStartMs_ = millis();
                dpPid_.reset();
                eventLog.add("Fan: заслонки позиционированы, разгон вентилятора");
            }
            break;
        }

        case FanPhase::RAMPING: {
            unsigned long elapsed = millis() - phaseStartMs_;
            float frac = (float)elapsed / (Defaults::fan_ramp_time_s * 1000UL);
            if (frac >= 1.0f) {
                phase_ = FanPhase::RUNNING;
                frac = 1.0f;
            }
            float pidOut = dpPid_.compute(dpSetpoint_, dpMeasured(), dtSeconds);
            applyPwm(pidOut * frac);
            break;
        }

        case FanPhase::RUNNING: {
            float pidOut = dpPid_.compute(dpSetpoint_, dpMeasured(), dtSeconds);
            float minSpeed = storage.getMinFanSpeed();
            if (pidOut < minSpeed) pidOut = minSpeed;

            float minStep = storage.getMinFanStep();
            if (fabs(pidOut - currentPwmPct_) >= minStep) {
                applyPwm(pidOut);
            }
            break;
        }

        case FanPhase::STOPPING: {
            // Во время выбега PWM не подаётся — вентилятор останавливается по инерции.
            applyPwm(0);
            if (millis() - phaseStartMs_ >= Defaults::fan_coastdown_time_s * 1000UL) {
                phase_ = FanPhase::OFF;
                eventLog.add("Fan: выбег завершён, остановлен");
            }
            break;
        }

        default: break;
    }
}
