#include "Heater.h"

Heater heater;

void Heater::begin() {
    for (uint8_t i = 0; i < HEATER_STAGES; i++) {
        pinMode(PIN_RELAY[i], OUTPUT);
        digitalWrite(PIN_RELAY[i], LOW);
    }
    stage_ = 0;
    lastStageChangeMs_ = millis();

    // Персистентность аварии через reboot (п.3) — если авария была активна, остаёмся заблокированными
    // до явного MQTT-сброса; сами реле остаются выключенными (allRelaysOff уже сделан выше).
}

void Heater::allRelaysOff() {
    for (uint8_t i = 0; i < HEATER_STAGES; i++) digitalWrite(PIN_RELAY[i], LOW);
    stage_ = 0;
}

void Heater::setStage(uint8_t n) {
    if (n > HEATER_STAGES) n = HEATER_STAGES;
    for (uint8_t i = 0; i < HEATER_STAGES; i++) {
        digitalWrite(PIN_RELAY[i], (i < n) ? HIGH : LOW);
    }
    if (n != stage_) {
        eventLog.add("Heater: ступень " + String(stage_) + " -> " + String(n));
    }
    stage_ = n;
}

void Heater::triggerFault() {
    allRelaysOff();   // мгновенно, без ожидания stage_down_delay
    storage.setHeaterFaultLatched(true);
    storage.setHeaterFaultCode(1, sensors.supplyTemp());
    eventLog.add("Heater FAULT: перегрев, T_supply=" + String(sensors.supplyTemp(), 1));
}

void Heater::resetFault() {
    storage.setHeaterFaultLatched(false);
    storage.setHeaterFaultCode(0, 0);
    eventLog.add("Heater: авария сброшена вручную");
}

void Heater::update(float dtSeconds, bool systemOn) {
    // ---- Мгновенная авария по перегреву (проверяется всегда, даже если что-то ещё блокирует) ----
    if (sensors.supplyTempValid() && sensors.supplyTemp() > storage.getHeaterAlarmTemp()) {
        if (!storage.getHeaterFaultLatched()) {
            triggerFault();
        }
    }

    if (storage.getHeaterFaultLatched()) {
        allRelaysOff();
        blockReason_ = HeaterBlockReason::NONE;   // причина - FAULT, отражается отдельным флагом в состоянии
        return;
    }

    // ---- Одновременное аварийное отключение при fan_fault (низкое/высокое давление) ----
    if (fan.faultLatched()) {
        allRelaysOff();
        return;
    }

    // ---- Список блокировок (автоснятие) ----
    HeaterBlockReason reason = HeaterBlockReason::NONE;

    if (!systemOn) {
        reason = HeaterBlockReason::SYSTEM_OFF;
    } else if (!storage.getWinterMode()) {
        reason = HeaterBlockReason::WINTER_MODE_OFF;
    } else if (!sensors.ductSensorsFresh()) {
        reason = HeaterBlockReason::NO_DUCT_DATA;
    } else if (sensors.outdoorTemp() >= storage.getHeaterOutdoorBlockTemp()) {
        reason = HeaterBlockReason::OUTDOOR_TOO_WARM;
    } else if (dampers.anyFreecoolActive()) {
        reason = HeaterBlockReason::FREECOOL_ACTIVE;
    } else if (!fan.isRunning() || !fan.fanRuntimeOk()) {
        reason = HeaterBlockReason::FAN_NOT_READY;
    } else if (!sensors.flowSensorOk()) {
        reason = HeaterBlockReason::FLOW_SENSOR_FAULT;
    } else if (sensors.flowPct() < storage.getFlowAlarmThreshold()) {
        reason = HeaterBlockReason::LOW_FLOW;
    } else if (!fan.dpWithinSetpointOk()) {
        reason = HeaterBlockReason::PRESSURE_OUT_OF_RANGE;
    }

    blockReason_ = reason;

    if (reason != HeaterBlockReason::NONE) {
        allRelaysOff();
        return;
    }

    // ---- Штатная ступенчатая логика с гистерезисом ----
    float target = storage.getHeaterTargetTemp();
    float dbLow = storage.getHeaterDeadbandLow();
    float dbHigh = storage.getHeaterDeadbandHigh();
    float tSupply = sensors.supplyTemp();
    unsigned long now = millis();

    if (tSupply < target - dbLow) {
        if (now - lastStageChangeMs_ >= Defaults::heater_stage_up_delay_s * 1000UL) {
            if (stage_ < HEATER_STAGES) {
                setStage(stage_ + 1);
                lastStageChangeMs_ = now;
            }
        }
    } else if (tSupply > target + dbHigh) {
        if (now - lastStageChangeMs_ >= Defaults::heater_stage_down_delay_s * 1000UL) {
            if (stage_ > 0) {
                setStage(stage_ - 1);
                lastStageChangeMs_ = now;
            }
        }
    }
    // иначе — в пределах зоны гистерезиса, ступень не меняется
}
