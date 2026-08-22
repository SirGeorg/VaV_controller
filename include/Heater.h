#pragma once
#include <Arduino.h>
#include "Config.h"
#include "Storage.h"
#include "Sensors.h"
#include "Fan.h"
#include "Dampers.h"
#include "EventLog.h"

// ============================================================
//  Heater — 5-ступенчатый нагреватель.
//   BLOCK (автоснятие, не latched) — любое из:
//     - система выключена (power=0)
//     - winter_mode = 0
//     - T_outdoor >= heater_outdoor_block_temp (T_in_min)
//     - фрикулинг активен хотя бы в одной комнате
//     - нет свежих данных приток/отток/улица (тайм-аут duct_sensor_timeout)
//     - вентилятор не работает / не отработал heater_min_fan_runtime_s
//     - расход воздуха ниже flow_alarm_threshold (%)
//     - обрыв датчика расхода (<3.5мА)
//     - факт. давление вне ±dp_within_setpoint_pct от dp_setpoint
//   FAULT (latched, ручной сброс):
//     - T_supply > heater_alarm_temp -> мгновенно все реле OFF
// ============================================================

enum class HeaterBlockReason : uint8_t {
    NONE = 0,
    SYSTEM_OFF,
    WINTER_MODE_OFF,
    OUTDOOR_TOO_WARM,
    FREECOOL_ACTIVE,
    NO_DUCT_DATA,
    FAN_NOT_READY,
    LOW_FLOW,
    FLOW_SENSOR_FAULT,
    PRESSURE_OUT_OF_RANGE
};

class Heater {
public:
    void begin();
    void update(float dtSeconds, bool systemOn);

    void triggerFault();
    void resetFault();

    // Публичный метод для принудительного отключения всех реле (вызывается из Fan::requestOff)
    void allRelaysOffPublic();

    uint8_t currentStage() const { return stage_; }
    HeaterBlockReason blockReason() const { return blockReason_; }
    bool faultLatched() const { return storage.getHeaterFaultLatched(); }
    uint8_t faultCode() const { return storage.getHeaterFaultCode(); }
    float faultTempAtTrip() const { return storage.getHeaterFaultTemp(); }

private:
    uint8_t stage_ = 0;
    HeaterBlockReason blockReason_ = HeaterBlockReason::NONE;
    unsigned long lastStageChangeMs_ = 0;

    void setStage(uint8_t n);
    void allRelaysOff();
};

extern Heater heater;
