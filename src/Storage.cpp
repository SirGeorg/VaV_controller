#include "Storage.h"
#include <ArduinoJson.h>

Storage storage;

void Storage::begin() {
    prefs.begin("ventcfg", false);

    if (!prefs.isKey("initialized")) {
        setWinterMode(Defaults::winter_mode);
        setPowerLast(false);
        setCo2Deadband(Defaults::co2_deadband);
        setCo2AlarmThreshold(Defaults::co2_alarm_threshold);
        setMinServoStep(Defaults::min_servo_step);
        setRoomDataTimeout(Defaults::room_data_timeout_s);
        setTempFreecoolDeadband(Defaults::temp_freecool_deadband);
        setFreecoolOutdoorMin(Defaults::freecool_outdoor_min);
        setDpMin(Defaults::dp_min);
        setDpMax(Defaults::dp_max);
        setDpAlarmLow(Defaults::dp_alarm_low);
        setDpAlarmHigh(Defaults::dp_alarm_high);
        setDpLowTimeout(Defaults::dp_low_timeout_s);
        setDpHighTimeout(Defaults::dp_high_timeout_s);
        setMinFanStep(Defaults::min_fan_step);
        setMinFanSpeed(Defaults::min_fan_speed);
        setMinDpStep(Defaults::min_dp_step);
        setFlowAlarmThreshold(Defaults::flow_alarm_threshold);
        setDpWithinSetpointPct(Defaults::dp_within_setpoint_pct);
        setHeaterTargetTemp(Defaults::heater_target_temp);
        setHeaterDeadbandLow(Defaults::heater_deadband_low);
        setHeaterDeadbandHigh(Defaults::heater_deadband_high);
        setHeaterAlarmTemp(Defaults::heater_alarm_temp);
        setHeaterOutdoorBlockTemp(Defaults::heater_outdoor_block_temp);
        setHeaterMinFanRuntime(Defaults::heater_min_fan_runtime_s);

        for (uint8_t i = 0; i < ROOM_COUNT; i++) {
            setRoomCo2Target(i, Defaults::co2_target);
            setRoomMinPos(i, Defaults::min_pos);
            setRoomMaxPos(i, Defaults::max_pos);
            setRoomTempTarget(i, Defaults::temp_target);
            setRoomTempModeEnable(i, false);
        }

        prefs.putBool("initialized", true);
    }
}

float Storage::getF(const char* key, float def) {
    return prefs.isKey(key) ? prefs.getFloat(key, def) : def;
}
void  Storage::setF(const char* key, float v)     { prefs.putFloat(key, v); }
uint32_t Storage::getU(const char* key, uint32_t def) {
    return prefs.isKey(key) ? prefs.getUInt(key, def) : def;
}
void  Storage::setU(const char* key, uint32_t v)  { prefs.putUInt(key, v); }
bool  Storage::getB(const char* key, bool def)    {
    return prefs.isKey(key) ? prefs.getBool(key, def) : def;
}
void  Storage::setB(const char* key, bool v)      { prefs.putBool(key, v); }

String Storage::roomKey(uint8_t idx, const char* suffix) {
    return "r" + String(idx) + "_" + suffix;
}

// ---- общие ----
bool Storage::getWinterMode()        { return getB("winter_mode", Defaults::winter_mode); }
void Storage::setWinterMode(bool v)  { setB("winter_mode", v); }
bool Storage::getPowerLast()         { return getB("power_last", false); }
void Storage::setPowerLast(bool v)   { setB("power_last", v); }

// ---- CO2 / заслонки ----
float Storage::getCo2Deadband()            { return getF("co2_db", Defaults::co2_deadband); }
void  Storage::setCo2Deadband(float v)     { setF("co2_db", v); }
float Storage::getCo2AlarmThreshold()      { return getF("co2_alarm", Defaults::co2_alarm_threshold); }
void  Storage::setCo2AlarmThreshold(float v){ setF("co2_alarm", v); }
float Storage::getMinServoStep()           { return getF("srv_step", Defaults::min_servo_step); }
void  Storage::setMinServoStep(float v)    { setF("srv_step", v); }
uint32_t Storage::getRoomDataTimeout()     { return getU("room_to", Defaults::room_data_timeout_s); }
void  Storage::setRoomDataTimeout(uint32_t v){ setU("room_to", v); }

RoomSettings Storage::getRoom(uint8_t idx) {
    RoomSettings r;
    r.co2_target        = getF(roomKey(idx, "co2t").c_str(), Defaults::co2_target);
    r.min_pos            = getF(roomKey(idx, "minp").c_str(), Defaults::min_pos);
    r.max_pos            = getF(roomKey(idx, "maxp").c_str(), Defaults::max_pos);
    r.temp_target        = getF(roomKey(idx, "tt").c_str(), Defaults::temp_target);
    r.temp_mode_enable   = getB(roomKey(idx, "tme").c_str(), false);
    return r;
}
void Storage::setRoomCo2Target(uint8_t idx, float v)      { setF(roomKey(idx, "co2t").c_str(), v); }
void Storage::setRoomMinPos(uint8_t idx, float v)         { setF(roomKey(idx, "minp").c_str(), v); }
void Storage::setRoomMaxPos(uint8_t idx, float v)         { setF(roomKey(idx, "maxp").c_str(), v); }
void Storage::setRoomTempTarget(uint8_t idx, float v)     { setF(roomKey(idx, "tt").c_str(), v); }
void Storage::setRoomTempModeEnable(uint8_t idx, bool v)  { setB(roomKey(idx, "tme").c_str(), v); }

// ---- фрикулинг ----
float Storage::getTempFreecoolDeadband()      { return getF("fc_db", Defaults::temp_freecool_deadband); }
void  Storage::setTempFreecoolDeadband(float v){ setF("fc_db", v); }
float Storage::getFreecoolOutdoorMin()        { return getF("fc_omin", Defaults::freecool_outdoor_min); }
void  Storage::setFreecoolOutdoorMin(float v) { setF("fc_omin", v); }

// ---- вентилятор / давление ----
float Storage::getDpMin()  { return getF("dp_min", Defaults::dp_min); }
void  Storage::setDpMin(float v) { setF("dp_min", v); }
float Storage::getDpMax()  { return getF("dp_max", Defaults::dp_max); }
void  Storage::setDpMax(float v) { setF("dp_max", v); }
float Storage::getDpAlarmLow()   { return getF("dp_al", Defaults::dp_alarm_low); }
void  Storage::setDpAlarmLow(float v)  { setF("dp_al", v); }
float Storage::getDpAlarmHigh()  { return getF("dp_ah", Defaults::dp_alarm_high); }
void  Storage::setDpAlarmHigh(float v) { setF("dp_ah", v); }
uint32_t Storage::getDpLowTimeout()  { return getU("dp_lt", Defaults::dp_low_timeout_s); }
void  Storage::setDpLowTimeout(uint32_t v)  { setU("dp_lt", v); }
uint32_t Storage::getDpHighTimeout() { return getU("dp_ht", Defaults::dp_high_timeout_s); }
void  Storage::setDpHighTimeout(uint32_t v) { setU("dp_ht", v); }
float Storage::getMinFanStep()   { return getF("fan_step", Defaults::min_fan_step); }
void  Storage::setMinFanStep(float v)  { setF("fan_step", v); }
float Storage::getMinFanSpeed()  { return getF("fan_min", Defaults::min_fan_speed); }
void  Storage::setMinFanSpeed(float v) { setF("fan_min", v); }
float Storage::getMinDpStep()    { return getF("dp_step", Defaults::min_dp_step); }
void  Storage::setMinDpStep(float v)   { setF("dp_step", v); }
float Storage::getFlowAlarmThreshold() { return getF("flow_al", Defaults::flow_alarm_threshold); }
void  Storage::setFlowAlarmThreshold(float v) { setF("flow_al", v); }
float Storage::getDpWithinSetpointPct() { return getF("dp_within", Defaults::dp_within_setpoint_pct); }
void  Storage::setDpWithinSetpointPct(float v) { setF("dp_within", v); }

// ---- нагреватель ----
float Storage::getHeaterTargetTemp()   { return getF("h_target", Defaults::heater_target_temp); }
void  Storage::setHeaterTargetTemp(float v) { setF("h_target", v); }
float Storage::getHeaterDeadbandLow()  { return getF("h_dbl", Defaults::heater_deadband_low); }
void  Storage::setHeaterDeadbandLow(float v) { setF("h_dbl", v); }
float Storage::getHeaterDeadbandHigh() { return getF("h_dbh", Defaults::heater_deadband_high); }
void  Storage::setHeaterDeadbandHigh(float v) { setF("h_dbh", v); }
float Storage::getHeaterAlarmTemp()    { return getF("h_alarm", Defaults::heater_alarm_temp); }
void  Storage::setHeaterAlarmTemp(float v) { setF("h_alarm", v); }
float Storage::getHeaterOutdoorBlockTemp() { return getF("h_oblk", Defaults::heater_outdoor_block_temp); }
void  Storage::setHeaterOutdoorBlockTemp(float v) { setF("h_oblk", v); }
uint32_t Storage::getHeaterMinFanRuntime() { return getU("h_minfan", Defaults::heater_min_fan_runtime_s); }
void  Storage::setHeaterMinFanRuntime(uint32_t v) { setU("h_minfan", v); }

// ---- FAULT персистентность ----
bool Storage::getHeaterFaultLatched()  { return getB("hflt", false); }
void Storage::setHeaterFaultLatched(bool v) { setB("hflt", v); }
bool Storage::getFanFaultLatched()     { return getB("fflt", false); }
void Storage::setFanFaultLatched(bool v)    { setB("fflt", v); }
void Storage::setHeaterFaultCode(uint8_t code, float temp) { prefs.putUChar("hf_code", code); setF("hf_temp", temp); }
void Storage::setFanFaultCode(uint8_t code, float temp)    { prefs.putUChar("ff_code", code); setF("ff_temp", temp); }
uint8_t Storage::getHeaterFaultCode() { return prefs.isKey("hf_code") ? prefs.getUChar("hf_code", 0) : 0; }
uint8_t Storage::getFanFaultCode()    { return prefs.isKey("ff_code") ? prefs.getUChar("ff_code", 0) : 0; }
float   Storage::getHeaterFaultTemp() { return getF("hf_temp", 0.0f); }
float   Storage::getFanFaultTemp()    { return getF("ff_temp", 0.0f); }

// ---- калибровка сервоприводов ----
void Storage::getServoCal(uint8_t idx, uint16_t &pulseMinUs, uint16_t &pulseMaxUs) {
    pulseMinUs = prefs.getUShort(("sv" + String(idx) + "min").c_str(), 500);
    pulseMaxUs = prefs.getUShort(("sv" + String(idx) + "max").c_str(), 2500);
}
void Storage::setServoCal(uint8_t idx, uint16_t pulseMinUs, uint16_t pulseMaxUs) {
    prefs.putUShort(("sv" + String(idx) + "min").c_str(), pulseMinUs);
    prefs.putUShort(("sv" + String(idx) + "max").c_str(), pulseMaxUs);
}

// ---- калибровка датчиков 4-20мА ----
void Storage::getAnalogCal(const char* key, float &offset, float &scale) {
    offset = getF((String(key) + "_o").c_str(), 0.0f);
    scale  = getF((String(key) + "_s").c_str(), 1.0f);
}
void Storage::setAnalogCal(const char* key, float offset, float scale) {
    setF((String(key) + "_o").c_str(), offset);
    setF((String(key) + "_s").c_str(), scale);
}

// ---- экспорт / импорт (п.12) ----
String Storage::exportJson() {
    DynamicJsonDocument doc(2048);
    doc["fw_version"] = FW_VERSION;
    doc["winter_mode"] = getWinterMode();
    doc["co2_deadband"] = getCo2Deadband();
    doc["co2_alarm_threshold"] = getCo2AlarmThreshold();
    doc["min_servo_step"] = getMinServoStep();
    doc["room_data_timeout"] = getRoomDataTimeout();
    doc["temp_freecool_deadband"] = getTempFreecoolDeadband();
    doc["freecool_outdoor_min"] = getFreecoolOutdoorMin();
    doc["dp_min"] = getDpMin();
    doc["dp_max"] = getDpMax();
    doc["dp_alarm_low"] = getDpAlarmLow();
    doc["dp_alarm_high"] = getDpAlarmHigh();
    doc["dp_low_timeout"] = getDpLowTimeout();
    doc["dp_high_timeout"] = getDpHighTimeout();
    doc["min_fan_step"] = getMinFanStep();
    doc["min_fan_speed"] = getMinFanSpeed();
    doc["min_dp_step"] = getMinDpStep();
    doc["flow_alarm_threshold"] = getFlowAlarmThreshold();
    doc["dp_within_setpoint_pct"] = getDpWithinSetpointPct();
    doc["heater_target_temp"] = getHeaterTargetTemp();
    doc["heater_deadband_low"] = getHeaterDeadbandLow();
    doc["heater_deadband_high"] = getHeaterDeadbandHigh();
    doc["heater_alarm_temp"] = getHeaterAlarmTemp();
    doc["heater_outdoor_block_temp"] = getHeaterOutdoorBlockTemp();
    doc["heater_min_fan_runtime"] = getHeaterMinFanRuntime();

    JsonArray rooms = doc.createNestedArray("rooms");
    for (uint8_t i = 0; i < ROOM_COUNT; i++) {
        RoomSettings r = getRoom(i);
        JsonObject ro = rooms.createNestedObject();
        ro["co2_target"] = r.co2_target;
        ro["min_pos"] = r.min_pos;
        ro["max_pos"] = r.max_pos;
        ro["temp_target"] = r.temp_target;
        ro["temp_mode_enable"] = r.temp_mode_enable;
    }
    String out;
    serializeJson(doc, out);
    return out;
}

bool Storage::importJson(const String &json) {
    DynamicJsonDocument doc(2048);
    if (deserializeJson(doc, json) != DeserializationError::Ok) return false;

    if (doc.containsKey("winter_mode")) setWinterMode(doc["winter_mode"]);
    if (doc.containsKey("co2_deadband")) setCo2Deadband(doc["co2_deadband"]);
    if (doc.containsKey("co2_alarm_threshold")) setCo2AlarmThreshold(doc["co2_alarm_threshold"]);
    if (doc.containsKey("min_servo_step")) setMinServoStep(doc["min_servo_step"]);
    if (doc.containsKey("room_data_timeout")) setRoomDataTimeout(doc["room_data_timeout"]);
    if (doc.containsKey("temp_freecool_deadband")) setTempFreecoolDeadband(doc["temp_freecool_deadband"]);
    if (doc.containsKey("freecool_outdoor_min")) setFreecoolOutdoorMin(doc["freecool_outdoor_min"]);
    if (doc.containsKey("dp_min")) setDpMin(doc["dp_min"]);
    if (doc.containsKey("dp_max")) setDpMax(doc["dp_max"]);
    if (doc.containsKey("dp_alarm_low")) setDpAlarmLow(doc["dp_alarm_low"]);
    if (doc.containsKey("dp_alarm_high")) setDpAlarmHigh(doc["dp_alarm_high"]);
    if (doc.containsKey("dp_low_timeout")) setDpLowTimeout(doc["dp_low_timeout"]);
    if (doc.containsKey("dp_high_timeout")) setDpHighTimeout(doc["dp_high_timeout"]);
    if (doc.containsKey("min_fan_step")) setMinFanStep(doc["min_fan_step"]);
    if (doc.containsKey("min_fan_speed")) setMinFanSpeed(doc["min_fan_speed"]);
    if (doc.containsKey("min_dp_step")) setMinDpStep(doc["min_dp_step"]);
    if (doc.containsKey("flow_alarm_threshold")) setFlowAlarmThreshold(doc["flow_alarm_threshold"]);
    if (doc.containsKey("dp_within_setpoint_pct")) setDpWithinSetpointPct(doc["dp_within_setpoint_pct"]);
    if (doc.containsKey("heater_target_temp")) setHeaterTargetTemp(doc["heater_target_temp"]);
    if (doc.containsKey("heater_deadband_low")) setHeaterDeadbandLow(doc["heater_deadband_low"]);
    if (doc.containsKey("heater_deadband_high")) setHeaterDeadbandHigh(doc["heater_deadband_high"]);
    if (doc.containsKey("heater_alarm_temp")) setHeaterAlarmTemp(doc["heater_alarm_temp"]);
    if (doc.containsKey("heater_outdoor_block_temp")) setHeaterOutdoorBlockTemp(doc["heater_outdoor_block_temp"]);
    if (doc.containsKey("heater_min_fan_runtime")) setHeaterMinFanRuntime(doc["heater_min_fan_runtime"]);

    if (doc.containsKey("rooms")) {
        JsonArray rooms = doc["rooms"];
        uint8_t i = 0;
        for (JsonObject ro : rooms) {
            if (i >= ROOM_COUNT) break;
            if (ro.containsKey("co2_target")) setRoomCo2Target(i, ro["co2_target"]);
            if (ro.containsKey("min_pos")) setRoomMinPos(i, ro["min_pos"]);
            if (ro.containsKey("max_pos")) setRoomMaxPos(i, ro["max_pos"]);
            if (ro.containsKey("temp_target")) setRoomTempTarget(i, ro["temp_target"]);
            if (ro.containsKey("temp_mode_enable")) setRoomTempModeEnable(i, ro["temp_mode_enable"]);
            i++;
        }
    }
    return true;
}
