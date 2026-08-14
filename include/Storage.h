#pragma once
#include <Arduino.h>
#include <Preferences.h>
#include "Config.h"

// ============================================================
//  Storage — единая точка хранения всех настраиваемых параметров
//  в NVS (Preferences). Каждый параметр имеет короткий ключ
//  (<=15 символов, ограничение Preferences) и метод get/set.
//  FAULT-флаги (heater_fault/fan_fault) тоже персистентны (п.3).
// ============================================================

struct RoomSettings {
    float co2_target;
    float min_pos;
    float max_pos;
    float temp_target;
    bool  temp_mode_enable;
};

struct MqttConfig {
    char server[64];
    uint16_t port;
    char user[32];
    char pass[32];
    char client_id[32];
    char root[32];
};

class Storage {
public:
    void begin();

    // ---- общие ----
    bool getWinterMode();      void setWinterMode(bool v);
    bool getPowerLast();       void setPowerLast(bool v); // для UI, не для автозапуска (п.4)

    // ---- MQTT конфигурация ----
    MqttConfig getMqttConfig();
    void setMqttConfig(const MqttConfig &cfg);
    void setMqttServer(const char* server);
    void setMqttPort(uint16_t port);
    void setMqttUser(const char* user);
    void setMqttPass(const char* pass);
    void setMqttClientId(const char* id);
    void setMqttRoot(const char* root);

    // ---- CO2 / заслонки ----
    float getCo2Deadband();          void setCo2Deadband(float v);
    float getCo2AlarmThreshold();    void setCo2AlarmThreshold(float v);
    float getMinServoStep();         void setMinServoStep(float v);
    uint32_t getRoomDataTimeout();   void setRoomDataTimeout(uint32_t v);

    RoomSettings getRoom(uint8_t idx);
    void setRoomCo2Target(uint8_t idx, float v);
    void setRoomMinPos(uint8_t idx, float v);
    void setRoomMaxPos(uint8_t idx, float v);
    void setRoomTempTarget(uint8_t idx, float v);
    void setRoomTempModeEnable(uint8_t idx, bool v);

    // ---- фрикулинг ----
    float getTempFreecoolDeadband();  void setTempFreecoolDeadband(float v);
    float getFreecoolOutdoorMin();    void setFreecoolOutdoorMin(float v);

    // ---- вентилятор / давление ----
    float getDpMin();  void setDpMin(float v);
    float getDpMax();  void setDpMax(float v);
    float getDpAlarmLow();   void setDpAlarmLow(float v);
    float getDpAlarmHigh();  void setDpAlarmHigh(float v);
    uint32_t getDpLowTimeout();   void setDpLowTimeout(uint32_t v);
    uint32_t getDpHighTimeout();  void setDpHighTimeout(uint32_t v);
    float getMinFanStep();   void setMinFanStep(float v);
    float getMinFanSpeed();  void setMinFanSpeed(float v);
    float getMinDpStep();    void setMinDpStep(float v);
    float getFlowAlarmThreshold(); void setFlowAlarmThreshold(float v);
    float getDpWithinSetpointPct(); void setDpWithinSetpointPct(float v);
    float getDpKp(); void setDpKp(float v);
    float getDpKi(); void setDpKi(float v);
    float getDpKd(); void setDpKd(float v);

    // ---- нагреватель ----
    float getHeaterTargetTemp();     void setHeaterTargetTemp(float v);
    float getHeaterDeadbandLow();    void setHeaterDeadbandLow(float v);
    float getHeaterDeadbandHigh();   void setHeaterDeadbandHigh(float v);
    float getHeaterAlarmTemp();      void setHeaterAlarmTemp(float v);
    float getHeaterOutdoorBlockTemp();void setHeaterOutdoorBlockTemp(float v);
    uint32_t getHeaterMinFanRuntime(); void setHeaterMinFanRuntime(uint32_t v);

    // ---- FAULT персистентность (п.3) ----
    bool getHeaterFaultLatched();  void setHeaterFaultLatched(bool v);
    bool getFanFaultLatched();     void setFanFaultLatched(bool v);
    void setHeaterFaultCode(uint8_t code, float temp);
    void setFanFaultCode(uint8_t code, float temp);
    uint8_t getHeaterFaultCode();
    uint8_t getFanFaultCode();
    float   getHeaterFaultTemp();
    float   getFanFaultTemp();

    // ---- калибровка сервоприводов (п.6) ----
    void  getServoCal(uint8_t idx, uint16_t &pulseMinUs, uint16_t &pulseMaxUs);
    void  setServoCal(uint8_t idx, uint16_t pulseMinUs, uint16_t pulseMaxUs);

    // ---- калибровка датчиков 4-20мА (п.6): raw мА -> физическая величина ----
    // physical = offset + scale * (mA - 4.0)
    void  getAnalogCal(const char* key, float &offset, float &scale);
    void  setAnalogCal(const char* key, float offset, float scale);

    // ---- экспорт/импорт всей конфигурации (п.12) ----
    String exportJson();
    bool   importJson(const String &json);

private:
    Preferences prefs;
    float getF(const char* key, float def);
    void  setF(const char* key, float v);
    uint32_t getU(const char* key, uint32_t def);
    void  setU(const char* key, uint32_t v);
    bool  getB(const char* key, bool def);
    void  setB(const char* key, bool v);
    String roomKey(uint8_t idx, const char* suffix);
};

extern Storage storage;
