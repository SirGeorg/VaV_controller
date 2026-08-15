#pragma once
#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_ADS1X15.h>
#include "Config.h"
#include "Storage.h"

// ============================================================
//  Sensors — физические и внешние датчики.
//  Версия 4: свежесть CO2 и температуры комнат проверяется
//  независимо по отдельным timestamp/флагам.
// ============================================================

struct RoomData {
    float co2 = NAN;
    float temp = NAN;

    unsigned long lastCo2UpdateMs = 0;
    unsigned long lastTempUpdateMs = 0;

    bool co2EverReceived = false;
    bool tempEverReceived = false;
};

class Sensors {
public:
    void begin();
    void update();

    bool  outdoorTempValid();
    float outdoorTemp();
    bool  supplyTempValid();
    float supplyTemp();
    bool  ductSensorsFresh();

    float pressurePa();
    bool  pressureSensorOk();
    float flowPct();
    bool  flowSensorOk();

    // --- Filter pressure sensor ---
    float filterPressurePa();
    bool  filterSensorOk();
    bool  filterAlarmActive();
    void  resetFilterAlarm();
    void  setFilterAlarmThreshold(float v);
    float getFilterAlarmThreshold();

    void setRoomData(uint8_t idx, float co2, float temp);
    RoomData getRoomData(uint8_t idx);

    bool isRoomCo2Fresh(uint8_t idx);
    bool isRoomTempFresh(uint8_t idx);
    bool isRoomDataFresh(uint8_t idx); // совместимость: co2_fresh && temp_fresh

    void setPressureCal(float offset, float scale);
    void setFlowCal(float offset, float scale);
    void getPressureCal(float &offset, float &scale);
    void getFlowCal(float &offset, float &scale);
    
    // Filter sensor calibration
    void setFilterCal(float offset, float scale);
    void getFilterCal(float &offset, float &scale);

private:
    OneWire oneWireOutdoor{PIN_ONEWIRE_OUTDOOR};
    DallasTemperature dsOutdoor{&oneWireOutdoor};
    OneWire oneWireSupply{PIN_ONEWIRE_SUPPLY};
    DallasTemperature dsSupply{&oneWireSupply};
    Adafruit_ADS1115 ads;
    bool adsOk = false;

    enum class DsState { IDLE, CONVERTING };
    DsState dsState = DsState::IDLE;
    unsigned long dsConvertStartMs = 0;
    unsigned long dsLastCycleMs = 0;
    float dsTemps[DS_COUNT] = {NAN, NAN};
    unsigned long dsLastGoodMs[DS_COUNT] = {0, 0};
    DeviceAddress dsAddr[DS_COUNT][DS_MAX_PER_BUS];
    bool dsAddrKnown[DS_COUNT][DS_MAX_PER_BUS] = {{false, false, false}, {false, false, false}};
    uint8_t dsBusCount[DS_COUNT] = {0, 0};
    float dsBusValues[DS_COUNT][DS_MAX_PER_BUS] = {{NAN, NAN, NAN}, {NAN, NAN, NAN}};

    float pressureBuf[ADC_FILTER_SAMPLES] = {0};
    float flowBuf[ADC_FILTER_SAMPLES] = {0};
    uint8_t pressureBufIdx = 0, flowBufIdx = 0;
    bool pressureBufFull = false, flowBufFull = false;
    float pressureRawMa = NAN, flowRawMa = NAN;

    // Filter pressure sensor
    float filterBuf[10] = {0};  // max 10 samples
    uint8_t filterBufIdx = 0;
    bool filterBufFull = false;
    float filterRawMa = NAN;
    float filterCalOffset = 0, filterCalScale = 1;
    unsigned long lastFilterPollMs = 0;
    unsigned long filterAlarmStartMs = 0;
    bool filterAlarmLatched = false;
    float filterAlarmThreshold = Defaults::filter_alarm_threshold;

    float pressureCalOffset = 0, pressureCalScale = 1;
    float flowCalOffset = 0, flowCalScale = 1;

    RoomData rooms[ROOM_COUNT];

    float adsVoltsToMa(int16_t raw);
    float filteredAverage(float* buf, uint8_t &idx, bool &full, float newVal);
    void startDsConversion();
    void finishDsConversion();
    bool isValidDuctTemp(float t);
    bool readDsValue(DsSensorIdx idx, DallasTemperature &bus);
    bool validateBusReadings(DsSensorIdx idx);
    float computeBusAverage(DsSensorIdx idx);
};

extern Sensors sensors;
