#include "Sensors.h"
#include "EventLog.h"
#include <Wire.h>

Sensors sensors;

void Sensors::begin() {
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    dsOutdoor.begin();
    dsOutdoor.setWaitForConversion(false);
    dsSupply.begin();
    dsSupply.setWaitForConversion(false);

    uint8_t outdoorCount = dsOutdoor.getDeviceCount();
    for (uint8_t i = 0; i < DS_MAX_PER_BUS && i < outdoorCount; i++) {
        if (dsOutdoor.getAddress(dsAddr[DS_OUTDOOR][i], i)) {
            dsAddrKnown[DS_OUTDOOR][i] = true;
            dsOutdoor.setResolution(dsAddr[DS_OUTDOOR][i], 12);
            dsBusCount[DS_OUTDOOR]++;
        }
    }

    uint8_t supplyCount = dsSupply.getDeviceCount();
    for (uint8_t i = 0; i < DS_MAX_PER_BUS && i < supplyCount; i++) {
        if (dsSupply.getAddress(dsAddr[DS_SUPPLY][i], i)) {
            dsAddrKnown[DS_SUPPLY][i] = true;
            dsSupply.setResolution(dsAddr[DS_SUPPLY][i], 12);
            dsBusCount[DS_SUPPLY]++;
        }
    }

    adsOk = ads.begin(ADS1115_ADDR);
    if (adsOk) ads.setGain(GAIN_ONE);

    storage.getAnalogCal("dp", pressureCalOffset, pressureCalScale);
    storage.getAnalogCal("flow", flowCalOffset, flowCalScale);
    storage.getAnalogCal("filter", filterCalOffset, filterCalScale);
    filterAlarmThreshold = storage.getFilterAlarmThreshold();
    startDsConversion();
}

void Sensors::startDsConversion() {
    dsOutdoor.requestTemperatures();
    dsSupply.requestTemperatures();
    dsConvertStartMs = millis();
    dsState = DsState::CONVERTING;
}

bool Sensors::readDsValue(DsSensorIdx idx, DallasTemperature &bus) {
    if (dsBusCount[idx] == 0) return false;

    float validValues[DS_MAX_PER_BUS];
    uint8_t validCount = 0;
    for (uint8_t i = 0; i < dsBusCount[idx]; i++) {
        if (!dsAddrKnown[idx][i]) continue;
        float t = bus.getTempC(dsAddr[idx][i]);
        if (t > -100.0f && isValidDuctTemp(t)) {
            dsBusValues[idx][i] = t;
            validValues[validCount++] = t;
        }
    }

    if (validCount == 0) return false;
    if (!validateBusReadings(idx)) {
        String busName = (idx == DS_OUTDOOR) ? "outdoor" : "supply";
        eventLog.add("DS18B20: bus " + busName + " rejected, spread > " + String(DS_VALUE_TOLERANCE_C, 1) + "C");
        return false;
    }

    dsTemps[idx] = computeBusAverage(idx);
    dsLastGoodMs[idx] = millis();
    return true;
}

bool Sensors::validateBusReadings(DsSensorIdx idx) {
    uint8_t count = 0;
    float first = NAN;
    for (uint8_t i = 0; i < dsBusCount[idx]; i++) {
        if (isnan(dsBusValues[idx][i])) continue;
        if (count == 0) first = dsBusValues[idx][i];
        count++;
        if (count > 1 && fabsf(dsBusValues[idx][i] - first) > DS_VALUE_TOLERANCE_C) return false;
    }
    return count > 0;
}

float Sensors::computeBusAverage(DsSensorIdx idx) {
    float sum = 0.0f;
    uint8_t count = 0;
    for (uint8_t i = 0; i < dsBusCount[idx]; i++) {
        if (isnan(dsBusValues[idx][i])) continue;
        sum += dsBusValues[idx][i];
        count++;
    }
    return count > 0 ? (sum / count) : NAN;
}

void Sensors::finishDsConversion() {
    readDsValue(DS_OUTDOOR, dsOutdoor);
    readDsValue(DS_SUPPLY, dsSupply);
    dsState = DsState::IDLE;
    dsLastCycleMs = millis();
}

bool Sensors::isValidDuctTemp(float t) {
    return t >= Valid::duct_temp_min && t <= Valid::duct_temp_max;
}

void Sensors::update() {
    if (dsState == DsState::CONVERTING) {
        if (millis() - dsConvertStartMs >= 750) finishDsConversion();
    } else if (millis() - dsLastCycleMs >= 1000) {
        startDsConversion();
    }

    if (adsOk) {
        int16_t rawP = ads.readADC_SingleEnded(ADS_CH_PRESSURE);
        int16_t rawF = ads.readADC_SingleEnded(ADS_CH_FLOW);
        pressureRawMa = adsVoltsToMa(rawP);
        flowRawMa = adsVoltsToMa(rawF);
        filteredAverage(pressureBuf, pressureBufIdx, pressureBufFull, pressureRawMa);
        filteredAverage(flowBuf, flowBufIdx, flowBufFull, flowRawMa);
    }

    // --- Filter pressure sensor: poll every 10 seconds ---
    unsigned long now = millis();
    if (adsOk && (now - lastFilterPollMs >= Defaults::filter_poll_interval_ms)) {
        lastFilterPollMs = now;
        int16_t rawFilt = ads.readADC_SingleEnded(ADS_CH_FILTER);
        filterRawMa = adsVoltsToMa(rawFilt);
        filteredAverage(filterBuf, filterBufIdx, filterBufFull, filterRawMa);
    }
}

float Sensors::adsVoltsToMa(int16_t raw) {
    float volts = ads.computeVolts(raw);
    if (volts < 0.0f || volts > ADS_MAX_INPUT_V) return NAN;
    return (volts / SHUNT_OHM) * 1000.0f;
}

float Sensors::filteredAverage(float* buf, uint8_t &idx, bool &full, float newVal) {
    buf[idx] = newVal;
    idx = (idx + 1) % ADC_FILTER_SAMPLES;
    if (idx == 0) full = true;
    uint8_t n = full ? ADC_FILTER_SAMPLES : idx;
    if (n == 0) return newVal;
    float sum = 0;
    for (uint8_t i = 0; i < n; i++) sum += buf[i];
    return sum / n;
}

bool Sensors::outdoorTempValid() {
    return dsAddrKnown[DS_OUTDOOR] && !isnan(dsTemps[DS_OUTDOOR]) &&
           (millis() - dsLastGoodMs[DS_OUTDOOR] < Defaults::duct_sensor_timeout_s * 1000UL);
}
float Sensors::outdoorTemp() { return dsTemps[DS_OUTDOOR]; }

bool Sensors::supplyTempValid() {
    return dsAddrKnown[DS_SUPPLY] && !isnan(dsTemps[DS_SUPPLY]) &&
           (millis() - dsLastGoodMs[DS_SUPPLY] < Defaults::duct_sensor_timeout_s * 1000UL);
}
float Sensors::supplyTemp() { return dsTemps[DS_SUPPLY]; }

bool Sensors::ductSensorsFresh() {
    return outdoorTempValid() && supplyTempValid();
}

float Sensors::pressurePa() {
    float avgMa = pressureRawMa;
    uint8_t n = pressureBufFull ? ADC_FILTER_SAMPLES : pressureBufIdx;
    if (n > 0) {
        float sum = 0;
        for (uint8_t i = 0; i < n; i++) sum += pressureBuf[i];
        avgMa = sum / n;
    }
    return pressureCalOffset + pressureCalScale * (avgMa - 4.0f);
}

bool Sensors::pressureSensorOk() {
    return !isnan(pressureRawMa) && pressureRawMa >= LOOP_BREAK_MA;
}

float Sensors::flowPct() {
    float avgMa = flowRawMa;
    uint8_t n = flowBufFull ? ADC_FILTER_SAMPLES : flowBufIdx;
    if (n > 0) {
        float sum = 0;
        for (uint8_t i = 0; i < n; i++) sum += flowBuf[i];
        avgMa = sum / n;
    }
    float pct = (avgMa - 4.0f) / 16.0f * 100.0f;
    return constrain(pct, 0.0f, 100.0f);
}

bool Sensors::flowSensorOk() {
    return !isnan(flowRawMa) && flowRawMa >= LOOP_BREAK_MA;
}

// --- Filter pressure sensor ---
float Sensors::filterPressurePa() {
    float avgMa = filterRawMa;
    uint8_t n = filterBufFull ? Defaults::filter_avg_samples : filterBufIdx;
    if (n > 0) {
        float sum = 0;
        for (uint8_t i = 0; i < n; i++) sum += filterBuf[i];
        avgMa = sum / n;
    }
    return filterCalOffset + filterCalScale * (avgMa - 4.0f);
}

bool Sensors::filterSensorOk() {
    return !isnan(filterRawMa) && filterRawMa >= LOOP_BREAK_MA;
}

bool Sensors::filterAlarmActive() {
    if (!filterSensorOk()) {
        // If sensor is not OK, clear any pending/latched alarm — the hardware signal is invalid
        filterAlarmLatched = false;
        filterAlarmStartMs = 0;
        return false;
    }
    
    float pressure = filterPressurePa();
    bool exceeded = pressure > filterAlarmThreshold;
    
    if (exceeded) {
        if (filterAlarmStartMs == 0) {
            filterAlarmStartMs = millis();
        }
        if (millis() - filterAlarmStartMs >= Defaults::filter_alarm_duration_s * 1000UL) {
            filterAlarmLatched = true;
        }
    } else {
        filterAlarmStartMs = 0;
    }
    
    return filterAlarmLatched;
}

void Sensors::resetFilterAlarm() {
    filterAlarmLatched = false;
    filterAlarmStartMs = 0;
    eventLog.add("Filter: alarm reset manually");
}

void Sensors::setFilterAlarmThreshold(float v) {
    filterAlarmThreshold = v;
    storage.setFilterAlarmThreshold(v);
}

float Sensors::getFilterAlarmThreshold() {
    return filterAlarmThreshold;
}

void Sensors::setFilterCal(float offset, float scale) {
    filterCalOffset = offset;
    filterCalScale = scale;
    storage.setAnalogCal("filter", offset, scale);
}

void Sensors::getFilterCal(float &offset, float &scale) {
    offset = filterCalOffset;
    scale = filterCalScale;
}

void Sensors::setRoomData(uint8_t idx, float co2, float temp) {
    if (idx >= ROOM_COUNT) return;
    const unsigned long now = millis();

    bool co2Valid = !isnan(co2) &&
                    co2 >= Valid::co2_min &&
                    co2 <= Valid::co2_max;
    bool tempValid = !isnan(temp) &&
                     temp >= Valid::room_temp_min &&
                     temp <= Valid::room_temp_max;

    // Версия 4: каждое корректное показание обновляется независимо.
    if (co2Valid) {
        rooms[idx].co2 = co2;
        rooms[idx].lastCo2UpdateMs = now;
        rooms[idx].co2EverReceived = true;
    }
    if (tempValid) {
        rooms[idx].temp = temp;
        rooms[idx].lastTempUpdateMs = now;
        rooms[idx].tempEverReceived = true;
    }
}

RoomData Sensors::getRoomData(uint8_t idx) {
    if (idx >= ROOM_COUNT) return RoomData();
    return rooms[idx];
}

bool Sensors::isRoomCo2Fresh(uint8_t idx) {
    if (idx >= ROOM_COUNT || !rooms[idx].co2EverReceived) return false;
    const uint32_t timeoutMs = storage.getRoomDataTimeout() * 1000UL;
    return (millis() - rooms[idx].lastCo2UpdateMs) < timeoutMs;
}

bool Sensors::isRoomTempFresh(uint8_t idx) {
    if (idx >= ROOM_COUNT || !rooms[idx].tempEverReceived) return false;
    const uint32_t timeoutMs = storage.getRoomDataTimeout() * 1000UL;
    return (millis() - rooms[idx].lastTempUpdateMs) < timeoutMs;
}

bool Sensors::isRoomDataFresh(uint8_t idx) {
    return isRoomCo2Fresh(idx) && isRoomTempFresh(idx);
}

void Sensors::setPressureCal(float offset, float scale) {
    pressureCalOffset = offset; pressureCalScale = scale;
    storage.setAnalogCal("dp", offset, scale);
}
void Sensors::setFlowCal(float offset, float scale) {
    flowCalOffset = offset; flowCalScale = scale;
    storage.setAnalogCal("flow", offset, scale);
}
void Sensors::getPressureCal(float &offset, float &scale) {
    offset = pressureCalOffset; scale = pressureCalScale;
}
void Sensors::getFlowCal(float &offset, float &scale) {
    offset = flowCalOffset; scale = flowCalScale;
}
