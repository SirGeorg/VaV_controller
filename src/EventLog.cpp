#include "EventLog.h"
#include <ArduinoJson.h>

EventLog eventLog;

void EventLog::begin() {
    head = 0;
    count = 0;
    memset(buf, 0, sizeof(buf));
}

void EventLog::syncTime() {
    configTime(TZ_OFFSET_SEC, 0, NTP_SERVER);
}

bool EventLog::timeIsSynced() {
    time_t now = time(nullptr);
    // после 2021-01-01 считаем что время синхронизировано
    if (now > 1609459200) { synced = true; }
    return synced;
}

void EventLog::add(const String &msg) {
    time_t now = timeIsSynced() ? time(nullptr) : 0;
    buf[head].ts = now;
    strncpy(buf[head].msg, msg.c_str(), sizeof(buf[head].msg) - 1);
    buf[head].msg[sizeof(buf[head].msg) - 1] = '\0';
    head = (head + 1) % EVENT_LOG_SIZE;
    if (count < EVENT_LOG_SIZE) count++;
}

String EventLog::toJson(uint8_t maxCount) {
    if (maxCount > count) maxCount = count;
    DynamicJsonDocument doc(256 + maxCount * 96);
    JsonArray arr = doc.to<JsonArray>();

    // Идём от самой новой записи к более старым
    int idx = (head == 0) ? (EVENT_LOG_SIZE - 1) : (head - 1);
    for (uint8_t i = 0; i < maxCount; i++) {
        JsonObject o = arr.createNestedObject();
        o["ts"] = (uint32_t)buf[idx].ts;
        o["msg"] = buf[idx].msg;
        idx = (idx == 0) ? (EVENT_LOG_SIZE - 1) : (idx - 1);
    }
    String out;
    serializeJson(doc, out);
    return out;
}
