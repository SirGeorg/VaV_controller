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

    const char* src = msg.c_str();
    size_t n = strlen(src);
    size_t cap = sizeof(buf[head].msg) - 1;
    if (n > cap) n = cap;
    // Не режем многобайтовый UTF-8 символ: откатываемся до границы символа,
    // иначе в конце строки окажется «битый» байт.
    while (n > 1 && ((uint8_t)src[n] & 0xC0) == 0x80) n--;
    memcpy(buf[head].msg, src, n);
    buf[head].msg[n] = '\0';

    head = (head + 1) % EVENT_LOG_SIZE;
    if (count < EVENT_LOG_SIZE) count++;
}

String EventLog::toJson(uint8_t maxCount) {
    if (maxCount > count) maxCount = count;
    DynamicJsonDocument doc(256 + maxCount * 192);
    JsonArray arr = doc.to<JsonArray>();

    // Идём от самой новой записи к более старым
    int idx = (head == 0) ? (EVENT_LOG_SIZE - 1) : (head - 1);
    for (uint8_t i = 0; i < maxCount; i++) {
        JsonObject o = arr.createNestedObject();
        o["ts"] = (uint32_t)buf[idx].ts;
        // Человекочитаемое время в локальном поясе устройства (UTC+3)
        if (buf[idx].ts > 0) {
            struct tm tmv;
            localtime_r(&buf[idx].ts, &tmv);
            char tbuf[24] = {0};
            strftime(tbuf, sizeof(tbuf), "%d.%m.%Y %H:%M:%S", &tmv);
            o["time"] = tbuf;
        } else {
            o["time"] = "";
        }
        o["msg"] = buf[idx].msg;
        idx = (idx == 0) ? (EVENT_LOG_SIZE - 1) : (idx - 1);
    }
    String out;
    serializeJson(doc, out);
    return out;
}
