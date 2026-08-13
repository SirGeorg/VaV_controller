#pragma once
#include <Arduino.h>
#include <time.h>
#include "Config.h"

// ============================================================
//  EventLog — кольцевой буфер последних событий (п.8).
//  Метки времени берутся из системных часов (синхронизируются по NTP).
// ============================================================

struct LogEntry {
    time_t  ts;
    char    msg[64];
};

class EventLog {
public:
    void begin();
    void add(const String &msg);
    // Возвращает JSON-массив последних `count` событий (новые первыми)
    String toJson(uint8_t count = EVENT_LOG_PUBLISH_COUNT);
    void syncTime();      // запускает NTP синхронизацию
    bool timeIsSynced();

private:
    LogEntry buf[EVENT_LOG_SIZE];
    uint8_t head = 0;      // индекс следующей записи для перезаписи
    uint8_t count = 0;
    bool synced = false;
};

extern EventLog eventLog;
