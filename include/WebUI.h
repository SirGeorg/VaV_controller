#pragma once
#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include "Config.h"
#include "Storage.h"
#include "Sensors.h"
#include "Dampers.h"
#include "Fan.h"
#include "Heater.h"
#include "EventLog.h"

// ============================================================
//  WebUI — локальный веб-интерфейс:
//   - статус (JSON + простая HTML-страница)
//   - калибровка сервоприводов и датчиков 4-20мА (п.6)
//   - экспорт/импорт конфигурации (п.12)
//   - версия прошивки (п.13)
//  Wi-Fi настраивается через WiFiManager (AP "Vent-Setup"),
//  mDNS-имя "vent.local".
// ============================================================

class WebUI {
public:
    void begin();     // поднимает WiFiManager (если нет сохранённых credentials), mDNS, сервер
    void loop();       // ArduinoOTA.handle()

private:
    AsyncWebServer server_{80};
    void setupRoutes();
};

extern WebUI webUi;
