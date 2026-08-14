#include <Arduino.h>
#include "Config.h"
#include "Storage.h"
#include "EventLog.h"
#include "Sensors.h"
#include "Dampers.h"
#include "Fan.h"
#include "Heater.h"
#include "MqttManager.h"
#include "WebUI.h"

// Параметры MQTT-брокера задаются в Config.h (MQTT_SERVER/PORT/USER/PASS)
// и могут быть переопределены через веб-UI (вкладка "MQTT").

static unsigned long lastDampersMs = 0;
static unsigned long lastFanHeaterMs = 0;
static bool lastPowerOn = false;

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("Vent Controller v" FW_VERSION " build " FW_BUILD_DATE);

    storage.begin();
    eventLog.begin();

    // Пункт 4: установка ВСЕГДА стартует в OFF независимо от того, что было
    // до перезагрузки. Автоматического возобновления работы нет.
    // (powerOn_ в MqttManager по умолчанию false, отдельно ничего восстанавливать не нужно)

    sensors.begin();
    dampers.begin();
    fan.begin();     // если в NVS была latched-авария fan_fault — восстановится автоматически (п.3)
    heater.begin();  // аналогично для heater_fault (проверяется в Heater::update через storage)

    webUi.begin();    // WiFiManager (AP "Vent-Setup"), mDNS ("vent.local"), веб-сервер, OTA
    eventLog.syncTime();  // запуск NTP синхронизации (UTC+3)

    // Параметры MQTT-брокера берём из NVS (сохраняются через веб-UI).
    // Если в NVS сервер/логин ещё не заданы — подставляем дефолты из Config.h.
    MqttConfig mcfg = storage.getMqttConfig();
    const char* mqttServer = mcfg.server[0] ? mcfg.server : MQTT_SERVER;
    uint16_t    mqttPort   = mcfg.port      ? mcfg.port   : MQTT_PORT;
    const char* mqttUser   = mcfg.user[0]   ? mcfg.user   : MQTT_USER;
    const char* mqttPass   = mcfg.pass[0]   ? mcfg.pass   : MQTT_PASS;

    Serial.print("[BOOT] MQTT server=");
    Serial.print(mqttServer);
    Serial.print(":");
    Serial.println(mqttPort);
    mqttManager.begin(mqttServer, mqttPort, mqttUser, mqttPass);

    eventLog.add("Система запущена, ожидание команды power=1");
}

void loop() {
    webUi.loop();
    mqttManager.loop();
    sensors.update();     // неблокирующий опрос DS18B20 + ADS1115, вызывать часто

    bool powerOn = mqttManager.powerOn();
    bool serviceMode = mqttManager.serviceMode();

    // ---- переключение power отслеживается тут же в main, доп. дёргать
    //      fan.requestOn()/Off() не нужно — это уже сделано в MqttManager::handleCommand ----
    lastPowerOn = powerOn;

    unsigned long now = millis();

    // ---- цикл заслонок / расчёта dp_setpoint — раз в 10 секунд ----
    if (now - lastDampersMs >= 10000) {
        float dt = (lastDampersMs == 0) ? 10.0f : (now - lastDampersMs) / 1000.0f;
        lastDampersMs = now;
        if (!serviceMode) {
            dampers.update(dt, powerOn);
            fan.recomputeSetpoint();   // dp_setpoint пересчитывается синхронно с позициями заслонок
        }
    }

    // ---- цикл вентилятора (контур давления) и нагревателя — раз в 1 секунду ----
    if (now - lastFanHeaterMs >= 1000) {
        float dt = (lastFanHeaterMs == 0) ? 1.0f : (now - lastFanHeaterMs) / 1000.0f;
        lastFanHeaterMs = now;
        if (!serviceMode) {
            fan.update(dt);
            heater.update(dt, powerOn);
        }
    }
}
