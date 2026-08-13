#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "Config.h"
#include "Storage.h"
#include "Sensors.h"
#include "Dampers.h"
#include "Fan.h"
#include "Heater.h"
#include "EventLog.h"

// ============================================================
//  MqttManager — вся интеграция с MQTT: приём команд, публикация
//  состояния, Home Assistant discovery, сервисный тестовый режим.
// ============================================================

struct MqttManagerTrampolineHelper;

class MqttManager {
public:
    void begin(const char* server, uint16_t port, const char* user, const char* pass);
    void loop();

    bool powerOn() const { return powerOn_; }
    bool serviceMode() const { return serviceMode_; }

    void publishState();          // раз в N секунд — общее состояние
    void publishHaDiscovery();

private:
    friend struct MqttManagerTrampolineHelper;

    WiFiClient wifiClient_;
    PubSubClient client_{wifiClient_};
    String mqttUser_, mqttPass_;
    unsigned long lastReconnectAttempt_ = 0;
    unsigned long lastStatePublishMs_ = 0;

    bool powerOn_ = false;          // всегда стартует false (п.4), не персистентно
    bool serviceMode_ = false;      // сброс при перезагрузке

    // сервисные тайм-ауты команд (п.11, 30с автосброс)
    unsigned long serviceRelayCmdMs_[HEATER_STAGES] = {0};
    unsigned long serviceServoCmdMs_[ROOM_COUNT] = {0};

    void onMessage(char* topic, uint8_t* payload, unsigned int len);
    void handleCommand(const String &topic, const String &payload);
    void subscribeAll();
    void serviceModeTick();   // проверка тайм-аутов сервисных команд

    void publishRetained(const String &topic, const String &payload);
};

extern MqttManager mqttManager;
