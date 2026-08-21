#pragma once
#include <Arduino.h>
#include <NeoPixelBrightnessBus.h>
#include "Config.h"
#include "Storage.h"
#include "Sensors.h"
#include "Fan.h"
#include "Heater.h"
#include "Dampers.h"

// ============================================================
//  LedIndicator — адресный светодиод NeoPixel на GPIO 48.
//  Индикация режимов работы:
//   - Цвет от зелёного к красному пропорционально мощности вентилятора
//   - Во время работы: раз в 10 сек быстрые мигания по количеству ступеней нагревателя
//   - При остановке/ожидании: медленное мигание зелёным
//   - Критическая ошибка (fan_fault/heater_fault): быстрое мигание синим
//   - Ошибка фильтра: двойное мигание фиолетовым в режиме работы
// ============================================================

class LedIndicator {
public:
    void begin();
    void update();
    
    // Принудительная установка цвета (для сервисного режима)
    void setStaticColor(uint8_t r, uint8_t g, uint8_t b);
    void clearStaticColor();
    
private:
    static const uint8_t LED_PIN = 48;
    static const uint8_t LED_COUNT = 1;
    
    NeoPixelBrightnessBus<NeoGrbFeature, Neo800KbpsMethod> strip{LED_COUNT, LED_PIN};
    
    enum class LedMode : uint8_t {
        OFF,
        IDLE_BLINK,           // медленное мигание зелёным (ожидание/заслонки)
        RUNNING_COLOR,        // цвет от мощности вентилятора + мигания нагревателя
        FAULT_BLINK,          // быстрое мигание синим (критическая ошибка)
        FILTER_ALERT          // двойное мигание фиолетовым (ошибка фильтра)
    };
    
    LedMode currentMode = LedMode::OFF;
    unsigned long modeStartMs = 0;
    unsigned long lastBlinkMs = 0;
    bool ledOn = false;
    
    // Для плавного дыхания в режиме ожидания
    unsigned long breathStartMs = 0;
    
    // Статический цвет (переопределяет автоматический режим)
    bool useStaticColor = false;
    uint8_t staticR = 0, staticG = 0, staticB = 0;
    
    // Счётчик миганий нагревателя
    uint8_t heaterPulseCount = 0;
    uint8_t heaterPulsesSent = 0;
    unsigned long heaterPulseIntervalMs = 150;  // быстрое мигание ~7 Гц
    unsigned long heaterSequenceStartMs = 0;
    bool heaterSequenceActive = false;
    
    // Счётчик двойных миганий фильтра
    uint8_t filterDoublePulseCount = 0;
    uint8_t filterPulsesSent = 0;
    unsigned long filterPulseIntervalMs = 120;
    unsigned long filterSequenceStartMs = 0;
    bool filterSequenceActive = false;
    
    void setColor(uint8_t r, uint8_t g, uint8_t b);
    void turnOff();
    uint8_t calculateFanPowerRed();
    void updateRunningMode();
    void updateFaultMode();
    void updateFilterAlertMode();
    void updateIdleBlinkMode();
    void triggerHeaterPulseSequence(uint8_t stages);
    void triggerFilterDoublePulse();
};

extern LedIndicator ledIndicator;
