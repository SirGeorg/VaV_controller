#include "LedIndicator.h"

LedIndicator ledIndicator;

void LedIndicator::begin() {
    strip.Begin();
    strip.SetBrightness(50);  // 50% яркость по умолчанию
    turnOff();
}

void LedIndicator::setStaticColor(uint8_t r, uint8_t g, uint8_t b) {
    useStaticColor = true;
    staticR = r;
    staticG = g;
    staticB = b;
    setColor(r, g, b);
}

void LedIndicator::clearStaticColor() {
    useStaticColor = false;
}

void LedIndicator::setColor(uint8_t r, uint8_t g, uint8_t b) {
    if (useStaticColor) return;  // не переопределять статический цвет
    RgbColor color(r, g, b);
    strip.SetPixelColor(0, color);
    strip.Show();
    ledOn = true;
}

void LedIndicator::turnOff() {
    if (useStaticColor) return;
    RgbColor color(0, 0, 0);
    strip.SetPixelColor(0, color);
    strip.Show();
    ledOn = false;
}

uint8_t LedIndicator::calculateFanPowerRed() {
    // Мощность вентилятора: 0% = зелёный (0), 100% = красный (255)
    float fanPwm = fan.pwmPct();
    return (uint8_t)(fanPwm * 2.55f);  // 0-100 -> 0-255
}

void LedIndicator::triggerHeaterPulseSequence(uint8_t stages) {
    if (stages == 0) return;
    heaterPulseCount = stages;
    heaterPulsesSent = 0;
    heaterSequenceActive = true;
    heaterSequenceStartMs = millis();
}

void LedIndicator::triggerFilterDoublePulse() {
    filterDoublePulseCount = 2;  // двойное мигание
    filterPulsesSent = 0;
    filterSequenceActive = true;
    filterSequenceStartMs = millis();
}

void LedIndicator::updateRunningMode() {
    unsigned long now = millis();
    
    // Проверка на срабатывание последовательности миганий нагревателя (раз в 10 сек)
    static unsigned long lastSequenceMs = 0;
    if (!heaterSequenceActive && !filterSequenceActive) {
        if (now - lastSequenceMs >= 10000UL) {
            lastSequenceMs = now;
            uint8_t stages = heater.currentStage();
            if (stages > 0) {
                triggerHeaterPulseSequence(stages);
            }
            // Проверка ошибки фильтра
            if (sensors.filterAlarmActive()) {
                triggerFilterDoublePulse();
            }
        }
    }
    
    // Обработка последовательности миганий нагревателя
    if (heaterSequenceActive) {
        if (now - heaterSequenceStartMs >= heaterPulseIntervalMs) {
            heaterSequenceStartMs = now;
            if (heaterPulsesSent < heaterPulseCount * 2) {
                // Чередование вкл/выкл
                if (heaterPulsesSent % 2 == 0) {
                    // Вкл: цвет от мощности вентилятора
                    uint8_t r = calculateFanPowerRed();
                    uint8_t g = 255 - r;
                    setColor(r, g, 0);
                } else {
                    turnOff();
                }
                heaterPulsesSent++;
            } else {
                heaterSequenceActive = false;
                // Возврат к обычному цвету
                uint8_t r = calculateFanPowerRed();
                uint8_t g = 255 - r;
                setColor(r, g, 0);
            }
            return;
        }
    }
    
    // Обработка последовательности миганий фильтра (приоритет выше нагревателя)
    if (filterSequenceActive) {
        if (now - filterSequenceStartMs >= filterPulseIntervalMs) {
            filterSequenceStartMs = now;
            if (filterPulsesSent < filterDoublePulseCount * 2) {
                if (filterPulsesSent % 2 == 0) {
                    // Фиолетовый (красный + синий)
                    setColor(180, 0, 180);
                } else {
                    turnOff();
                }
                filterPulsesSent++;
            } else {
                filterSequenceActive = false;
                // Возврат к обычному цвету
                uint8_t r = calculateFanPowerRed();
                uint8_t g = 255 - r;
                setColor(r, g, 0);
            }
        }
        return;
    }
    
    // Обычный режим работы: цвет от мощности вентилятора
    uint8_t r = calculateFanPowerRed();
    uint8_t g = 255 - r;
    setColor(r, g, 0);
}

void LedIndicator::updateFaultMode() {
    // Быстрое мигание синим (критическая ошибка)
    unsigned long now = millis();
    const unsigned long blinkInterval = 200;  // 5 Гц
    
    if (now - lastBlinkMs >= blinkInterval) {
        lastBlinkMs = now;
        if (ledOn) {
            turnOff();
        } else {
            setColor(0, 0, 255);  // Синий
        }
    }
}

void LedIndicator::updateFilterAlertMode() {
    // Двойное мигание фиолетовым (ошибка фильтра)
    unsigned long now = millis();
    
    if (!filterSequenceActive) {
        triggerFilterDoublePulse();
    }
    
    if (filterSequenceActive) {
        if (now - filterSequenceStartMs >= filterPulseIntervalMs) {
            filterSequenceStartMs = now;
            if (filterPulsesSent < filterDoublePulseCount * 2) {
                if (filterPulsesSent % 2 == 0) {
                    setColor(180, 0, 180);  // Фиолетовый
                } else {
                    turnOff();
                }
                filterPulsesSent++;
            } else {
                filterSequenceActive = false;
                // Пауза между повторами
                modeStartMs = now;
            }
        }
    } else {
        // Пауза 2 секунды перед повтором
        if (now - modeStartMs >= 2000UL) {
            triggerFilterDoublePulse();
        }
    }
}

void LedIndicator::updateIdleBlinkMode() {
    // Медленное плавное мигание зелёным (дыхание)
    unsigned long now = millis();
    const unsigned long breathDuration = 2000;  // 2 секунды полный цикл
    
    float progress = (float)(now % breathDuration) / breathDuration;
    // Плавная функция дыхания (синусоида)
    float brightness = (sin(progress * 2 * PI - PI/2) + 1) / 2;  // 0..1
    
    uint8_t g = (uint8_t)(brightness * 255);
    setColor(0, g, 0);
}

void LedIndicator::update() {
    if (useStaticColor) return;  // Статический цвет переопределяет всё
    
    bool fanFault = fan.faultLatched();
    bool heaterFault = heater.faultLatched();
    bool filterAlarm = sensors.filterAlarmActive();
    bool systemOn = fan.isRunning() || fan.phase() == FanPhase::STARTING;
    bool inStarting = fan.phase() == FanPhase::STARTING;
    
    // Приоритеты режимов:
    // 1. Критическая ошибка (fan_fault или heater_fault)
    // 2. Ошибка фильтра (только если система работает)
    // 3. Работа (вентилятор запущен)
    // 4. Ожидание/запуск (медленное мигание)
    // 5. Выключено
    
    if (fanFault || heaterFault) {
        currentMode = LedMode::FAULT_BLINK;
        updateFaultMode();
    } else if (filterAlarm && systemOn) {
        currentMode = LedMode::FILTER_ALERT;
        updateFilterAlertMode();
    } else if (systemOn) {
        currentMode = LedMode::RUNNING_COLOR;
        updateRunningMode();
    } else if (inStarting || !systemOn) {
        // Заслонки позиционируются или система выключена
        currentMode = LedMode::IDLE_BLINK;
        updateIdleBlinkMode();
    } else {
        currentMode = LedMode::OFF;
        turnOff();
    }
}
