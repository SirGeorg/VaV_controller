#pragma once
#include <Arduino.h>
#include <ESP32Servo.h>
#include "Config.h"
#include "Storage.h"
#include "Sensors.h"

// ============================================================
//  Dampers (RoomCtrl) — управление 4 заслонками:
//   - PI-регулирование по CO2 с зоной нечувствительности (co2_deadband)
//   - Надбавка фрикулинга с deadband 2°C
//   - Аварийный форсаж по CO2 >= co2_alarm_threshold -> max_pos,
//     игнорируя ручной режим
//   - Ручной режим: ограничен min_pos..max_pos
//   - Сервисный режим: без ограничений (прямой угол)
//   - min_servo_step — не двигать привод чаще, чем на X градусов
// ============================================================

enum class RoomBlockReason : uint8_t {
    NONE = 0,
    NO_DATA,          // нет свежих данных (min_pos по тайм-ауту)
    CO2_EMERGENCY     // аварийный форсаж
};

struct RoomState {
    float commandedPos = 0;     // % последняя реально выставленная позиция
    float targetPos = 0;        // % расчётная целевая позиция
    bool  manualMode = false;   // сброс при перезагрузке
    float manualPos = 0;
    RoomBlockReason blockReason = RoomBlockReason::NONE;
    float lastFreecoolDeltaT = 0; // для deadband фрикулинга
    float freecoolAdditive = 0;
    float piIntegral = 0;
};

class Dampers {
public:
    void begin();
    // systemOn=false (установка выключена/останавливается) -> все заслонки
    // безусловно идут в min_pos, независимо от CO2/ручного/аварийного режима
    void update(float dtSeconds, bool systemOn);

    // --- ручной режим (через MQTT) ---
    void setManualMode(uint8_t idx, bool enable);
    void setManualPos(uint8_t idx, float pos);      // клампится min..max

    // --- сервисный режим (напрямую, без ограничений) ---
    void serviceSetAngle(uint8_t idx, float angleDeg);
    void serviceSetPercent(uint8_t idx, float pct);   // для веб-управления в сервисном режиме

    // --- глобальный флаг разрешения фрикулинга (по уличной температуре) ---
    void setFreecoolAllowed(bool allowed);
    bool freecoolAllowed() { return freecoolAllowed_; }

    // --- геттеры для публикации состояния ---
    float getPos(uint8_t idx);            // текущая фактическая позиция %
    float getMaxPos(uint8_t idx);
    RoomBlockReason getBlockReason(uint8_t idx);
    bool  isManual(uint8_t idx);
    float getManualPos(uint8_t idx);  // ручная уставка % (для веб-интерфейса)

    // --- нужно для расчёта dp_setpoint (макс. позиция среди комнат) ---
    float maxPosAcrossRooms();

    // --- используется Heater'ом: активен ли фрикулинг хотя бы в одной комнате ---
    bool anyFreecoolActive();

    // --- калибровка серво ---
    void calibrateServo(uint8_t idx, uint16_t pulseMinUs, uint16_t pulseMaxUs);

private:
    Servo servos[ROOM_COUNT];
    RoomState state[ROOM_COUNT];
    bool freecoolAllowed_ = true;
    unsigned long lastUpdateMs = 0;

    void writeServoPercent(uint8_t idx, float pct);
    void writeServoAngle(uint8_t idx, float angleDeg);
};

extern Dampers dampers;
