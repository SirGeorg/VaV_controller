#include "Dampers.h"

Dampers dampers;

void Dampers::begin() {
    for (uint8_t i = 0; i < ROOM_COUNT; i++) {
        uint16_t pmin, pmax;
        storage.getServoCal(i, pmin, pmax);
        servos[i].setPeriodHertz(50);
        servos[i].attach(PIN_SERVO[i], pmin, pmax);
        RoomSettings rs = storage.getRoom(i);
        state[i].commandedPos = rs.min_pos;
        state[i].targetPos = rs.min_pos;
        writeServoPercent(i, rs.min_pos);
    }
}

void Dampers::writeServoAngle(uint8_t idx, float angleDeg) {
    angleDeg = constrain(angleDeg, 0.0f, 180.0f);
    servos[idx].write((int)angleDeg);
    // Держим статус согласованным с фактическим углом
    state[idx].commandedPos = angleDeg / 180.0f * 100.0f;
}

void Dampers::writeServoPercent(uint8_t idx, float pct) {
    pct = constrain(pct, 0.0f, 100.0f);
    writeServoAngle(idx, pct / 100.0f * 180.0f);
    state[idx].commandedPos = pct;
}

void Dampers::setFreecoolAllowed(bool allowed) { freecoolAllowed_ = allowed; }

void Dampers::setManualMode(uint8_t idx, bool enable) {
    if (idx >= ROOM_COUNT) return;
    state[idx].manualMode = enable;
}

void Dampers::setManualPos(uint8_t idx, float pos) {
    if (idx >= ROOM_COUNT) return;
    RoomSettings rs = storage.getRoom(idx);
    state[idx].manualPos = constrain(pos, rs.min_pos, rs.max_pos);
}

void Dampers::serviceSetAngle(uint8_t idx, float angleDeg) {
    if (idx >= ROOM_COUNT) return;
    writeServoAngle(idx, angleDeg);
}

void Dampers::serviceSetPercent(uint8_t idx, float pct) {
    if (idx >= ROOM_COUNT) return;
    writeServoPercent(idx, pct);   // 0..100 %, обновляет commandedPos
}

void Dampers::calibrateServo(uint8_t idx, uint16_t pulseMinUs, uint16_t pulseMaxUs) {
    if (idx >= ROOM_COUNT) return;
    storage.setServoCal(idx, pulseMinUs, pulseMaxUs);
    servos[idx].detach();
    servos[idx].attach(PIN_SERVO[idx], pulseMinUs, pulseMaxUs);
}

float Dampers::getPos(uint8_t idx) { return (idx < ROOM_COUNT) ? state[idx].commandedPos : 0; }
float Dampers::getMaxPos(uint8_t idx) { return (idx < ROOM_COUNT) ? storage.getRoom(idx).max_pos : 100; }
RoomBlockReason Dampers::getBlockReason(uint8_t idx) { return (idx < ROOM_COUNT) ? state[idx].blockReason : RoomBlockReason::NONE; }
bool Dampers::isManual(uint8_t idx) { return (idx < ROOM_COUNT) ? state[idx].manualMode : false; }
float Dampers::getManualPos(uint8_t idx) { return (idx < ROOM_COUNT) ? state[idx].manualPos : 0; }

bool Dampers::anyFreecoolActive() {
    for (uint8_t i = 0; i < ROOM_COUNT; i++) {
        if (storage.getRoom(i).temp_mode_enable) return true;
    }
    return false;
}

float Dampers::maxPosAcrossRooms() {
    float m = 0;
    for (uint8_t i = 0; i < ROOM_COUNT; i++) {
        if (state[i].commandedPos > m) m = state[i].commandedPos;
    }
    return m;
}

void Dampers::update(float dtSeconds, bool systemOn, bool serviceMode) {
    const float co2Deadband = storage.getCo2Deadband();
    const float co2AlarmThreshold = storage.getCo2AlarmThreshold();
    const float minServoStepDeg = storage.getMinServoStep();
    const float minServoStepPct = minServoStepDeg / 180.0f * 100.0f;
    const float freecoolDeadband = storage.getTempFreecoolDeadband();

    if (serviceMode) {
        // Сервисный режим: не пересчитываем цели CO2/управления, а лишь
        // периодически призываем серво к последней команде (удерживаем позицию).
        for (uint8_t i = 0; i < ROOM_COUNT; i++) {
            writeServoPercent(i, state[i].commandedPos);
        }
        return;
    }

    for (uint8_t i = 0; i < ROOM_COUNT; i++) {
        RoomSettings rs = storage.getRoom(i);
        RoomState &s = state[i];
        RoomData rd = sensors.getRoomData(i);

        const bool co2Fresh = sensors.isRoomCo2Fresh(i);
        const bool tempFresh = sensors.isRoomTempFresh(i);

        float target;

        if (!systemOn) {
            target = rs.min_pos;
            s.blockReason = RoomBlockReason::NONE;
            s.piIntegral = 0;
            s.freecoolAdditive = 0;
        } else if (s.manualMode) {
            // Ручной режим имеет приоритет над CO2-форсажем и отсутствием данных.
            target = constrain(s.manualPos, rs.min_pos, rs.max_pos);
            s.blockReason = RoomBlockReason::NONE;
        } else if (co2Fresh && rd.co2 >= co2AlarmThreshold) {
            // Аварийный форсаж зависит только от свежести CO2.
            target = rs.max_pos;
            s.blockReason = RoomBlockReason::CO2_EMERGENCY;
        } else if (!co2Fresh && !tempFresh) {
            // Нет свежих показаний обоих типов.
            target = rs.min_pos;
            s.blockReason = RoomBlockReason::NO_DATA;
            s.piIntegral = 0;
            s.freecoolAdditive = 0;
        } else {
            // CO2-регулирование работает только по свежему CO2.
            float basePos = 0.0f;
            if (co2Fresh) {
                const float error = rd.co2 - rs.co2_target;
                float effError = 0.0f;
                if (fabs(error) > co2Deadband) {
                    effError = (error > 0) ? (error - co2Deadband) : (error + co2Deadband);
                }
                s.piIntegral += effError * Defaults::co2_ki * dtSeconds;
                s.piIntegral = constrain(s.piIntegral, 0.0f, 100.0f);
                basePos = Defaults::co2_kp * effError + s.piIntegral;
                basePos = constrain(basePos, 0.0f, 100.0f);
            } else {
                // Не интегрируем устаревший CO2.
                s.piIntegral = 0;
            }

            // Фрикулинг использует только свежую температуру.
            float additive = s.freecoolAdditive;
            if (rs.temp_mode_enable && freecoolAllowed_ && tempFresh) {
                const float deltaT = rd.temp - rs.temp_target;
                if (fabs(deltaT - s.lastFreecoolDeltaT) >= freecoolDeadband) {
                    additive = constrain(deltaT * 5.0f, 0.0f, 100.0f - basePos);
                    s.lastFreecoolDeltaT = deltaT;
                    s.freecoolAdditive = additive;
                }
            } else {
                additive = 0;
                s.freecoolAdditive = 0;
            }

            target = constrain(basePos + additive, rs.min_pos, rs.max_pos);
            s.blockReason = RoomBlockReason::NONE;
        }

        s.targetPos = target;
        if (fabs(target - s.commandedPos) >= minServoStepPct) {
            writeServoPercent(i, target);
        }
    }
}
