#pragma once
#include <Arduino.h>

// ============================================================
//  Vent Controller v4.2.3 — общая конфигурация, пины, константы
// ============================================================

#define FW_VERSION      "4.2.3"
#define FW_BUILD_DATE   __DATE__ " " __TIME__

// ---------- Сеть ----------
#define WIFI_AP_NAME        "Vent-Setup"
#define MDNS_HOSTNAME        "vent"
#define NTP_SERVER           "pool.ntp.org"
#define TZ_OFFSET_SEC        (3 * 3600)
#define OTA_HOSTNAME         "vent-controller"

// ---------- MQTT ----------
#define MQTT_ROOT             "vent"
#define MQTT_CLIENT_ID        "vent-controller"
// Дефолтные параметры MQTT-брокера (задайте свои; переопределяются через веб-UI)
#define MQTT_SERVER           "192.168.1.222"
#define MQTT_PORT             1883
#define MQTT_USER             "mos"
#define MQTT_PASS             "mosmos"
#define MQTT_KEEPALIVE_S      30
#define MQTT_RECONNECT_MS     5000

// ---------- Число комнат ----------
#define ROOM_COUNT            4

// ---------- Пины сервоприводов заслонок (по комнатам) ----------
static const uint8_t PIN_SERVO[ROOM_COUNT] = {39, 40, 41, 42};

// ---------- Пины реле нагревателя (5 ступеней) ----------
#define HEATER_STAGES         5
static const uint8_t PIN_RELAY[HEATER_STAGES] = {1, 2, 4, 5, 6};

// ---------- Пин управления вентилятором ----------
#define PIN_FAN_PWM           15
#define FAN_PWM_FREQ_HZ       1000
#define FAN_PWM_RES_BITS      10
// ВАЖНО: канал НЕ должен совпадать с автоматически выделяемыми ESP32Servo
// (серво занимают каналы 0,1,8,9). Используем свободный канал 2.
#define FAN_PWM_CHANNEL       2

// ---------- DS18B20 ----------
#define PIN_ONEWIRE_OUTDOOR   11
#define PIN_ONEWIRE_SUPPLY    10
#define DS_MAX_PER_BUS        3
#define DS_VALUE_TOLERANCE_C  3.0f
enum DsSensorIdx {
    DS_OUTDOOR = 0,
    DS_SUPPLY  = 1,
    DS_COUNT
};

// ---------- I2C / ADS1115 ----------
#define PIN_I2C_SDA           8
#define PIN_I2C_SCL           9
#define ADS1115_ADDR          0x48
#define ADS_CH_PRESSURE       0
#define ADS_CH_FLOW           1
#define ADS_CH_FILTER         2
#define ADS_VDD_VOLTAGE       3.3f
#define ADS_MAX_INPUT_V       (ADS_VDD_VOLTAGE - 0.1f)
#define SHUNT_OHM             120.0f
#define ADC_FILTER_SAMPLES    8

namespace Defaults {
    const bool     winter_mode           = false;
    const float    co2_target            = 800.0f;
    const float    co2_kp                = 0.05f;
    const float    co2_ki                = 0.002f;
    const float    co2_deadband          = 100.0f;
    const float    co2_alarm_threshold   = 3000.0f;
    const float    min_pos               = 10.0f;
    const float    max_pos               = 100.0f;
    const float    min_servo_step        = 10.0f;
    const uint32_t room_data_timeout_s   = 600;

    const float    temp_target           = 22.0f;
    const float    temp_freecool_deadband= 2.0f;
    const float    freecool_outdoor_min  = 10.0f;

    const float    dp_min                = 50.0f;
    const float    dp_max                = 200.0f;
    const float    dp_kp                 = 0.8f;
    const float    dp_ki                 = 0.15f;
    const float    dp_kd                 = 0.0f;
    const float    dp_alarm_low          = 20.0f;
    const float    dp_alarm_high         = 300.0f;
    const uint32_t dp_low_timeout_s      = 180;
    const uint32_t dp_high_timeout_s     = 20;
    const float    min_fan_step          = 5.0f;
    const float    min_fan_speed         = 5.0f;
    const float    min_dp_step           = 10.0f;
    const uint32_t damper_settle_time_s  = 15;
    const uint32_t fan_ramp_time_s       = 10;
    const uint32_t fan_coastdown_time_s  = 180;
    const float    flow_alarm_threshold  = 30.0f;
    const float    dp_within_setpoint_pct= 20.0f;

    const float    heater_target_temp    = 20.0f;
    const float    heater_deadband_low   = 5.0f;
    const float    heater_deadband_high  = 3.0f;
    const uint32_t heater_stage_up_delay_s   = 60;
    const uint32_t heater_stage_down_delay_s = 5;
    const float    heater_alarm_temp     = 60.0f;
    const float    heater_outdoor_block_temp = 18.0f;
    const uint32_t heater_min_fan_runtime_s  = 180;

    const uint32_t duct_sensor_timeout_s = 120;
    const uint32_t sensor_grace_s        = 3;   // короткий грэйс перед аварией отсутствия датчика

    // --- Filter pressure sensor ---
    const float    filter_alarm_threshold  = 300.0f;  // Pa, авария если превышено > 1 мин
    const uint32_t filter_poll_interval_ms = 10000;   // опрос раз в 10 сек
    const uint8_t  filter_avg_samples      = 5;       // скользящее среднее
    const uint32_t filter_alarm_duration_s = 60;      // длительность превышения для аварии
}

namespace Valid {
    const float co2_min = 0.0f, co2_max = 5000.0f;
    const float duct_temp_min = -40.0f, duct_temp_max = 120.0f;
    const float room_temp_min = -20.0f, room_temp_max = 50.0f;
}

#define SERVICE_CMD_TIMEOUT_MS   30000UL
#define EVENT_LOG_SIZE           50
#define EVENT_LOG_PUBLISH_COUNT  15
#define LOOP_BREAK_MA            3.5f
