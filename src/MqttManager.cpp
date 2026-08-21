#include "MqttManager.h"
#include <ArduinoJson.h>

MqttManager mqttManager;

void MqttManager::onMessage(char* topic, uint8_t* payload, unsigned int len) {
    String p;
    p.reserve(len + 1);
    for (unsigned int i = 0; i < len; i++) p += (char)payload[i];
    handleCommand(String(topic), p);
}

struct MqttManagerTrampolineHelper {
    static void call(char* topic, uint8_t* payload, unsigned int len) {
        mqttManager.onMessage(topic, payload, len);
    }
};

void MqttManager::begin(const char* server, uint16_t port, const char* user, const char* pass) {
    mqttServer_ = server ? server : "";
    mqttUser_ = user ? user : "";
    mqttPass_ = pass ? pass : "";
    MqttConfig c = storage.getMqttConfig();
    clientId_ = c.client_id[0] ? c.client_id : MQTT_CLIENT_ID;
    root_     = c.root[0]      ? c.root      : MQTT_ROOT;
    // PubSubClient::setServer хранит указатель без копирования, поэтому
    // передаём стабильную строку-член класса (не локальную переменную).
    client_.setServer(mqttServer_.c_str(), port);
    client_.setBufferSize(1024);
    client_.setCallback(MqttManagerTrampolineHelper::call);
    Serial.print("[MQTT] init server=");
    Serial.print(server);
    Serial.print(":");
    Serial.println(port);
}

void MqttManager::setPower(bool on) {
    powerOn_ = on;
    storage.setPowerLast(on);
    if (on) { fan.requestOn();  eventLog.add("Power ON"); }
    else    { fan.requestOff(); eventLog.add("Power OFF"); }
}

void MqttManager::setServiceMode(bool on) {
    serviceMode_ = on;
    eventLog.add(on ? "Service mode ON" : "Service mode OFF");
}

void MqttManager::publishRetained(const String &topic, const String &payload) {
    if (!client_.publish(topic.c_str(), payload.c_str(), true)) {
        Serial.print("[MQTT] publish failed: ");
        Serial.println(topic);
    }
}

void MqttManager::subscribeAll() {
    bool ok1 = client_.subscribe((root_ + "/set/#").c_str());
    bool ok2 = client_.subscribe((root_ + "/room/+/set/#").c_str());
    bool ok3 = client_.subscribe((root_ + "/room/+/data/#").c_str());
    bool ok4 = client_.subscribe((root_ + "/service/#").c_str());
    bool ok5 = client_.subscribe((root_ + "/config/restore").c_str());
    bool ok6 = client_.subscribe((root_ + "/config/backup_request").c_str());
    if (!(ok1 && ok2 && ok3 && ok4 && ok5 && ok6)) Serial.println("[MQTT] one or more subscriptions failed");
}

void MqttManager::loop() {
    if (!client_.connected()) {
        unsigned long now = millis();
        if (now - lastReconnectAttempt_ >= MQTT_RECONNECT_MS) {
            lastReconnectAttempt_ = now;
            if (!WiFi.isConnected()) return;
            String willTopic = root_ + "/status";
            bool ok;
            if (mqttUser_.length()) {
                ok = client_.connect(clientId_.c_str(), mqttUser_.c_str(), mqttPass_.c_str(),
                                     willTopic.c_str(), 0, true, "offline");
            } else {
                ok = client_.connect(clientId_.c_str(), willTopic.c_str(), 0, true, "offline");
            }
            if (ok) {
                client_.publish(willTopic.c_str(), "online", true);
                subscribeAll();
                publishHaDiscovery();
                eventLog.add("MQTT: подключено");
            }
        }
    } else {
        client_.loop();
        serviceModeTick();
        if (millis() - lastStatePublishMs_ >= 5000) {
            lastStatePublishMs_ = millis();
            publishState();
        }
    }
}

void MqttManager::serviceModeTick() {
    if (!serviceMode_) return;
    unsigned long now = millis();
    for (uint8_t i = 0; i < HEATER_STAGES; i++) {
        if (serviceRelayCmdMs_[i] != 0 && now - serviceRelayCmdMs_[i] > SERVICE_CMD_TIMEOUT_MS) {
            digitalWrite(PIN_RELAY[i], LOW);
            serviceRelayCmdMs_[i] = 0;
        }
    }
    for (uint8_t i = 0; i < ROOM_COUNT; i++) {
        if (serviceServoCmdMs_[i] != 0 && now - serviceServoCmdMs_[i] > SERVICE_CMD_TIMEOUT_MS) {
            dampers.serviceSetAngle(i, 0);
            serviceServoCmdMs_[i] = 0;
        }
    }
}

void MqttManager::handleCommand(const String &topic, const String &payload) {
    String root = root_ + "/";
    if (!topic.startsWith(root)) return;
    String sub = topic.substring(root.length());

    if (sub.startsWith("room/")) {
        int secondSlash = sub.indexOf('/', 5);
        if (secondSlash < 0) return;
        int roomIdx = sub.substring(5, secondSlash).toInt() - 1;
        if (roomIdx < 0 || roomIdx >= ROOM_COUNT) return;
        String rest = sub.substring(secondSlash + 1);

        if (rest == "data" || rest == "data/co2" || rest == "data/temp") {
            if (rest == "data") {
                DynamicJsonDocument doc(256);
                if (deserializeJson(doc, payload) == DeserializationError::Ok) {
                    float co2 = doc["co2"] | NAN;
                    float temp = doc["temp"] | NAN;
                    sensors.setRoomData(roomIdx, co2, temp);
                }
            } else {
                if (payload.length() == 0 || payload.equalsIgnoreCase("nan")) return;
                RoomData current = sensors.getRoomData(roomIdx);
                if (rest == "data/co2") {
                    sensors.setRoomData(roomIdx, payload.toFloat(), NAN);
                } else {
                    sensors.setRoomData(roomIdx, NAN, payload.toFloat());
                }
            }
            return;
        }
        if (rest == "set/manual_mode") { dampers.setManualMode(roomIdx, payload.toInt() != 0); return; }
        if (rest == "set/manual_pos")  { dampers.setManualPos(roomIdx, payload.toFloat()); return; }
        if (rest == "set/co2_target")  { storage.setRoomCo2Target(roomIdx, payload.toFloat()); return; }
        if (rest == "set/min_pos")     { storage.setRoomMinPos(roomIdx, payload.toFloat()); return; }
        if (rest == "set/max_pos")     { storage.setRoomMaxPos(roomIdx, payload.toFloat()); return; }
        if (rest == "set/temp_target") { storage.setRoomTempTarget(roomIdx, payload.toFloat()); return; }
        if (rest == "set/temp_mode") {
            bool enable = payload.toInt() != 0;
            if (enable && (!sensors.outdoorTempValid() ||
                           sensors.outdoorTemp() < storage.getFreecoolOutdoorMin())) {
                eventLog.add("temp_mode room" + String(roomIdx + 1) + ": команда отклонена (низкая T улицы)");
                return;
            }
            storage.setRoomTempModeEnable(roomIdx, enable);
            return;
        }
        return;
    }

    if (sub.startsWith("service/")) {
        if (!serviceMode_) return;
        String rest = sub.substring(8);
        if (rest.startsWith("relay/")) {
            int n = rest.substring(6).toInt() - 1;
            if (n < 0 || n >= HEATER_STAGES) return;
            bool on = payload.toInt() != 0;
            digitalWrite(PIN_RELAY[n], on ? HIGH : LOW);
            serviceRelayCmdMs_[n] = on ? millis() : 0;
            return;
        }
        if (rest.startsWith("servo/")) {
            int n = rest.substring(6).toInt() - 1;
            if (n < 0 || n >= ROOM_COUNT) return;
            float angle = payload.toFloat();
            Serial.print("[SERVICE] servo idx="); Serial.print(n);
            Serial.print(" angle="); Serial.println(angle);
            dampers.serviceSetAngle(n, angle);
            serviceServoCmdMs_[n] = millis();
            return;
        }
        return;
    }

    if (sub == "config/restore") {
        if (storage.importJson(payload)) eventLog.add("Config: импорт настроек выполнен");
        return;
    }
    if (sub == "config/backup_request") {
        publishRetained(root_ + "/config/backup", storage.exportJson());
        return;
    }

    if (sub == "set/power") { setPower(payload.toInt() != 0); return; }
    if (sub == "set/winter_mode") { storage.setWinterMode(payload.toInt() != 0); return; }
    if (sub == "set/service_mode") { setServiceMode(payload.toInt() != 0); return; }

    // Прямое управление вентилятором в сервисном режиме
    if (sub == "set/fan_pwm" && serviceMode_) {
        fan.setServicePwm(payload.toFloat());
        return;
    }

    if (sub == "set/co2_deadband") { storage.setCo2Deadband(payload.toFloat()); return; }
    if (sub == "set/co2_alarm_threshold") { storage.setCo2AlarmThreshold(payload.toFloat()); return; }
    if (sub == "set/min_servo_step") { storage.setMinServoStep(payload.toFloat()); return; }
    if (sub == "set/room_data_timeout") { storage.setRoomDataTimeout((uint32_t)payload.toInt()); return; }
    if (sub == "set/temp_freecool_deadband") { storage.setTempFreecoolDeadband(payload.toFloat()); return; }
    if (sub == "set/freecool_outdoor_min") { storage.setFreecoolOutdoorMin(payload.toFloat()); return; }

    if (sub == "set/dp_min") { storage.setDpMin(payload.toFloat()); return; }
    if (sub == "set/dp_max") { storage.setDpMax(payload.toFloat()); return; }
    if (sub == "set/dp_alarm_low") { storage.setDpAlarmLow(payload.toFloat()); return; }
    if (sub == "set/dp_alarm_high") { storage.setDpAlarmHigh(payload.toFloat()); return; }
    if (sub == "set/dp_low_timeout") { storage.setDpLowTimeout((uint32_t)payload.toInt()); return; }
    if (sub == "set/dp_high_timeout") { storage.setDpHighTimeout((uint32_t)payload.toInt()); return; }
    if (sub == "set/min_fan_step") { storage.setMinFanStep(payload.toFloat()); return; }
    if (sub == "set/min_fan_speed") { storage.setMinFanSpeed(payload.toFloat()); return; }
    if (sub == "set/min_dp_step") { storage.setMinDpStep(payload.toFloat()); return; }
    if (sub == "set/dp_kp") { fan.setDpTunings(payload.toFloat(), fan.dpKi(), fan.dpKd()); return; }
    if (sub == "set/dp_ki") { fan.setDpTunings(fan.dpKp(), payload.toFloat(), fan.dpKd()); return; }
    if (sub == "set/dp_kd") { fan.setDpTunings(fan.dpKp(), fan.dpKi(), payload.toFloat()); return; }
    if (sub == "set/flow_alarm_threshold") { storage.setFlowAlarmThreshold(payload.toFloat()); return; }
    if (sub == "set/dp_within_setpoint_pct") { storage.setDpWithinSetpointPct(payload.toFloat()); return; }
    if (sub == "set/filter_alarm_threshold") { sensors.setFilterAlarmThreshold(payload.toFloat()); return; }

    if (sub == "set/heater_target_temp") { storage.setHeaterTargetTemp(payload.toFloat()); return; }
    if (sub == "set/heater_deadband_low") { storage.setHeaterDeadbandLow(payload.toFloat()); return; }
    if (sub == "set/heater_deadband_high") { storage.setHeaterDeadbandHigh(payload.toFloat()); return; }
    if (sub == "set/heater_alarm_temp") { storage.setHeaterAlarmTemp(payload.toFloat()); return; }
    if (sub == "set/heater_outdoor_block_temp") { storage.setHeaterOutdoorBlockTemp(payload.toFloat()); return; }
    if (sub == "set/heater_min_fan_runtime") { storage.setHeaterMinFanRuntime((uint32_t)payload.toInt()); return; }
    if (sub == "set/heater_fault_reset") { if (payload.toInt() != 0) heater.resetFault(); return; }
    if (sub == "set/fan_fault_reset") { if (payload.toInt() != 0) fan.resetFault(); return; }
}

void MqttManager::publishState() {
    {
        DynamicJsonDocument doc(384);
        doc["power"] = powerOn_;
        doc["winter_mode"] = storage.getWinterMode();
        doc["service_mode"] = serviceMode_;
        doc["fw_version"] = FW_VERSION;
        doc["build_date"] = FW_BUILD_DATE;
        doc["uptime_s"] = millis() / 1000;
        doc["outdoor_temp"] = sensors.outdoorTempValid() ? sensors.outdoorTemp() : NAN;
        String out; serializeJson(doc, out);
        publishRetained(root_ + "/state", out);
    }

    for (uint8_t i = 0; i < ROOM_COUNT; i++) {
        RoomData rd = sensors.getRoomData(i);
        const bool co2Fresh = sensors.isRoomCo2Fresh(i);
        const bool tempFresh = sensors.isRoomTempFresh(i);
        RoomSettings rs = storage.getRoom(i);

        DynamicJsonDocument doc(384);
        doc["co2"] = rd.co2;
        doc["temp"] = rd.temp;
        doc["pos"] = dampers.getPos(i);
        doc["max_pos"] = dampers.getMaxPos(i);
        doc["manual_mode"] = dampers.isManual(i);
        doc["co2_fresh"] = co2Fresh;
        doc["temp_fresh"] = tempFresh;
        doc["data_fresh"] = co2Fresh && tempFresh;
        doc["block_reason"] = (int)dampers.getBlockReason(i);
        doc["temp_target"] = rs.temp_target;
        doc["co2_target"] = rs.co2_target;
        doc["temp_mode_enable"] = rs.temp_mode_enable;

        String out; serializeJson(doc, out);
        publishRetained(root_ + "/room/" + String(i + 1) + "/state", out);
    }

    {
        DynamicJsonDocument doc(384);
        doc["phase"] = (int)fan.phase();
        doc["pwm"] = fan.pwmPct();
        doc["dp"] = fan.dpMeasured();
        doc["dp_setpoint"] = fan.dpSetpoint();
        doc["dp_min"] = storage.getDpMin();
        doc["dp_max"] = storage.getDpMax();
        doc["flow_pct"] = sensors.flowPct();
        doc["flow_sensor_ok"] = sensors.flowSensorOk();
        doc["fault_latched"] = fan.faultLatched();
        doc["fault_code"] = fan.faultCode();
        doc["service_pwm"] = fan.servicePwm();
        doc["in_service_mode"] = fan.inServiceMode();
        doc["fault_dp_at_trip"] = fan.faultPressureAtTrip();
        String out; serializeJson(doc, out);
        publishRetained(root_ + "/fan/state", out);
    }

    {
        DynamicJsonDocument doc(320);
        doc["stage"] = heater.currentStage();
        doc["t_supply"] = sensors.supplyTempValid() ? sensors.supplyTemp() : NAN;
        doc["block_reason"] = (int)heater.blockReason();
        doc["fault_latched"] = heater.faultLatched();
        doc["fault_code"] = heater.faultCode();
        doc["fault_temp_at_trip"] = heater.faultTempAtTrip();
        String out; serializeJson(doc, out);
        publishRetained(root_ + "/heater/state", out);
    }

    publishRetained(root_ + "/log", eventLog.toJson(EVENT_LOG_PUBLISH_COUNT));

    // Filter sensor state
    {
        DynamicJsonDocument doc(256);
        doc["pressure"] = sensors.filterSensorOk() ? sensors.filterPressurePa() : NAN;
        doc["sensor_ok"] = sensors.filterSensorOk();
        doc["alarm_active"] = sensors.filterAlarmActive();
        doc["alarm_threshold"] = sensors.getFilterAlarmThreshold();
        String out; serializeJson(doc, out);
        publishRetained(root_ + "/filter/state", out);
    }
}

void MqttManager::publishHaDiscovery() {
    auto pubSwitch = [&](const String &objId, const String &name, const String &cmdTopic, const String &stateTopic, const String &valueTemplate) {
        DynamicJsonDocument doc(512);
        doc["name"] = name;
        doc["unique_id"] = "vent_" + objId;
        doc["command_topic"] = cmdTopic;
        doc["state_topic"] = stateTopic;
        doc["value_template"] = valueTemplate;
        doc["payload_on"] = "1";
        doc["payload_off"] = "0";
        String out; serializeJson(doc, out);
        publishRetained("homeassistant/switch/vent_" + objId + "/config", out);
    };
    auto pubBinarySensor = [&](const String &objId, const String &name, const String &stateTopic, const String &valueTemplate, const String &deviceClass = "") {
        DynamicJsonDocument doc(512);
        doc["name"] = name;
        doc["unique_id"] = "vent_" + objId;
        doc["state_topic"] = stateTopic;
        doc["value_template"] = valueTemplate;
        if (deviceClass.length()) doc["device_class"] = deviceClass;
        String out; serializeJson(doc, out);
        publishRetained("homeassistant/binary_sensor/vent_" + objId + "/config", out);
    };
    auto pubNumber = [&](const String &objId, const String &name, const String &cmdTopic, const String &stateTopic, const String &valueTemplate, const String &unit, float min, float max, float step) {
        DynamicJsonDocument doc(512);
        doc["name"] = name;
        doc["unique_id"] = "vent_" + objId;
        doc["command_topic"] = cmdTopic;
        doc["state_topic"] = stateTopic;
        doc["value_template"] = valueTemplate;
        if (unit.length()) doc["unit_of_measurement"] = unit;
        doc["min"] = min;
        doc["max"] = max;
        doc["step"] = step;
        doc["mode"] = "box";
        String out; serializeJson(doc, out);
        publishRetained("homeassistant/number/vent_" + objId + "/config", out);
    };
    auto pubSensor = [&](const String &objId, const String &name, const String &stateTopic, const String &valueTemplate, const String &unit) {
        DynamicJsonDocument doc(512);
        doc["name"] = name;
        doc["unique_id"] = "vent_" + objId;
        doc["state_topic"] = stateTopic;
        doc["value_template"] = valueTemplate;
        if (unit.length()) doc["unit_of_measurement"] = unit;
        String out; serializeJson(doc, out);
        publishRetained("homeassistant/sensor/vent_" + objId + "/config", out);
    };
    auto pubButton = [&](const String &objId, const String &name, const String &cmdTopic, const String &deviceClass = "") {
        DynamicJsonDocument doc(512);
        doc["name"] = name;
        doc["unique_id"] = "vent_" + objId;
        doc["command_topic"] = cmdTopic;
        doc["payload_press"] = "1";
        if (deviceClass.length()) doc["device_class"] = deviceClass;
        String out; serializeJson(doc, out);
        publishRetained("homeassistant/button/vent_" + objId + "/config", out);
    };

    String root = root_;
    pubSwitch("power", "Vent Power", root + "/set/power", root + "/state", "{{ value_json.power }}");
    pubSwitch("winter_mode", "Vent Winter Mode", root + "/set/winter_mode", root + "/state", "{{ value_json.winter_mode }}");
    
    // Binary sensors for faults
    pubBinarySensor("fan_fault", "Fan Fault", root + "/fan/state", "{{ value_json.fault_latched }}", "problem");
    pubBinarySensor("heater_fault", "Heater Fault", root + "/heater/state", "{{ value_json.fault_latched }}", "problem");
    
    pubSensor("outdoor_temp", "Outdoor Temp", root + "/state", "{{ value_json.outdoor_temp }}", "°C");
    pubSensor("fan_pwm", "Fan PWM", root + "/fan/state", "{{ value_json.pwm }}", "%");
    pubSensor("fan_dp", "Fan Pressure", root + "/fan/state", "{{ value_json.dp }}", "Pa");
    pubSensor("fan_dp_setpoint", "Fan Pressure Setpoint", root + "/fan/state", "{{ value_json.dp_setpoint }}", "Pa");
    pubSensor("flow_pct", "Air Flow", root + "/fan/state", "{{ value_json.flow_pct }}", "%");
    pubSensor("filter_pressure", "Filter Pressure", root + "/filter/state", "{{ value_json.pressure }}", "Pa");
    pubSensor("filter_alarm", "Filter Alarm", root + "/filter/state", "{{ value_json.alarm_active }}", "");
    pubSensor("heater_stage", "Heater Stage", root + "/heater/state", "{{ value_json.stage }}", "");
    pubSensor("heater_t_supply", "Supply Temp", root + "/heater/state", "{{ value_json.t_supply }}", "°C");
    
    // Text sensors for phase and block reasons
    pubSensor("fan_phase", "Fan Phase", root + "/fan/state", "{% set p=value_json.phase|int %}{% if p==0 %}Off{% elif p==1 %}Starting{% elif p==2 %}Ramping{% elif p==3 %}Running{% elif p==4 %}Stopping{% elif p==5 %}Fault{% else %}Unknown{% endif %}", "");
    pubSensor("heater_block_reason", "Heater Block Reason", root + "/heater/state", "{% set r=value_json.block_reason|int %}{% if r==0 %}None{% elif r==1 %}System Off{% elif r==2 %}Winter Mode Off{% elif r==3 %}Outdoor Too Warm{% elif r==4 %}Freecool Active{% elif r==5 %}No Duct Data{% elif r==6 %}Fan Not Ready{% elif r==7 %}Low Flow{% elif r==8 %}Flow Sensor Fault{% elif r==9 %}Pressure Out of Range{% else %}Unknown{% endif %}", "");
    
    // Number for heater target temp
    pubNumber("heater_target_temp", "Heater Target Temp", root + "/set/heater_target_temp", root + "/heater/state", "{{ value_json.t_supply }}", "°C", 10.0, 60.0, 1.0);
    
    // Buttons for fault reset
    pubButton("heater_fault_reset", "Heater Fault Reset", root + "/set/heater_fault_reset", "problem");
    pubButton("fan_fault_reset", "Fan Fault Reset", root + "/set/fan_fault_reset", "problem");

    for (uint8_t i = 0; i < ROOM_COUNT; i++) {
        String rt = root + "/room/" + String(i + 1) + "/state";
        String roomSetRoot = root + "/room/" + String(i + 1) + "/set";
        
        pubSensor("room" + String(i+1) + "_co2", "Room " + String(i+1) + " CO2", rt, "{{ value_json.co2 }}", "ppm");
        pubSensor("room" + String(i+1) + "_temp", "Room " + String(i+1) + " Temp", rt, "{{ value_json.temp }}", "°C");
        pubSensor("room" + String(i+1) + "_pos", "Room " + String(i+1) + " Damper", rt, "{{ value_json.pos }}", "%");
        pubSensor("room" + String(i+1) + "_co2_fresh", "Room " + String(i+1) + " CO2 Fresh", rt, "{{ value_json.co2_fresh }}", "");
        pubSensor("room" + String(i+1) + "_temp_fresh", "Room " + String(i+1) + " Temperature Fresh", rt, "{{ value_json.temp_fresh }}", "");
        
        // Switch for manual mode
        pubSwitch("room" + String(i+1) + "_manual_mode", "Room " + String(i+1) + " Manual Mode", roomSetRoot + "/manual_mode", rt, "{{ value_json.manual_mode }}");
        
        // Switch for temp mode enable
        pubSwitch("room" + String(i+1) + "_temp_mode", "Room " + String(i+1) + " Temp Mode", roomSetRoot + "/temp_mode", rt, "{{ value_json.temp_mode_enable }}");
        
        // Number for temp target
        pubNumber("room" + String(i+1) + "_temp_target", "Room " + String(i+1) + " Temp Target", roomSetRoot + "/temp_target", rt, "{{ value_json.temp_target }}", "°C", 10.0, 35.0, 0.5);
        
        // Number for co2 target
        pubNumber("room" + String(i+1) + "_co2_target", "Room " + String(i+1) + " CO2 Target", roomSetRoot + "/co2_target", rt, "{{ value_json.co2_target }}", "ppm", 400.0, 2000.0, 50.0);
    }
}
