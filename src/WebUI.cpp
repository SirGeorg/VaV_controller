#include <WiFi.h>
#include <WiFiManager.h>
#include "WebUI.h"
#include "MqttManager.h"
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>

WebUI webUi;

static const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html><html lang="ru"><head><meta charset="utf-8">
<title>Vent Controller</title><meta name="viewport" content="width=device-width, initial-scale=1">
<style>
body{font-family:sans-serif;background:#111;color:#eee;margin:0;padding:16px}
h1{font-size:18px}.card{background:#1c1c1c;border-radius:8px;padding:12px;margin-bottom:10px}
table{width:100%;font-size:14px}td{padding:2px 4px}
</style></head><body>
<h1>Vent Controller — vent.local</h1>
<div class="card" id="status">Загрузка...</div>
<div class="card"><b>Версия:</b> <span id="fw"></span><br>
<b>Аптайм:</b> <span id="uptime"></span> с</div>
<script>
function fnum(x,d){return(x===null||x===undefined)?'—':Number(x).toFixed(d);}
function fresh(x){return x?'OK':'нет';}
async function refresh(){
 try{
  const r=await fetch('/api/status'); if(!r.ok)throw new Error('HTTP '+r.status);
  const d=await r.json();
  document.getElementById('fw').textContent=d.fw_version+' ('+d.build_date+')';
  document.getElementById('uptime').textContent=d.uptime_s;
  let html='<table><tr><td>Power</td><td>'+d.power+'</td></tr>'+
   '<tr><td>Winter mode</td><td>'+d.winter_mode+'</td></tr>'+
   '<tr><td>Service mode</td><td>'+d.service_mode+'</td></tr>'+
   '<tr><td>Fan phase</td><td>'+d.fan.phase+'</td></tr>'+
   '<tr><td>Fan PWM</td><td>'+fnum(d.fan.pwm,1)+' %</td></tr>'+
   '<tr><td>dP / setpoint</td><td>'+fnum(d.fan.dp,0)+' / '+fnum(d.fan.dp_setpoint,0)+' Pa</td></tr>'+
   '<tr><td>Flow</td><td>'+fnum(d.fan.flow_pct,0)+' %</td></tr>'+
   '<tr><td>Heater stage</td><td>'+d.heater.stage+' / 5</td></tr>'+
   '<tr><td>T supply</td><td>'+fnum(d.heater.t_supply,1)+' °C</td></tr></table>';
  if(d.rooms){
   html+='<h3>Комнаты</h3><table>';
   d.rooms.forEach((x,i)=>{
    html+='<tr><td>Комната '+(i+1)+'</td><td>CO₂ '+fnum(x.co2,0)+' ('+fresh(x.co2_fresh)+
      '), T '+fnum(x.temp,1)+' °C ('+fresh(x.temp_fresh)+'), заслонка '+fnum(x.pos,0)+'%</td></tr>';
   });
   html+='</table>';
  }
  document.getElementById('status').innerHTML=html;
 }catch(err){document.getElementById('status').innerHTML='<div style="color:#f88">Ошибка: '+err.message+'</div>';}
}
setInterval(refresh,3000);refresh();
</script></body></html>
)HTML";

void WebUI::begin() {
    WiFiManager wm;
    wm.setConfigPortalTimeout(180);
    wm.autoConnect(WIFI_AP_NAME);
    if (MDNS.begin(MDNS_HOSTNAME)) MDNS.addService("http","tcp",80);
    ArduinoOTA.setHostname(OTA_HOSTNAME);
    ArduinoOTA.begin();
    setupRoutes();
    server_.begin();
}

void WebUI::loop() { ArduinoOTA.handle(); }

void WebUI::setupRoutes() {
    server_.on("/", HTTP_GET, [](AsyncWebServerRequest *req){ req->send_P(200,"text/html",INDEX_HTML); });

    server_.on("/api/status", HTTP_GET, [](AsyncWebServerRequest *req) {
        DynamicJsonDocument doc(2048);
        doc["fw_version"] = FW_VERSION;
        doc["build_date"] = FW_BUILD_DATE;
        doc["uptime_s"] = millis() / 1000;
        doc["power"] = mqttManager.powerOn();
        doc["winter_mode"] = storage.getWinterMode();
        doc["service_mode"] = mqttManager.serviceMode();

        JsonObject fanObj = doc.createNestedObject("fan");
        fanObj["phase"] = (int)fan.phase();
        fanObj["pwm"] = fan.pwmPct();
        fanObj["dp"] = fan.dpMeasured();
        fanObj["dp_setpoint"] = fan.dpSetpoint();
        fanObj["flow_pct"] = sensors.flowPct();

        JsonObject heaterObj = doc.createNestedObject("heater");
        heaterObj["stage"] = heater.currentStage();
        heaterObj["t_supply"] = sensors.supplyTempValid() ? sensors.supplyTemp() : NAN;

        JsonArray rooms = doc.createNestedArray("rooms");
        for (uint8_t i = 0; i < ROOM_COUNT; i++) {
            RoomData rd = sensors.getRoomData(i);
            JsonObject ro = rooms.createNestedObject();
            ro["co2"] = rd.co2;
            ro["temp"] = rd.temp;
            ro["co2_fresh"] = sensors.isRoomCo2Fresh(i);
            ro["temp_fresh"] = sensors.isRoomTempFresh(i);
            ro["data_fresh"] = sensors.isRoomDataFresh(i);
            ro["pos"] = dampers.getPos(i);
            ro["block_reason"] = (int)dampers.getBlockReason(i);
        }

        String out;
        serializeJson(doc, out);
        req->send(200, "application/json", out);
    });

    server_.on("/api/calibrate/servo", HTTP_POST, [](AsyncWebServerRequest *req) {
        if (!req->hasParam("room") || !req->hasParam("pmin") || !req->hasParam("pmax")) {
            req->send(400, "text/plain", "missing params"); return;
        }
        int room = req->getParam("room")->value().toInt() - 1;
        uint16_t pmin = req->getParam("pmin")->value().toInt();
        uint16_t pmax = req->getParam("pmax")->value().toInt();
        if (room < 0 || room >= ROOM_COUNT) { req->send(400, "text/plain", "bad room"); return; }
        dampers.calibrateServo(room, pmin, pmax);
        req->send(200, "text/plain", "ok");
    });

    server_.on("/api/calibrate/analog", HTTP_POST, [](AsyncWebServerRequest *req) {
        if (!req->hasParam("sensor") || !req->hasParam("offset") || !req->hasParam("scale")) {
            req->send(400, "text/plain", "missing params"); return;
        }
        String sensor = req->getParam("sensor")->value();
        float offset = req->getParam("offset")->value().toFloat();
        float scale = req->getParam("scale")->value().toFloat();
        if (sensor == "dp") sensors.setPressureCal(offset, scale);
        else if (sensor == "flow") sensors.setFlowCal(offset, scale);
        else { req->send(400, "text/plain", "bad sensor"); return; }
        req->send(200, "text/plain", "ok");
    });

    server_.on("/api/backup", HTTP_GET, [](AsyncWebServerRequest *req){ req->send(200,"application/json",storage.exportJson()); });

    server_.on("/api/restore", HTTP_POST,
        [](AsyncWebServerRequest *req){},
        nullptr,
        [](AsyncWebServerRequest *req,uint8_t *data,size_t len,size_t index,size_t total){
            String body((char*)data,len);
            bool ok=storage.importJson(body);
            req->send(ok?200:400,"text/plain",ok?"ok":"invalid json");
        });

    server_.on("/api/log", HTTP_GET, [](AsyncWebServerRequest *req){
        req->send(200,"application/json",eventLog.toJson(EVENT_LOG_SIZE));
    });
}
