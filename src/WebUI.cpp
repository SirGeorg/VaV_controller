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
<title>Vent Controller v4.1</title><meta name="viewport" content="width=device-width, initial-scale=1">
<style>
body{font-family:sans-serif;background:#111;color:#eee;margin:0;padding:16px}
h1{font-size:18px}.card{background:#1c1c1c;border-radius:8px;padding:12px;margin-bottom:10px}
table{width:100%;font-size:14px}td{padding:2px 4px}
input,select{background:#333;color:#eee;border:1px solid #555;padding:4px;border-radius:4px}
button{background:#0a84ff;color:#fff;border:none;padding:6px 12px;border-radius:4px;cursor:pointer}
button:hover{background:#0077ed}
.chart-container{height:200px;position:relative;margin-top:10px}
canvas{width:100%!important;height:100%!important}
.tab-btn{background:#333;margin-right:4px}
.tab-btn.active{background:#0a84ff}
.tab-content{display:none}
.tab-content.active{display:block}
</style></head><body>
<h1>Vent Controller v4.1 — vent.local</h1>
<div style="margin-bottom:10px">
 <button class="tab-btn active" onclick="showTab('status')">Статус</button>
 <button class="tab-btn" onclick="showTab('mqtt')">MQTT</button>
 <button class="tab-btn" onclick="showTab('calib')">Калибровка</button>
 <button class="tab-btn" onclick="showTab('config')">Конфигурация</button>
</div>

<div id="tab-status" class="tab-content active">
 <div class="card" id="status">Загрузка...</div>
 <div class="card"><b>Версия:</b> <span id="fw"></span><br><b>Аптайм:</b> <span id="uptime"></span> с</div>
 <div class="card">
  <h3>График перепада давления (10 мин)</h3>
  <div class="chart-container"><canvas id="dpChart"></canvas></div>
 </div>
</div>

<div id="tab-mqtt" class="tab-content">
 <div class="card">
  <h3>Настройки MQTT</h3>
  <form id="mqttForm">
   <table style="max-width:400px">
    <tr><td>Сервер:</td><td><input type="text" id="mqtt_server" required></td></tr>
    <tr><td>Порт:</td><td><input type="number" id="mqtt_port" value="1883" required></td></tr>
    <tr><td>Пользователь:</td><td><input type="text" id="mqtt_user"></td></tr>
    <tr><td>Пароль:</td><td><input type="password" id="mqtt_pass"></td></tr>
    <tr><td>Client ID:</td><td><input type="text" id="mqtt_client_id" readonly></td></tr>
    <tr><td>Root topic:</td><td><input type="text" id="mqtt_root" value="vent" readonly></td></tr>
   </table>
   <p><button type="submit">Сохранить и перезагрузить</button></p>
  </form>
 </div>
</div>

<div id="tab-calib" class="tab-content">
 <div class="card">
  <h3>Калибровка сервоприводов</h3>
  <form id="servoForm">
   <table>
    <tr><td>Комната:</td><td><select id="cal_room"><option>1</option><option>2</option><option>3</option><option>4</option></select></td></tr>
    <tr><td>Pmin (мкс):</td><td><input type="number" id="cal_pmin" value="1000"></td></tr>
    <tr><td>Pmax (мкс):</td><td><input type="number" id="cal_pmax" value="2000"></td></tr>
   </table>
   <p><button type="submit">Калибровать</button></p>
  </form>
 </div>
 <div class="card">
  <h3>Калибровка датчиков 4-20мА</h3>
  <form id="analogForm">
   <table>
    <tr><td>Датчик:</td><td><select id="analog_sensor"><option value="dp">Перепад давления</option><option value="flow">Расход воздуха</option></select></td></tr>
    <tr><td>Offset:</td><td><input type="number" step="0.01" id="analog_offset" value="0"></td></tr>
    <tr><td>Scale:</td><td><input type="number" step="0.01" id="analog_scale" value="1"></td></tr>
   </table>
   <p><button type="submit">Калибровать</button></p>
  </form>
 </div>
</div>

<div id="tab-config" class="tab-content">
 <div class="card">
  <h3>Резервное копирование</h3>
  <p><button onclick="backup()">Скачать конфигурацию</button></p>
  <p><label>Восстановить из JSON: <input type="file" id="restoreFile" accept=".json"></label></p>
  <p><button onclick="restore()">Восстановить</button></p>
 </div>
 <div class="card">
  <h3>Журнал событий</h3>
  <p><button onclick="loadLog()">Загрузить журнал</button></p>
  <pre id="logOutput" style="background:#222;padding:8px;overflow:auto;max-height:300px;font-size:12px"></pre>
 </div>
</div>

<script>
const DP_HISTORY_MAX=200;
let dpHistory={labels:[],dp:[],dpSetpoint:[]};
let chart=null;

function showTab(name){
 document.querySelectorAll('.tab-content').forEach(el=>el.classList.remove('active'));
 document.querySelectorAll('.tab-btn').forEach(el=>el.classList.remove('active'));
 document.getElementById('tab-'+name).classList.add('active');
 event.target.classList.add('active');
 if(name==='mqtt') loadMqttConfig();
}

function fnum(x,d){return(x===null||x===undefined)?'—':Number(x).toFixed(d);}
function fresh(x){return x?'OK':'нет';}

function initChart(){
 const ctx=document.getElementById('dpChart').getContext('2d');
 chart=new Chart(ctx,{
  type:'line',
  data:{labels:dpHistory.labels,datasets:[
   {label:'dP (Pa)',data:dpHistory.dp,borderColor:'#0a84ff',tension:0.3,pointRadius:0},
   {label:'Уставка (Pa)',data:dpHistory.dpSetpoint,borderColor:'#30d158',tension:0.3,pointRadius:0}
  ]},
  options:{responsive:true,maintainAspectRatio:false,animation:false,
   scales:{x:{display:false},y:{beginAtZero:false}},
   plugins:{legend:{labels:{color:'#eee'}}}
  }
 });
}

async function updateChart(dp,dpSp){
 const now=new Date();
 const timeLabel=now.getHours().toString().padStart(2,'0')+':'+now.getMinutes().toString().padStart(2,'0')+':'+now.getSeconds().toString().padStart(2,'0');
 dpHistory.labels.push(timeLabel);
 dpHistory.dp.push(dp);
 dpHistory.dpSetpoint.push(dpSp);
 if(dpHistory.labels.length>DP_HISTORY_MAX){
  dpHistory.labels.shift();
  dpHistory.dp.shift();
  dpHistory.dpSetpoint.shift();
 }
 if(chart){
  chart.data.labels=dpHistory.labels;
  chart.data.datasets[0].data=dpHistory.dp;
  chart.data.datasets[1].data=dpHistory.dpSetpoint;
  chart.update();
 }
}

async function loadMqttConfig(){
 try{
  const r=await fetch('/api/mqtt/config');
  if(r.ok){
   const d=await r.json();
   document.getElementById('mqtt_server').value=d.server||'';
   document.getElementById('mqtt_port').value=d.port||1883;
   document.getElementById('mqtt_user').value=d.user||'';
   document.getElementById('mqtt_pass').value='';
   document.getElementById('mqtt_client_id').value=d.client_id||'vent_esp32';
   document.getElementById('mqtt_root').value=d.root||'vent';
  }
 }catch(e){console.error(e);}
}

document.getElementById('mqttForm').addEventListener('submit',async(e)=>{
 e.preventDefault();
 const data={
  server:document.getElementById('mqtt_server').value,
  port:parseInt(document.getElementById('mqtt_port').value),
  user:document.getElementById('mqtt_user').value,
  pass:document.getElementById('mqtt_pass').value
 };
 try{
  const r=await fetch('/api/mqtt/config',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data)});
  if(r.ok){alert('Настройки сохранены. Устройство будет перезапущено.');}
  else{alert('Ошибка: '+await r.text());}
 }catch(e){alert('Ошибка: '+e.message);}
});

document.getElementById('servoForm').addEventListener('submit',async(e)=>{
 e.preventDefault();
 const room=document.getElementById('cal_room').value;
 const pmin=document.getElementById('cal_pmin').value;
 const pmax=document.getElementById('cal_pmax').value;
 try{
  const r=await fetch('/api/calibrate/servo?room='+room+'&pmin='+pmin+'&pmax='+pmax,{method:'POST'});
  if(r.ok)alert('Калибровка выполнена');
  else alert('Ошибка: '+await r.text());
 }catch(e){alert('Ошибка: '+e.message);}
});

document.getElementById('analogForm').addEventListener('submit',async(e)=>{
 e.preventDefault();
 const sensor=document.getElementById('analog_sensor').value;
 const offset=document.getElementById('analog_offset').value;
 const scale=document.getElementById('analog_scale').value;
 try{
  const r=await fetch('/api/calibrate/analog?sensor='+sensor+'&offset='+offset+'&scale='+scale,{method:'POST'});
  if(r.ok)alert('Калибровка выполнена');
  else alert('Ошибка: '+await r.text());
 }catch(e){alert('Ошибка: '+e.message);}
});

async function backup(){
 try{
  const r=await fetch('/api/backup');
  if(r.ok){
   const blob=await r.blob();
   const a=document.createElement('a');
   a.href=URL.createObjectURL(blob);
   a.download='vent_config_'+new Date().toISOString().slice(0,19).replace(/:/g,'-')+'.json';
   a.click();
  }
 }catch(e){alert('Ошибка: '+e.message);}
}

async function restore(){
 const file=document.getElementById('restoreFile').files[0];
 if(!file){alert('Выберите файл');return;}
 try{
  const formData=new FormData();
  formData.append('file',file);
  const r=await fetch('/api/restore',{method:'POST',body:formData});
  if(r.ok)alert('Конфигурация восстановлена');
  else alert('Ошибка: '+await r.text());
 }catch(e){alert('Ошибка: '+e.message);}
}

async function loadLog(){
 try{
  const r=await fetch('/api/log');
  if(r.ok){
   const d=await r.json();
   document.getElementById('logOutput').textContent=JSON.stringify(d,null,2);
  }
 }catch(e){alert('Ошибка: '+e.message);}
}

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
      '), T '+fnum(x.temp,1)+' °C ('+fresh(x.temp_fresh)+'), заслонка '+fnum(x.pos,0)+'%, ручн. '+(x.manual_mode?' да':' нет')+'</td></tr>';
   });
   html+='</table>';
  }
  document.getElementById('status').innerHTML=html;
  updateChart(d.fan.dp,d.fan.dp_setpoint);
 }catch(err){document.getElementById('status').innerHTML='<div style="color:#f88">Ошибка: '+err.message+'</div>';}
}

window.addEventListener('load',()=>{initChart();setInterval(refresh,3000);refresh();});
</script>
<script src="https://cdn.jsdelivr.net/npm/chart.js@4.4.0/dist/chart.umd.min.js"></script>
</body></html>
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
            ro["manual_mode"] = dampers.isManual(i);
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

    // MQTT конфигурация - чтение/запись через веб-интерфейс
    server_.on("/api/mqtt/config", HTTP_GET, [](AsyncWebServerRequest *req) {
        MqttConfig cfg = storage.getMqttConfig();
        DynamicJsonDocument doc(512);
        doc["server"] = cfg.server[0] != '\0' ? cfg.server : MQTT_SERVER;
        doc["port"] = cfg.port;
        doc["user"] = cfg.user;
        doc["client_id"] = MQTT_CLIENT_ID;
        doc["root"] = MQTT_ROOT;
        String out;
        serializeJson(doc, out);
        req->send(200, "application/json", out);
    });

    server_.on("/api/mqtt/config", HTTP_POST, [](AsyncWebServerRequest *req) {
        if (!req->hasParam("server") || !req->hasParam("port")) {
            req->send(400, "text/plain", "missing params"); return;
        }
        String server = req->getParam("server")->value();
        uint16_t port = req->getParam("port")->value().toInt();
        String user = req->hasParam("user") ? req->getParam("user")->value() : "";
        String pass = req->hasParam("pass") ? req->getParam("pass")->value() : "";

        // Сохраняем в NVS
        storage.setMqttServer(server.c_str());
        storage.setMqttPort(port);
        storage.setMqttUser(user.c_str());
        storage.setMqttPass(pass.c_str());

        // Перезагружаем ESP32 для применения настроек
        req->send(200, "text/plain", "ok");
        delay(500);
        ESP.restart();
    });
}
