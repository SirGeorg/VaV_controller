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
<h1>Vent Controller v4.3 — vent.local</h1>
<div style="margin-bottom:10px">
 <button class="tab-btn active" onclick="showTab('status')">Статус</button>
 <button class="tab-btn" onclick="showTab('mqtt')">MQTT</button>
 <button class="tab-btn" onclick="showTab('calib')">Калибровка</button>
 <button class="tab-btn" onclick="showTab('config')">Конфигурация</button>
 <button class="tab-btn" onclick="showTab('control')">Управление</button>
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
    <tr><td>Пароль:</td><td><input type="text" id="mqtt_pass"></td></tr>
    <tr><td>Client ID:</td><td><input type="text" id="mqtt_client_id"></td></tr>
    <tr><td>Root topic:</td><td><input type="text" id="mqtt_root" value="vent"></td></tr>
   </table>
   <p><button type="submit">Сохранить и перезагрузить</button></p>
  </form>
 </div>
</div>

<div id="tab-control" class="tab-content">
 <div class="card" id="control">Загрузка...</div>
</div>

<div id="tab-calib" class="tab-content">
<div class="card">
  <h3>Текущие значения калибровки</h3>
  <table style="max-width:420px">
   <tr><td>Сервоприводы (мкс):</td><td id="cal_servo_info">—</td></tr>
   <tr><td>Датчик dP (offset / scale):</td><td id="cal_dp_info">—</td></tr>
   <tr><td>Датчик расхода (offset / scale):</td><td id="cal_flow_info">—</td></tr>
   <tr><td>Датчик фильтра (offset / scale):</td><td id="cal_filter_info">—</td></tr>
  </table>
  <p id="cal_msg" style="color:#aaa"></p>
 </div>
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
    <tr><td>Датчик:</td><td><select id="analog_sensor"><option value="dp">Перепад давления</option><option value="flow">Расход воздуха</option><option value="filter">Фильтр</option></select></td></tr>
    <tr><td>Offset:</td><td><input type="number" step="0.01" id="analog_offset" value="0"></td></tr>
    <tr><td>Scale:</td><td><input type="number" step="0.01" id="analog_scale" value="1"></td></tr>
   </table>
   <p><button type="submit">Калибровать</button></p>
  </form>
 </div>
 <div class="card">
  <h3>Коэффициенты PID вентилятора</h3>
  <table>
   <tr><td>Kp:</td><td><input type="number" step="0.001" id="pid_kp"></td></tr>
   <tr><td>Ki:</td><td><input type="number" step="0.0001" id="pid_ki"></td></tr>
   <tr><td>Kd:</td><td><input type="number" step="0.0001" id="pid_kd"></td></tr>
  </table>
  <p><button onclick="sendPids()">Применить</button></p>
  <p style="color:#aaa">Текущие коэффициенты берутся из NVS (см. вкладку «Управление»).</p>
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
let dpHistory={labels:[],dp:[],dpSetpoint:[],filter:[],flow:[],fanPwm:[]};
let chart=null;

function showTab(name){
 document.querySelectorAll('.tab-content').forEach(el=>el.classList.remove('active'));
 document.querySelectorAll('.tab-btn').forEach(el=>el.classList.remove('active'));
 document.getElementById('tab-'+name).classList.add('active');
 event.target.classList.add('active');
 if(name==='mqtt') loadMqttConfig();
 if(name==='calib') loadCalib();
}

function fnum(x,d){return(x===null||x===undefined)?'—':Number(x).toFixed(d);}
function fresh(x){return x?'OK':'нет';}

function initChart(){
 const ctx=document.getElementById('dpChart').getContext('2d');
 chart=new Chart(ctx,{
  type:'line',
  data:{labels:dpHistory.labels,datasets:[
   {label:'dP (Pa)',data:dpHistory.dp,borderColor:'#0a84ff',tension:0.3,pointRadius:0,yAxisID:'y'},
   {label:'Уставка (Pa)',data:dpHistory.dpSetpoint,borderColor:'#30d158',tension:0.3,pointRadius:0,yAxisID:'y'},
   {label:'Фильтр (Pa)',data:dpHistory.filter,borderColor:'#ff9f0a',tension:0.3,pointRadius:0,yAxisID:'y'},
   {label:'Расход (%)',data:dpHistory.flow,borderColor:'#bf5af2',tension:0.3,pointRadius:0,yAxisID:'y1'},
   {label:'Вентилятор (%)',data:dpHistory.fanPwm,borderColor:'#ff3b30',tension:0.3,pointRadius:0,yAxisID:'y1'}
  ]},
  options:{responsive:true,maintainAspectRatio:false,animation:false,
   scales:{
    x:{display:false},
    y:{
      type:'linear',
      display:true,
      position:'left',
      beginAtZero:false,
      ticks:{color:'#eee'},
      grid:{color:'rgba(255,255,255,0.06)'}
    },
    y1:{
      type:'linear',
      display:true,
      position:'right',
      beginAtZero:false,
      ticks:{color:'#eee'},
      grid:{drawOnChartArea:false}
    }
   },
   plugins:{legend:{labels:{color:'#eee'}}}
  }
 });
}

async function updateChart(dp,dpSp,filterPressure,flowPct,fanPwm){
 const now=new Date();
 const timeLabel=now.getHours().toString().padStart(2,'0')+':'+now.getMinutes().toString().padStart(2,'0')+':'+now.getSeconds().toString().padStart(2,'0');
 dpHistory.labels.push(timeLabel);
 dpHistory.dp.push(dp);
 dpHistory.dpSetpoint.push(dpSp);
 dpHistory.filter.push(filterPressure);
 dpHistory.flow.push(flowPct);
 dpHistory.fanPwm.push(fanPwm);
 if(dpHistory.labels.length>DP_HISTORY_MAX){
  dpHistory.labels.shift();
  dpHistory.dp.shift();
  dpHistory.dpSetpoint.shift();
  dpHistory.filter.shift();
  dpHistory.flow.shift();
  dpHistory.fanPwm.shift();
 }
 if(chart){
  chart.data.labels=dpHistory.labels;
  chart.data.datasets[0].data=dpHistory.dp;
  chart.data.datasets[1].data=dpHistory.dpSetpoint;
  chart.data.datasets[2].data=dpHistory.filter;
  chart.data.datasets[3].data=dpHistory.flow;
  chart.data.datasets[4].data=dpHistory.fanPwm;
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

async function loadCalib(){
 try{
  const r=await fetch('/api/calib');
  if(!r.ok)throw new Error('HTTP '+r.status);
  const d=await r.json();
  const s=(d.servo||[]).map(x=>'комн.'+x.room+': '+x.pmin+'/'+x.pmax+' мкс').join('<br>');
  document.getElementById('cal_servo_info').innerHTML=s||'—';
  document.getElementById('cal_dp_info').textContent='offset='+(d.dp&&d.dp.offset!==undefined?d.dp.offset:'—')+', scale='+(d.dp&&d.dp.scale!==undefined?d.dp.scale:'—');
  document.getElementById('cal_flow_info').textContent='offset='+(d.flow&&d.flow.offset!==undefined?d.flow.offset:'—')+', scale='+(d.flow&&d.flow.scale!==undefined?d.flow.scale:'—');
  document.getElementById('cal_filter_info').textContent='offset='+(d.filter&&d.filter.offset!==undefined?d.filter.offset:'—')+', scale='+(d.filter&&d.filter.scale!==undefined?d.filter.scale:'—');
 }catch(e){const m=document.getElementById('cal_msg'); if(m)m.textContent='Ошибка: '+e.message;}
}

document.getElementById('mqttForm').addEventListener('submit',async(e)=>{
 e.preventDefault();
 const data={
  server:document.getElementById('mqtt_server').value,
  port:parseInt(document.getElementById('mqtt_port').value),
  user:document.getElementById('mqtt_user').value,
  pass:document.getElementById('mqtt_pass').value,
  client_id:document.getElementById('mqtt_client_id').value,
  root:document.getElementById('mqtt_root').value
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
  if(r.ok){alert('Калибровка выполнена');loadCalib();}
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
  if(r.ok){alert('Калибровка выполнена');loadCalib();}
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
   const lines=(d||[]).map(x=>'['+(x.time||'—')+'] '+x.msg);
   document.getElementById('logOutput').textContent=lines.join('\n')||'Журнал пуст';
  }
 }catch(e){alert('Ошибка: '+e.message);}
}

async function refresh(){
 try{
  const r=await fetch('/api/status'); if(!r.ok)throw new Error('HTTP '+r.status);
  const d=await r.json();
  document.getElementById('fw').textContent=d.fw_version+' ('+d.build_date+')';
  document.getElementById('uptime').textContent=d.uptime_s;
  let html='<table><tr><td><b>Вентилятор</b></td><td>Фаза: '+d.fan.phase+'</td><td>ШИМ: '+fnum(d.fan.pwm,1)+' %</td></tr>'+
   '<tr><td></td><td>Перепад: '+fnum(d.fan.dp,0)+' / '+fnum(d.fan.dp_setpoint,0)+' Па</td><td>Расход: '+fnum(d.fan.flow_pct,0)+' %</td></tr>'+
   '<tr><td><b>Фильтр</b></td><td>Перепад: '+(d.filter&&d.filter.pressure?fnum(d.filter.pressure,0):'—')+' Па</td><td>Порог: '+(d.filter?fnum(d.filter.alarm_threshold,0):'—')+' Па</td></tr>'+
   '<tr><td><b>Нагреватель</b></td><td>Ступень: '+d.heater.stage+' / 5</td><td>Т приток: '+fnum(d.heater.t_supply,1)+' °C</td></tr>'+
   '<tr><td><b>Режимы</b></td><td>Зима: '+(d.winter_mode?'ВКЛ':'ВЫКЛ')+'</td><td>Сервис: '+(d.service_mode?'ВКЛ':'ВЫКЛ')+'</td></tr></table>';
  if(d.rooms){
   html+='<h3>Комнаты</h3><table>';
   d.rooms.forEach((x,i)=>{
    html+='<tr><td>Комната '+(i+1)+'</td><td>CO₂ '+fnum(x.co2,0)+' ('+fresh(x.co2_fresh)+
      '), T '+fnum(x.temp,1)+' °C ('+fresh(x.temp_fresh)+'), заслонка '+fnum(x.pos,0)+'%, ручн. '+(x.manual_mode?' да':' нет')+'</td></tr>';
   });
   html+='</table>';
  }
  document.getElementById('status').innerHTML=html;
  renderControl(d);
  updateChart(d.fan.dp,d.fan.dp_setpoint, d.filter?d.filter.pressure:NaN, d.fan.flow_pct, d.fan.pwm);
 }catch(err){document.getElementById('status').innerHTML='<div style="color:#f88">Ошибка: '+err.message+'</div>';}
}

async function cmd(qs){
 try{
  const r=await fetch('/api/ctrl?cmd='+qs);
  const t=await r.text();
  if(!r.ok)alert('Ошибка: '+t);
 }catch(e){alert('Ошибка: '+e.message);}
}
function sendPower(v){cmd('power&on='+v);}
function sendWinter(v){cmd('winter&on='+v);}
function sendService(v){cmd('service&on='+v);}
function sendFanPwm(v){cmd('fan_pwm&pwm='+v);}
function sendTempMode(i,v){cmd('temp_mode&room='+(+i+1)+'&on='+v);}
function sendManual(i,v){cmd('manual&room='+(+i+1)+'&on='+v);}
function sendManualPos(i,v){cmd('manual_pos&room='+(+i+1)+'&pos='+v);}
function sendCo2(i){cmd('co2_target&room='+(+i+1)+'&val='+document.getElementById('co2_'+i).value);}
function sendRoomLimits(i){cmd('room_pos_limits&room='+(+i+1)+'&min='+document.getElementById('lim_min_'+i).value+'&max='+document.getElementById('lim_max_'+i).value);}
function sendDpLimits(){cmd('dp_limits&min='+document.getElementById('dp_min_inp').value+'&max='+document.getElementById('dp_max_inp').value);}
function sendFilterAlarmThreshold(){cmd('filter_alarm_threshold&val='+document.getElementById('filter_alarm_thresh_inp').value);}
function sendFlowAlarmThreshold(){cmd('flow_alarm_threshold&val='+document.getElementById('flow_alarm_thresh_inp').value);}
function sendDpWithinSetpointPct(){cmd('dp_within_setpoint_pct&val='+document.getElementById('dp_within_setpoint_pct_inp').value);}
function sendDpAlarmThresholds(){cmd('dp_alarm_thresholds&low='+document.getElementById('dp_alarm_low_inp').value+'&high='+document.getElementById('dp_alarm_high_inp').value);}
function sendPids(){cmd('pids&kp='+document.getElementById('pid_kp').value+'&ki='+document.getElementById('pid_ki').value+'&kd='+document.getElementById('pid_kd').value);}
function resetFan(){cmd('reset&target=fan');}
function resetHeater(){cmd('reset&target=heater');}
function resetFilter(){cmd('reset&target=filter');}
function blockReasonText(b){const m=['нет','нет свежих данных','аварийный форсаж CO₂'];return m[b]||('код '+b);}
function faultText(c){const m=['нет','низкое давление','высокое давление','нет датчика dP'];return m[c]||('код '+c);}
function heaterBlockText(b){const m=['нет','система выкл','зимний режим выкл','на улице тепло','фрикулинг','нет данных датчиков','вент. не готов','малый расход','нет датчика расхода','dP вне диапазона'];return m[b]||('код '+b);}

function renderControl(d){
 const el=document.getElementById('control'); if(!el)return;
 // Не перерисовывать, пока пользователь редактирует поле ввода (иначе автообновление
 // каждые 3с затирает вводимые значения). Активные поля PID на вкладке «Калибровка» — тоже.
 const ae=document.activeElement;
 const pidActive=['pid_kp','pid_ki','pid_kd'].some(id=>{const n=document.getElementById(id);return n&&n===ae;});
 if(ae && (el===ae || el.contains(ae) || pidActive)) return;
 let h='<table style="max-width:520px">'+
  '<tr><td>Улица</td><td>'+fnum(d.outdoor_temp,1)+' °C</td>'+
    '<td>Приток</td><td>'+fnum(d.heater.t_supply,1)+' °C</td></tr>'+
  '<tr><td>Перепад dP</td><td>'+fnum(d.fan.dp,0)+' / '+fnum(d.fan.dp_setpoint,0)+' Па</td>'+
    '<td>Расход</td><td>'+fnum(d.fan.flow_pct,0)+' %</td></tr>'+
  '<tr><td>Фаза вент.</td><td>'+d.fan.phase+'</td>'+
    '<td>ШИМ</td><td>'+fnum(d.fan.pwm,0)+' %</td></tr>'+
  '</table>'+
  '<h3>Общее</h3><table style="max-width:520px">'+
  '<tr><td>Установка</td><td>'+(d.power?'<b style="color:#30d158">ВКЛ</b>':'<b style="color:#ff9f0a">ВЫКЛ</b>')+'</td>'+
    '<td><button onclick="sendPower(1)">ВКЛ</button> <button onclick="sendPower(0)">ВЫКЛ</button></td></tr>'+
  '<tr><td>Зимний режим</td><td>'+(d.winter_mode?'ВКЛ':'ВЫКЛ')+'</td>'+
    '<td><input type="checkbox" '+(d.winter_mode?'checked':'')+' onchange="sendWinter(this.checked?1:0)"></td></tr>'+
  '<tr><td>Сервисный</td><td>'+(d.service_mode?'ВКЛ':'ВЫКЛ')+'</td>'+
    '<td><input type="checkbox" '+(d.service_mode?'checked':'')+' onchange="sendService(this.checked?1:0)"></td></tr>'+
  '</table>';

// Управление вентилятором в сервисном режиме
if (d.service_mode) {
  h += '<h3>Вентилятор (сервисный режим)</h3><table style="max-width:520px">'+
    '<tr><td>PWM:</td><td>'+fnum(d.fan.pwm,0)+' %</td>'+
    '<td><input type="range" id="fan_pwm_slider" min="0" max="100" value="'+Math.round(d.fan.service_pwm||0)+'" oninput="this.nextElementSibling.value=this.value" style="width:180px"> <output>'+Math.round(d.fan.service_pwm||0)+'</output> %</td>'+
    '<td><button onclick="sendFanPwm(document.getElementById(\'fan_pwm_slider\').value)">Установить</button></td></tr>'+
    '<tr><td>Статус сервиса:</td><td>'+(d.fan.in_service_mode?'<b style="color:#ff9f0a">активно</b>':'Ожидание')+'</td></tr>'+
    '</table>';
}
 (d.rooms||[]).forEach((x,i)=>{
  h+='<h3>Комната '+(i+1)+'</h3><table style="max-width:560px">'+
   '<tr><td>CO₂ '+fnum(x.co2,0)+' ('+fresh(x.co2_fresh)+')</td>'+
     '<td>Уставка</td><td><input id="co2_'+i+'" type="number" step="10" value="'+fnum(x.co2_target,0)+'"> <button onclick="sendCo2('+i+')">OK</button></td></tr>'+
   '<tr><td>Заслонка '+fnum(x.pos,0)+'%</td>'+
     '<td>Ручной</td><td><input type="checkbox" '+(x.manual_mode?'checked':'')+' onchange="sendManual('+i+',this.checked?1:0)"></td></tr>'+
   '<tr><td>Позиция</td><td colspan="2"><input type="range" min="0" max="'+(x.max_pos||100)+'" value="'+(x.manual_mode?(x.manual_pos||0):Math.min(x.pos||0,(x.max_pos||100)))+'" onchange="sendManualPos('+i+',this.value)" style="width:180px"></td></tr>'+
   '<tr><td>Фрикулинг</td><td colspan="2"><input type="checkbox" '+(x.temp_mode?'checked':'')+' onchange="sendTempMode('+i+',this.checked?1:0)"></td></tr>'+
   '<tr><td>Пределы %</td><td colspan="2">min <input id="lim_min_'+i+'" type="number" value="'+fnum(x.min_pos,0)+'"> max <input id="lim_max_'+i+'" type="number" value="'+fnum(x.max_pos,0)+'"> <button onclick="sendRoomLimits('+i+')">OK</button></td></tr>'+
   (x.block_reason>0?'<tr><td style="color:#f88">Блок</td><td colspan="2">'+blockReasonText(x.block_reason)+'</td></tr>':'')+
   '</table>';
 });
h+='<h3>Уставки давления</h3><table style="max-width:520px"><tr>'+
  '<td>min</td><td><input id="dp_min_inp" type="number" value="'+fnum(d.fan.dp_min,0)+'"></td>'+
  '<td>max</td><td><input id="dp_max_inp" type="number" value="'+fnum(d.fan.dp_max,0)+'"></td>'+
  '<td><button onclick="sendDpLimits()">OK</button></td></tr></table>';
h+='<h3>Аварийные пороги dP (Па)</h3><table style="max-width:520px"><tr>'+
  '<td>Low</td><td><input id="dp_alarm_low_inp" type="number" step="1" value="'+fnum(d.fan.dp_alarm_low,0)+'"></td>'+
  '<td>High</td><td><input id="dp_alarm_high_inp" type="number" step="1" value="'+fnum(d.fan.dp_alarm_high,0)+'"></td>'+
  '<td><button onclick="sendDpAlarmThresholds()">OK</button></td></tr></table>';
 h+='<h3>Порог расхода для аварии (%)</h3><table style="max-width:520px"><tr>'+
  '<td>Порог</td><td><input id="flow_alarm_thresh_inp" type="number" step="1" value="'+fnum(d.fan.flow_alarm_threshold,0)+'"></td>'+
  '<td><button onclick="sendFlowAlarmThreshold()">OK</button></td></tr></table>';
 h+='<h3>Допуск dP в уставке (%)</h3><table style="max-width:520px"><tr>'+
  '<td>Допуск</td><td><input id="dp_within_setpoint_pct_inp" type="number" step="1" value="'+fnum(d.fan.dp_within_setpoint_pct,0)+'"></td>'+
  '<td><button onclick="sendDpWithinSetpointPct()">OK</button></td></tr></table>';
 h+='<h3>Фильтр (порог аварии, Па)</h3><table style="max-width:520px"><tr>'+
  '<td>Порог</td><td><input id="filter_alarm_thresh_inp" type="number" step="1" value="'+fnum(d.filter.alarm_threshold,0)+'"></td>'+
  '<td><button onclick="sendFilterAlarmThreshold()">OK</button></td></tr></table>';
 h+='<h3>Ошибки</h3><table style="max-width:560px">'+
  '<tr><td>Вентилятор</td><td>'+(d.fan.fault_latched?'<b style="color:#ff453a">'+faultText(d.fan.fault_code)+'</b>':'<span style="color:#30d158">Ок</span>')+'</td>'+
    '<td><button onclick="resetFan()">Сброс</button></td></tr>'+
  '<tr><td>Нагреватель</td><td>'+(d.heater.fault_latched?'<b style="color:#ff453a">'+faultText(d.heater.fault_code)+'</b>':'<span style="color:#30d158">Ок</span>')+'</td>'+
    '<td><button onclick="resetHeater()">Сброс</button></td></tr>'+
  '<tr><td>Фильтр</td><td>'+( (d.filter && !d.filter.sensor_ok) ? '<b style="color:#ff453a">Ошибка датчика</b>' : (d.filter && d.filter.alarm_active ? '<b style="color:#ff453a">Тревога фильтра</b>' : '<span style="color:#30d158">Ок</span>') )+'</td>'+
    '<td><button onclick="resetFilter()">Сброс</button></td></tr>'+
  (d.heater.block_reason>0?'<tr><td>Блок нагревателя</td><td colspan="2">'+heaterBlockText(d.heater.block_reason)+'</td></tr>':'')+
  '</table>';
 el.innerHTML=h;
 const pk=document.getElementById('pid_kp'); if(pk)pk.value=d.fan.kp;
 const pi=document.getElementById('pid_ki'); if(pi)pi.value=d.fan.ki;
 const pd=document.getElementById('pid_kd'); if(pd)pd.value=d.fan.kd;
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
        doc["outdoor_temp"] = sensors.outdoorTempValid() ? sensors.outdoorTemp() : NAN;

        JsonObject fanObj = doc.createNestedObject("fan");
        fanObj["phase"] = (int)fan.phase();
        fanObj["pwm"] = fan.pwmPct();
        fanObj["dp"] = fan.dpMeasured();
        fanObj["dp_setpoint"] = fan.dpSetpoint();
        fanObj["dp_min"] = storage.getDpMin();
        fanObj["dp_max"] = storage.getDpMax();
        fanObj["dp_alarm_low"] = storage.getDpAlarmLow();
        fanObj["dp_alarm_high"] = storage.getDpAlarmHigh();
        fanObj["flow_alarm_threshold"] = storage.getFlowAlarmThreshold();
        fanObj["dp_within_setpoint_pct"] = storage.getDpWithinSetpointPct();
        fanObj["flow_pct"] = sensors.flowPct();
        fanObj["fault_latched"] = fan.faultLatched();
        fanObj["fault_code"] = fan.faultCode();
        fanObj["fault_dp_at_trip"] = fan.faultPressureAtTrip();
        fanObj["kp"] = fan.dpKp();
        fanObj["ki"] = fan.dpKi();
        fanObj["kd"] = fan.dpKd();

        JsonObject filterObj = doc.createNestedObject("filter");
        filterObj["pressure"] = sensors.filterSensorOk() ? sensors.filterPressurePa() : NAN;
        filterObj["sensor_ok"] = sensors.filterSensorOk();
        filterObj["alarm_active"] = sensors.filterAlarmActive();
        filterObj["alarm_threshold"] = sensors.getFilterAlarmThreshold();

        JsonObject heaterObj = doc.createNestedObject("heater");
        heaterObj["stage"] = heater.currentStage();
        heaterObj["t_supply"] = sensors.supplyTempValid() ? sensors.supplyTemp() : NAN;
        heaterObj["fault_latched"] = heater.faultLatched();
        heaterObj["fault_code"] = heater.faultCode();
        heaterObj["fault_temp_at_trip"] = heater.faultTempAtTrip();
        heaterObj["block_reason"] = (int)heater.blockReason();

        JsonArray rooms = doc.createNestedArray("rooms");
        for (uint8_t i = 0; i < ROOM_COUNT; i++) {
            RoomData rd = sensors.getRoomData(i);
            RoomSettings rs = storage.getRoom(i);
            JsonObject ro = rooms.createNestedObject();
            ro["co2"] = rd.co2;
            ro["temp"] = rd.temp;
            ro["co2_fresh"] = sensors.isRoomCo2Fresh(i);
            ro["temp_fresh"] = sensors.isRoomTempFresh(i);
            ro["data_fresh"] = sensors.isRoomDataFresh(i);
            ro["pos"] = dampers.getPos(i);
            ro["manual_mode"] = dampers.isManual(i);
            ro["manual_pos"] = dampers.getManualPos(i);
            ro["temp_mode"] = rs.temp_mode_enable;
            ro["co2_target"] = rs.co2_target;
            ro["min_pos"] = rs.min_pos;
            ro["max_pos"] = rs.max_pos;
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
        else if (sensor == "filter") sensors.setFilterCal(offset, scale);
        else { req->send(400, "text/plain", "bad sensor"); return; }
        req->send(200, "text/plain", "ok");
    });

    server_.on("/api/ctrl", HTTP_GET, [](AsyncWebServerRequest *req) {
        if (!req->hasParam("cmd")) { req->send(400, "text/plain", "missing cmd"); return; }
        String cmd = req->getParam("cmd")->value();
        int room = req->hasParam("room") ? req->getParam("room")->value().toInt() - 1 : -1;

        if (cmd == "power")                { mqttManager.setPower(req->getParam("on")->value().toInt() != 0); }
        else if (cmd == "winter")          { storage.setWinterMode(req->getParam("on")->value().toInt() != 0); }
        else if (cmd == "service")         { mqttManager.setServiceMode(req->getParam("on")->value().toInt() != 0); }
        else if (cmd == "fan_pwm")         { 
            float pwm = req->getParam("pwm")->value().toFloat();
            fan.setServicePwm(pwm);
        }
        else if (cmd == "reset") {
            String t = req->getParam("target")->value();
            if (t == "fan")    fan.resetFault();
            else if (t == "heater") heater.resetFault();
            else if (t == "filter") sensors.resetFilterAlarm();
            else { req->send(400, "text/plain", "bad target"); return; }
        }
        else if (cmd == "temp_mode") {
            if (room < 0 || room >= ROOM_COUNT) { req->send(400, "text/plain", "bad room"); return; }
            bool en = req->getParam("on")->value().toInt() != 0;
            if (en && (!sensors.outdoorTempValid() ||
                       sensors.outdoorTemp() < storage.getFreecoolOutdoorMin())) {
                eventLog.add("temp_mode room" + String(room + 1) + ": отклонено (низкая T улицы)");
                req->send(200, "text/plain", "blocked: low outdoor temp"); return;
            }
            storage.setRoomTempModeEnable((uint8_t)room, en);
        }
        else if (cmd == "manual") {
            if (room < 0 || room >= ROOM_COUNT) { req->send(400, "text/plain", "bad room"); return; }
            bool en = req->getParam("on")->value().toInt() != 0;
            dampers.setManualMode((uint8_t)room, en);
            // Сервисный режим: немедленно применяем позицию
            if (en && mqttManager.serviceMode()) {
                float pos = dampers.getManualPos((uint8_t)room);
                dampers.serviceSetAngle((uint8_t)room, pos / 100.0f * 180.0f);
            }
        }
        else if (cmd == "manual_pos") {
            if (room < 0 || room >= ROOM_COUNT) { req->send(400, "text/plain", "bad room"); return; }
            float pos = req->getParam("pos")->value().toFloat();
            dampers.setManualPos((uint8_t)room, pos);
            // Сервисный режим: пишем в серво напрямую (update() не вызывается)
            if (mqttManager.serviceMode()) {
                dampers.serviceSetAngle((uint8_t)room, pos / 100.0f * 180.0f);
            }
        }
        else if (cmd == "co2_target") {
            if (room < 0 || room >= ROOM_COUNT) { req->send(400, "text/plain", "bad room"); return; }
            storage.setRoomCo2Target((uint8_t)room, req->getParam("val")->value().toFloat());
        }
        else if (cmd == "room_pos_limits") {
            if (room < 0 || room >= ROOM_COUNT) { req->send(400, "text/plain", "bad room"); return; }
            storage.setRoomMinPos((uint8_t)room, req->getParam("min")->value().toFloat());
            storage.setRoomMaxPos((uint8_t)room, req->getParam("max")->value().toFloat());
        }
        else if (cmd == "dp_limits") {
            storage.setDpMin(req->getParam("min")->value().toFloat());
            storage.setDpMax(req->getParam("max")->value().toFloat());
        }
        else if (cmd == "dp_alarm_thresholds") {
            storage.setDpAlarmLow(req->getParam("low")->value().toFloat());
            storage.setDpAlarmHigh(req->getParam("high")->value().toFloat());
        }
        else if (cmd == "filter_alarm_threshold") {
            sensors.setFilterAlarmThreshold(req->getParam("val")->value().toFloat());
        }
        else if (cmd == "flow_alarm_threshold") {
            storage.setFlowAlarmThreshold(req->getParam("val")->value().toFloat());
        }
        else if (cmd == "dp_within_setpoint_pct") {
            storage.setDpWithinSetpointPct(req->getParam("val")->value().toFloat());
        }
        else if (cmd == "pids") {
            fan.setDpTunings(req->getParam("kp")->value().toFloat(),
                             req->getParam("ki")->value().toFloat(),
                             req->getParam("kd")->value().toFloat());
        }
        else if (cmd == "servo") {
            if (room < 0 || room >= ROOM_COUNT) { req->send(400, "text/plain", "bad room"); return; }
            dampers.serviceSetPercent((uint8_t)room, req->getParam("pos")->value().toFloat());
        }
        else { req->send(400, "text/plain", "unknown cmd"); return; }

        req->send(200, "text/plain", "ok");
    });

    server_.on("/api/calib", HTTP_GET, [](AsyncWebServerRequest *req) {
        DynamicJsonDocument doc(1024);
        JsonArray servos = doc.createNestedArray("servo");
        for (uint8_t i = 0; i < ROOM_COUNT; i++) {
            uint16_t pmin, pmax;
            storage.getServoCal(i, pmin, pmax);
            JsonObject o = servos.createNestedObject();
            o["room"] = i + 1;
            o["pmin"] = pmin;
            o["pmax"] = pmax;
        }
        float off, sc;
        storage.getAnalogCal("dp", off, sc);
        doc["dp"]["offset"] = off;
        doc["dp"]["scale"] = sc;
        storage.getAnalogCal("flow", off, sc);
        doc["flow"]["offset"] = off;
        doc["flow"]["scale"] = sc;
        storage.getAnalogCal("filter", off, sc);
        doc["filter"]["offset"] = off;
        doc["filter"]["scale"] = sc;
        String out;
        serializeJson(doc, out);
        req->send(200, "application/json", out);
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
        doc["client_id"] = cfg.client_id[0] ? cfg.client_id : MQTT_CLIENT_ID;
        doc["root"] = cfg.root[0] ? cfg.root : MQTT_ROOT;
        String out;
        serializeJson(doc, out);
        req->send(200, "application/json", out);
    });

    server_.on("/api/mqtt/config", HTTP_POST,
        [](AsyncWebServerRequest *){},
        nullptr,
        [](AsyncWebServerRequest *req, uint8_t *data, size_t len, size_t index, size_t total){
            if (index + len != total) return; // обрабатываем только финальный фрагмент тела

            DynamicJsonDocument doc(512);
            if (deserializeJson(doc, data, len) != DeserializationError::Ok) {
                req->send(400, "text/plain", "invalid json"); return;
            }
            const char* server = doc["server"] | "";
            if (strlen(server) == 0) { req->send(400, "text/plain", "missing server"); return; }
            uint16_t port      = doc["port"] | 1883;
            const char* user   = doc["user"] | "";
            const char* pass   = doc["pass"] | "";
            const char* client_id = doc["client_id"] | "";
            const char* root   = doc["root"] | "";

            // Сохраняем в NVS
            storage.setMqttServer(server);
            storage.setMqttPort(port);
            storage.setMqttUser(user);
            storage.setMqttPass(pass);
            storage.setMqttClientId(client_id);
            storage.setMqttRoot(root);

            // Перезагружаем ESP32 для применения настроек
            req->send(200, "text/plain", "ok");
            delay(500);
            ESP.restart();
        });
}
