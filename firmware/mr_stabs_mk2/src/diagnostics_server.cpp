#include "diagnostics_server.h"

#include <ESPAsyncWebServer.h>
#include <WiFi.h>

static AsyncWebServer server(80);
static AsyncEventSource events("/events");
static bool server_started = false;
static portMUX_TYPE status_mux = portMUX_INITIALIZER_UNLOCKED;
static sensor_status_t shared_status = {};

static const char INDEX_HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>MR STABS Diagnostics</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{font-family:monospace;background:#1a1a2e;color:#e0e0e0;padding:16px}
h1{color:#0ff;margin-bottom:12px;font-size:1.4em}
h2{color:#0ff;margin:12px 0 6px;font-size:1.1em}
td.bad{color:#f55}
table{border-collapse:collapse;width:100%;max-width:600px;margin-bottom:16px}
td{padding:4px 10px;border-bottom:1px solid #333}
td:first-child{color:#888;width:40%}
td:last-child{color:#0f0;text-align:right}
.btn{display:inline-block;padding:8px 20px;margin:4px;border:none;border-radius:4px;
font-family:monospace;font-size:1em;cursor:pointer;color:#fff}
.rec{background:#c00}.rec.active{background:#0a0}
.dl{background:#06c}
.status{margin:12px 0;padding:8px;border-radius:4px;font-size:0.9em}
.connected{background:#0a3}
.disconnected{background:#600}
#count{color:#ff0;margin-left:8px}
</style>
</head>
<body>
<h1>MR STABS Diagnostics</h1>
<div id="conn" class="status disconnected">Connecting...</div>
<table>
<tr><td>timestamp_ms</td><td id="v_ts">-</td></tr>
<tr><td>radio_connected</td><td id="v_radio">-</td></tr>
<tr><td>armed</td><td id="v_armed">-</td></tr>
<tr><td>a_percent</td><td id="v_a">-</td></tr>
<tr><td>b_percent</td><td id="v_b">-</td></tr>
<tr><td>button_state</td><td id="v_btn">-</td></tr>
<tr><td>flip_switch</td><td id="v_flip">-</td></tr>
<tr><td>left_cmd</td><td id="v_left">-</td></tr>
<tr><td>right_cmd</td><td id="v_right">-</td></tr>
<tr><td>accel_x</td><td id="v_ax">-</td></tr>
<tr><td>accel_y</td><td id="v_ay">-</td></tr>
<tr><td>accel_z</td><td id="v_az">-</td></tr>
<tr><td>is_upside_down</td><td id="v_usd">-</td></tr>
<tr><td>loop_us</td><td id="v_loop">-</td></tr>
<tr><td>wifi_clients</td><td id="v_wifi">-</td></tr>
<tr><td>orientation_x</td><td id="v_ox">-</td></tr>
<tr><td>orientation_y</td><td id="v_oy">-</td></tr>
<tr><td>orientation_z</td><td id="v_oz">-</td></tr>
<tr><td>pid_setpoint</td><td id="v_sp">-</td></tr>
<tr><td>pid_output</td><td id="v_po">-</td></tr>
<tr><td>vbat</td><td id="v_vbat">-</td></tr>
<tr><td>ibat</td><td id="v_ibat">-</td></tr>
<tr><td>yaw_rate (deg/s, CW)</td><td id="v_yr">-</td></tr>
<tr><td>yaw_rate_cmd (deg/s, CW)</td><td id="v_yrc">-</td></tr>
</table>
<div style="margin-bottom:16px;padding:10px;border:1px solid #444;border-radius:4px;max-width:600px">
<div style="margin-bottom:8px">
<label style="color:#ff0">Left ESC Deadzone (%):</label>
<input type="number" id="dzL" value="1" min="0" max="50" step="0.5"
 style="width:80px;background:#222;color:#0f0;border:1px solid #555;padding:4px;font-family:monospace">
<button class="btn" style="background:#555;padding:4px 12px" onclick="setTune('left_esc_dz','dzL','dzLS')">Apply</button>
<span id="dzLS" style="margin-left:8px;color:#888"></span>
</div>
<div>
<label style="color:#ff0">Right ESC Deadzone (%):</label>
<input type="number" id="dzR" value="1" min="0" max="50" step="0.5"
 style="width:80px;background:#222;color:#0f0;border:1px solid #555;padding:4px;font-family:monospace">
<button class="btn" style="background:#555;padding:4px 12px" onclick="setTune('right_esc_dz','dzR','dzRS')">Apply</button>
<span id="dzRS" style="margin-left:8px;color:#888"></span>
</div>
</div>
<h2>I2C bus <span id="busPins" style="color:#888;font-size:0.8em"></span></h2>
<table id="busT"></table>
<button class="btn" style="background:#555;padding:4px 12px" onclick="rescan()">Rescan bus</button>
<span id="scanS" style="margin-left:8px;color:#888"></span>
<h2>BNO055 IMU</h2>
<table id="imuT"></table>
<h2>INA228 pack sensor</h2>
<table id="inaT"></table>
<button class="btn rec" id="recBtn" onclick="toggleRec()">Record</button>
<button class="btn dl" onclick="downloadCSV()">Download CSV</button>
<span id="count"></span>
<script>
const hdr='timestamp_ms,radio_connected,armed,a_percent,b_percent,button_state,flip_switch,left_cmd,right_cmd,accel_x,accel_y,accel_z,is_upside_down,loop_us,wifi_clients,orientation_x,orientation_y,orientation_z,pid_setpoint,pid_output,vbat,ibat,yaw_rate,yaw_rate_cmd';
const ids=['v_ts','v_radio','v_armed','v_a','v_b','v_btn','v_flip','v_left','v_right','v_ax','v_ay','v_az','v_usd','v_loop','v_wifi','v_ox','v_oy','v_oz','v_sp','v_po','v_vbat','v_ibat','v_yr','v_yrc'];
let rows=[];
let recording=false;
let es;
function connect(){
 es=new EventSource('/events');
 es.onopen=()=>{document.getElementById('conn').className='status connected';document.getElementById('conn').textContent='Connected';};
 es.onerror=()=>{document.getElementById('conn').className='status disconnected';document.getElementById('conn').textContent='Disconnected';};
 es.onmessage=(e)=>{
  const f=e.data.split(',');
  for(let i=0;i<ids.length&&i<f.length;i++)document.getElementById(ids[i]).textContent=f[i];
  if(recording){rows.push(e.data);document.getElementById('count').textContent=rows.length+' rows';}
 };
}
function toggleRec(){
 recording=!recording;
 const b=document.getElementById('recBtn');
 if(recording){b.textContent='Stop';b.classList.add('active');rows=[];document.getElementById('count').textContent='';
  fetch('/record/start');
 }else{b.textContent='Record';b.classList.remove('active');
  fetch('/record/stop');
  document.getElementById('count').textContent=rows.length+' rows (stopped)';
 }
}
function downloadCSV(){
 if(rows.length===0){alert('No recorded data');return;}
 const blob=new Blob([hdr+'\n'+rows.join('\n')+'\n'],{type:'text/csv'});
 const a=document.createElement('a');a.href=URL.createObjectURL(blob);
 a.download='mr_stabs_'+new Date().toISOString().slice(0,19).replace(/[:-]/g,'')+'.csv';
 a.click();URL.revokeObjectURL(a.href);
}
function setTune(ep,inputId,statusId){
 const v=document.getElementById(inputId).value;
 fetch('/tune/'+ep+'?value='+v).then(r=>r.text()).then(t=>{
  document.getElementById(statusId).textContent='Set to '+t;
  setTimeout(()=>document.getElementById(statusId).textContent='',3000);
 });
}
function loadTune(ep,inputId){fetch('/tune/'+ep).then(r=>r.text()).then(v=>{document.getElementById(inputId).value=v;});}
const I2C_ERR={0:'ok',1:'data too long',2:'address NACK: device not answering',3:'data NACK',4:'bus error',5:'timeout: bus stuck?',6:'short read',255:'never tried'};
const KNOWN={0x28:'BNO055',0x29:'BNO055 (ADR high)',0x41:'INA228 (alt)',0x44:'INA228 (alt)',0x45:'INA228'};
const SYS_STATUS=['idle','system error','initializing peripherals','system init','running self-test','fusion running','running without fusion'];
const SYS_ERR=['none','peripheral init error','system init error','self-test failed','register value out of range','register address out of range','register write error','low power mode not available','accel power mode not available','fusion config error','sensor config error'];
const MODES={0:'CONFIG',8:'IMUPLUS',12:'NDOF'};
function hex(v,n){return '0x'+v.toString(16).toUpperCase().padStart(n||2,'0');}
function age(ms){return ms==null?'never':(ms/1000).toFixed(1)+' s ago';}
function err(c){return [I2C_ERR[c]||('error '+c),c!==0&&c!==255];}
function fill(id,rows){document.getElementById(id).innerHTML=rows.map(r=>'<tr><td>'+r[0]+'</td><td'+(r[2]?' class="bad"':'')+'>'+r[1]+'</td></tr>').join('');}
function selfTest(v){const n=['accel','mag','gyro','MCU'];const f=n.filter((_,i)=>!(v>>i&1));return [f.length?'failed: '+f.join(', '):'all passed ('+hex(v)+')',f.length>0];}
function calib(v){return 'sys '+(v>>6&3)+', gyro '+(v>>4&3)+', accel '+(v>>2&3)+', mag '+(v&3)+' (of 3)';}
let prevImu=null;
function sampleRate(s){
 const p=prevImu;prevImu={t:s.snapshot_ms,n:s.imu.samples};
 if(!p||s.snapshot_ms<=p.t||s.imu.samples<p.n)return '-';
 return ((s.imu.samples-p.n)*1000/(s.snapshot_ms-p.t)).toFixed(0)+' Hz';
}
function renderStatus(s){
 document.getElementById('busPins').textContent='Wire1, SDA '+s.bus.sda_pin+', SCL '+s.bus.scl_pin;
 const found=s.bus.found.map(a=>hex(a)+(KNOWN[a]?' '+KNOWN[a]:'')).join(', ')||'nothing';
 fill('busT',[
  ['SDA idle level',s.bus.sda_high?'high':'LOW: held by a device',!s.bus.sda_high],
  ['SCL idle level',s.bus.scl_high?'high':'LOW: held by a device',!s.bus.scl_high],
  ['devices found',s.bus.scanned?found:'not scanned',s.bus.scanned&&!s.bus.found.includes(0x28)],
  ['last scan',s.bus.scanned?age(s.bus.scan_age_ms):'never'],
 ]);
 document.getElementById('scanS').textContent=s.bus.scan_pending?'scan queued (runs while disarmed)':'';
 const m=s.imu;
 const rate=sampleRate(s);
 const imuRows=[
  ['initialized',m.initialized?'yes':'NO',!m.initialized],
  ['begin attempts / failures',m.begin_attempts+' / '+m.begin_failures,m.begin_failures>0],
  ['chip id',m.last_error===255?'not read':hex(m.chip_id)+(m.chip_id===0xA0?' (BNO055)':' (expected 0xA0)'),m.last_error===0&&m.chip_id!==0xA0],
  ['last chip id read'].concat(err(m.last_error)),
  ['dropouts after init',m.lost_count,m.lost_count>0],
  ['samples',m.samples],
  ['sample rate (expect ~100 Hz)',rate],
  ['last sample',m.sample_age_ms==null?'never':m.sample_age_ms+' ms before snapshot',m.initialized&&(m.sample_age_ms==null||m.sample_age_ms>100)],
 ];
 if(m.details_age_ms!=null){
  if(m.details_error!==0)imuRows.push(['status registers'].concat(err(m.details_error)));
  else imuRows.push(
   ['operation mode',(MODES[m.operation_mode]||'mode')+' ('+hex(m.operation_mode)+')',m.operation_mode!==8],
   ['system status',SYS_STATUS[m.sys_status]||hex(m.sys_status),m.sys_status!==5],
   ['system error',SYS_ERR[m.sys_error]||hex(m.sys_error),m.sys_error!==0],
   ['self-test'].concat(selfTest(m.self_test)),
   ['calibration',calib(m.calibration)],
   ['status read',age(m.details_age_ms)]);
 }
 imuRows.push(
  ['yaw rate: gyro / heading change',m.yaw_rate.toFixed(0)+' / '+m.heading_rate.toFixed(0)+' deg/s CW'],
  ['gyro sign check (spin it)',m.gyro_sign_suspect?'FAILED: yaw loop off, flip YAW_RATE_SIGN':(m.gyro_agree+m.gyro_disagree===0?'no votes yet':'agree '+m.gyro_agree+', disagree '+m.gyro_disagree),m.gyro_sign_suspect]);
 fill('imuT',imuRows);
 const n=s.ina;
 fill('inaT',[
  ['present',n.present?'yes':'NO',!n.present],
  ['device id',hex(n.device_id,4)+((n.device_id>>4)===0x228?' (INA228)':' (expected 0x228x)'),(n.device_id>>4)!==0x228],
  ['last read'].concat(err(n.last_error)),
  ['reads / failures',n.reads+' / '+n.read_failures,n.read_failures>0],
  ['last good read',age(n.last_ok_age_ms)],
 ]);
}
function pollStatus(){fetch('/status').then(r=>r.json()).then(renderStatus).catch(()=>{});}
function rescan(){fetch('/i2c/scan').then(()=>{document.getElementById('scanS').textContent='scan queued';});}
setInterval(pollStatus,1000);
pollStatus();
loadTune('left_esc_dz','dzL');
loadTune('right_esc_dz','dzR');
connect();
</script>
</body>
</html>
)rawhtml";

static void handle_tunable(AsyncWebServerRequest *request, float *ptr) {
    if (!ptr) {
        request->send(500, "text/plain", "n/a");
        return;
    }
    if (request->hasParam("value")) *ptr = request->getParam("value")->value().toFloat();
    request->send(200, "text/plain", String(*ptr, 1));
}

// Age in ms of a millis() stamp at snapshot time, or JSON null when it never happened.
static String age_or_null(bool happened, uint32_t stamp_ms, uint32_t snapshot_ms) {
    return happened ? String(snapshot_ms - stamp_ms) : String("null");
}

static String status_json(const sensor_status_t &s, bool scan_pending) {
    String found = "[";
    for (uint8_t i = 0; i < s.scan.count; i++) {
        if (i) found += ",";
        found += String(s.scan.addresses[i]);
    }
    found += "]";

    const updown_sensor::status_t &m = s.imu;
    const vbat_sensor::status_t &n = s.ina;
    char buf[900];
    snprintf(buf, sizeof(buf),
             "{\"snapshot_ms\":%lu,\"bus\":{\"sda_pin\":%u,\"scl_pin\":%u,\"sda_high\":%s,\"scl_high\":%s,"
             "\"scanned\":%s,\"scan_age_ms\":%s,\"scan_pending\":%s,\"found\":%s},"
             "\"imu\":{\"initialized\":%s,\"begin_attempts\":%lu,\"begin_failures\":%lu,"
             "\"chip_id\":%u,\"last_error\":%u,\"lost_count\":%lu,\"samples\":%lu,"
             "\"sample_age_ms\":%s,\"details_age_ms\":%s,\"details_error\":%u,"
             "\"operation_mode\":%u,\"sys_status\":%u,\"self_test\":%u,\"sys_error\":%u,"
             "\"calibration\":%u,\"gyro_agree\":%lu,\"gyro_disagree\":%lu,\"yaw_rate\":%.1f,"
             "\"heading_rate\":%.1f,\"gyro_sign_suspect\":%s},"
             "\"ina\":{\"present\":%s,\"device_id\":%u,\"last_error\":%u,\"reads\":%lu,"
             "\"read_failures\":%lu,\"last_ok_age_ms\":%s}}",
             (unsigned long)s.snapshot_ms, s.sda_pin, s.scl_pin, s.lines.sda_high ? "true" : "false",
             s.lines.scl_high ? "true" : "false", s.scan.scanned ? "true" : "false",
             age_or_null(s.scan.scanned, s.scan.scan_ms, s.snapshot_ms).c_str(), scan_pending ? "true" : "false", found.c_str(),
             m.initialized ? "true" : "false", (unsigned long)m.begin_attempts,
             (unsigned long)m.begin_failures, m.chip_id, m.last_error,
             (unsigned long)m.lost_count, (unsigned long)m.samples,
             age_or_null(m.samples > 0, m.last_sample_ms, s.snapshot_ms).c_str(),
             age_or_null(m.details_ms != 0, m.details_ms, s.snapshot_ms).c_str(), m.details_error,
             m.operation_mode, m.sys_status, m.self_test, m.sys_error, m.calibration,
             (unsigned long)m.gyro_agree, (unsigned long)m.gyro_disagree, s.yaw_rate,
             s.heading_rate, s.gyro_sign_suspect ? "true" : "false",
             n.present ? "true" : "false", n.device_id, n.last_error, (unsigned long)n.reads,
             (unsigned long)n.read_failures, age_or_null(n.last_ok_ms != 0, n.last_ok_ms, s.snapshot_ms).c_str());
    return String(buf);
}

void DiagnosticsServer::begin(tunable_ptrs_t tunables) {
    _tunables = tunables;

    server.on("/", HTTP_GET,
              [](AsyncWebServerRequest *request) { request->send(200, "text/html", INDEX_HTML); });

    server.on("/record/start", HTTP_GET, [this](AsyncWebServerRequest *request) {
        _recording = true;
        request->send(200, "text/plain", "ok");
    });

    server.on("/record/stop", HTTP_GET, [this](AsyncWebServerRequest *request) {
        _recording = false;
        request->send(200, "text/plain", "ok");
    });

    server.on("/tune/left_esc_dz", HTTP_GET, [this](AsyncWebServerRequest *request) {
        handle_tunable(request, _tunables.left_esc_deadzone);
    });

    server.on("/tune/right_esc_dz", HTTP_GET, [this](AsyncWebServerRequest *request) {
        handle_tunable(request, _tunables.right_esc_deadzone);
    });

    server.on("/status", HTTP_GET, [this](AsyncWebServerRequest *request) {
        sensor_status_t s;
        portENTER_CRITICAL(&status_mux);
        s = shared_status;
        portEXIT_CRITICAL(&status_mux);
        request->send(200, "application/json", status_json(s, _scan_requested));
    });

    server.on("/i2c/scan", HTTP_GET, [this](AsyncWebServerRequest *request) {
        _scan_requested = true;
        request->send(200, "text/plain", "ok");
    });

    events.onConnect(
        [](AsyncEventSourceClient *client) { client->send("connected", NULL, millis(), 1000); });

    server.addHandler(&events);
    server.begin();
    server_started = true;
}

bool DiagnosticsServer::has_clients() { return server_started && events.count() > 0; }

void DiagnosticsServer::set_status(const sensor_status_t &status) {
    portENTER_CRITICAL(&status_mux);
    shared_status = status;
    portEXIT_CRITICAL(&status_mux);
}

void DiagnosticsServer::update(const diag_data_t *data) {
    if (!server_started || events.count() == 0) return;

    uint32_t now = millis();
    if (!_recording && (now - _last_send_ms < 100)) return;
    _last_send_ms = now;

    char buf[320];
    snprintf(
        buf, sizeof(buf),
        "%lu,%d,%d,%.1f,%.1f,%d,%u,%.1f,%.1f,%.1f,%.1f,%.1f,%d,%lu,%u,%.1f,%.1f,%.1f,%.1f,%.2f,%.3f,%.2f,%.1f,%.1f",
        (unsigned long)data->timestamp_ms, data->radio_connected, data->armed, data->a_percent,
        data->b_percent, data->button_state, data->flip_switch, data->left_cmd, data->right_cmd,
        data->accel_x, data->accel_y, data->accel_z, data->is_upside_down,
        (unsigned long)data->loop_us, data->wifi_clients, data->orientation_x, data->orientation_y,
        data->orientation_z, data->pid_setpoint, data->pid_output, data->vbat, data->ibat,
        data->yaw_rate, data->yaw_rate_cmd);

    events.send(buf, NULL, now);
}
