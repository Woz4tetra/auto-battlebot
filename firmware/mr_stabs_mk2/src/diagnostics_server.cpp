#include "diagnostics_server.h"
#include "debug_log.h"
#include <WiFi.h>
#include <ESPAsyncWebServer.h>

static AsyncWebServer server(80);
static AsyncEventSource events("/events");
static bool server_started = false;

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
<div style="margin-bottom:12px"><a href="/debug" style="color:#f80">Debug Log</a></div>
<div id="conn" class="status disconnected">Connecting...</div>
<table>
<tr><td>timestamp_ms</td><td id="v_ts">-</td></tr>
<tr><td>mode</td><td id="v_mode">-</td></tr>
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
</table>
<button class="btn rec" id="recBtn" onclick="toggleRec()">Record</button>
<button class="btn dl" onclick="downloadCSV()">Download CSV</button>
<span id="count"></span>
<script>
const hdr='timestamp_ms,mode,radio_connected,armed,a_percent,b_percent,button_state,flip_switch,left_cmd,right_cmd,accel_x,accel_y,accel_z,is_upside_down,loop_us,wifi_clients';
const ids=['v_ts','v_mode','v_radio','v_armed','v_a','v_b','v_btn','v_flip','v_left','v_right','v_ax','v_ay','v_az','v_usd','v_loop','v_wifi'];
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
connect();
</script>
</body>
</html>
)rawhtml";

static const char DEBUG_HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>MR STABS Debug Log</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{font-family:monospace;background:#1a1a2e;color:#e0e0e0;padding:16px}
h1{color:#f80;margin-bottom:12px;font-size:1.4em}
pre{background:#111;color:#0f0;padding:12px;border-radius:4px;overflow-x:auto;
max-height:80vh;overflow-y:auto;font-size:0.85em;white-space:pre-wrap;word-break:break-all}
.bar{margin-bottom:12px}
.btn{display:inline-block;padding:8px 16px;margin:4px;border:none;border-radius:4px;
font-family:monospace;font-size:1em;cursor:pointer;color:#fff}
.refresh{background:#06c}
.clear{background:#c00}
.auto{background:#080}
.auto.off{background:#555}
#status{color:#888;margin-left:12px;font-size:0.9em}
</style>
</head>
<body>
<h1>MR STABS Debug Log</h1>
<div class="bar">
<button class="btn refresh" onclick="poll()">Refresh</button>
<button class="btn clear" onclick="clearLog()">Clear</button>
<button class="btn auto" id="autoBtn" onclick="toggleAuto()">Auto: ON</button>
<span id="status"></span>
</div>
<pre id="log"></pre>
<script>
let auto_poll=true,timer=null,seen=new Set();
function poll(){
 fetch('/debuglog').then(r=>r.text()).then(t=>{
  if(!t){return;}
  const el=document.getElementById('log');
  const lines=t.split('\n');
  let added=false;
  for(const ln of lines){
   if(!ln||seen.has(ln))continue;
   seen.add(ln);
   el.textContent+=ln+'\n';
   added=true;
  }
  if(added)el.scrollTop=el.scrollHeight;
  document.getElementById('status').textContent='Updated '+new Date().toLocaleTimeString();
 }).catch(()=>{document.getElementById('status').textContent='fetch error';});
}
function clearLog(){
 fetch('/debugclear').then(()=>{
  seen.clear();
  document.getElementById('log').textContent='';
 });
}
function toggleAuto(){
 auto_poll=!auto_poll;
 const b=document.getElementById('autoBtn');
 if(auto_poll){b.textContent='Auto: ON';b.classList.remove('off');startTimer();}
 else{b.textContent='Auto: OFF';b.classList.add('off');if(timer)clearInterval(timer);}
}
function startTimer(){if(timer)clearInterval(timer);timer=setInterval(poll,500);}
poll();startTimer();
</script>
</body>
</html>
)rawhtml";

void DiagnosticsServer::begin()
{
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send(200, "text/html", INDEX_HTML); });

    server.on("/record/start", HTTP_GET, [this](AsyncWebServerRequest *request)
              { _recording = true; request->send(200, "text/plain", "ok"); });

    server.on("/record/stop", HTTP_GET, [this](AsyncWebServerRequest *request)
              { _recording = false; request->send(200, "text/plain", "ok"); });

    server.on("/debug", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send(200, "text/html", DEBUG_HTML); });

    server.on("/debuglog", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send(200, "text/plain", debug_log_get_all()); });

    server.on("/debugclear", HTTP_GET, [](AsyncWebServerRequest *request)
              { debug_log_clear(); request->send(200, "text/plain", "ok"); });

    events.onConnect([](AsyncEventSourceClient *client)
                     { client->send("connected", NULL, millis(), 1000); });

    server.addHandler(&events);
    server.begin();
    server_started = true;
}

void DiagnosticsServer::update(const diag_data_t *data)
{
    if (!server_started || events.count() == 0)
        return;

    uint32_t now = millis();
    if (!_recording && (now - _last_send_ms < 100))
        return;
    _last_send_ms = now;

    char buf[256];
    snprintf(buf, sizeof(buf),
             "%lu,%s,%d,%d,%.1f,%.1f,%d,%u,%.1f,%.1f,%.1f,%.1f,%.1f,%d,%lu,%u",
             (unsigned long)data->timestamp_ms,
             data->mode,
             data->radio_connected,
             data->armed,
             data->a_percent,
             data->b_percent,
             data->button_state,
             data->flip_switch,
             data->left_cmd,
             data->right_cmd,
             data->accel_x,
             data->accel_y,
             data->accel_z,
             data->is_upside_down,
             (unsigned long)data->loop_us,
             data->wifi_clients);

    events.send(buf, NULL, now);
}
