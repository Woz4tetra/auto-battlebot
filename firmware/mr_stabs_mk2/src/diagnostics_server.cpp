#include "diagnostics_server.h"
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
</table>
<button class="btn rec" id="recBtn" onclick="toggleRec()">Record</button>
<button class="btn dl" onclick="downloadCSV()">Download CSV</button>
<span id="count"></span>
<script>
const hdr='timestamp_ms,radio_connected,armed,a_percent,b_percent,button_state,flip_switch,left_cmd,right_cmd,accel_x,accel_y,accel_z,is_upside_down,loop_us,wifi_clients';
const ids=['v_ts','v_radio','v_armed','v_a','v_b','v_btn','v_flip','v_left','v_right','v_ax','v_ay','v_az','v_usd','v_loop','v_wifi'];
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

void DiagnosticsServer::begin()
{
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send(200, "text/html", INDEX_HTML); });

    server.on("/record/start", HTTP_GET, [this](AsyncWebServerRequest *request)
              { _recording = true; request->send(200, "text/plain", "ok"); });

    server.on("/record/stop", HTTP_GET, [this](AsyncWebServerRequest *request)
              { _recording = false; request->send(200, "text/plain", "ok"); });

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
             "%lu,%d,%d,%.1f,%.1f,%d,%u,%.1f,%.1f,%.1f,%.1f,%.1f,%d,%lu,%u",
             (unsigned long)data->timestamp_ms,
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
