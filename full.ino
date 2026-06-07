#include <Arduino.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

// ── Pin definitions ──────────────────────────────────────
#define SERVO_PIN   18
#define FSR_PIN_A   34
#define FSR_PIN_B   35

// ── Config ───────────────────────────────────────────────
#define FSR_SAMPLES          10
#define ADC_REF              3.3f
#define ADC_MAX              4095.0f
#define R_FIXED              40000.0f
#define EMA_ALPHA            0.15f

#define OPEN_ANGLE           60
#define CLOSE_ANGLE          1
#define FAST_STEP_MS         15
#define SLOW_STEP_MS         40
#define FSR_CONTACT_THRESH   0.3f
#define FSR_HOLD_THRESH      0.6f
#define SIGNAL_LOSS_TIMEOUT  5000   // ms before failsafe open

const char* SSID     = "UW MPSK";
const char* PASSWORD = "rK4AnMkaQpj5uG6Y";

// ── State machine ─────────────────────────────────────────
enum GripState { IDLE, FAST_CLOSING, SLOW_CLOSING, HOLDING, OPENING };
GripState currentState = IDLE;

// ── Globals ──────────────────────────────────────────────
Servo gripper;
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
int currentAngle = OPEN_ANGLE;

float emaA = 0.0f;
float emaB = 0.0f;
bool  emaInit = false;

unsigned long lastClientSeen  = 0;
bool          wsEverConnected = false;

// ── WiFi ─────────────────────────────────────────────────
void connectWiFi() {
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASSWORD);
  Serial.print("[WiFi] Connecting to UW MPSK");
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 40) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.print("[WiFi] Connected! IP: http://");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\n[WiFi] FAILED — will retry in loop");
  }
}

// ── FSR helpers ──────────────────────────────────────────
float readVoltage(int pin) {
  long sum = 0;
  for (int i = 0; i < FSR_SAMPLES; i++) {
    sum += analogRead(pin);
    delayMicroseconds(200);
  }
  return (sum / (float)FSR_SAMPLES) / ADC_MAX * ADC_REF;
}

float voltageToResistance(float v) {
  if (v < 0.01f) return 999999.0f;
  return R_FIXED * (ADC_REF - v) / v;
}

float readMaxFSR(float &outVA, float &outVB) {
  float rawA = readVoltage(FSR_PIN_A);
  float rawB = readVoltage(FSR_PIN_B);

  if (!emaInit) {
    emaA = rawA;
    emaB = rawB;
    emaInit = true;
  }

  emaA = EMA_ALPHA * rawA + (1.0f - EMA_ALPHA) * emaA;
  emaB = EMA_ALPHA * rawB + (1.0f - EMA_ALPHA) * emaB;

  outVA = emaA;
  outVB = emaB;
  return max(emaA, emaB);
}

// ── State name helper ─────────────────────────────────────
const char* stateName(GripState s) {
  switch (s) {
    case IDLE:         return "IDLE";
    case FAST_CLOSING: return "FAST_CLOSING";
    case SLOW_CLOSING: return "SLOW_CLOSING";
    case HOLDING:      return "HOLDING";
    case OPENING:      return "OPENING";
    default:           return "UNKNOWN";
  }
}

void setState(GripState s) {
  Serial.print("[STATE] "); Serial.print(stateName(currentState));
  Serial.print(" → ");     Serial.println(stateName(s));
  currentState = s;
}

// ── Serial print ──────────────────────────────────────────
void printFSR(float vA, float vB, float vMax) {
  float rMax = voltageToResistance(vMax);
  Serial.print("Servo: "); Serial.print(currentAngle); Serial.print("°  |  ");
  Serial.print("FSR-A: "); Serial.print(vA, 3); Serial.print("V  ");
  Serial.print("FSR-B: "); Serial.print(vB, 3); Serial.print("V  |  ");
  Serial.print("MAX: ");   Serial.print(vMax, 3); Serial.print("V  ");
  if (rMax < 900000) { Serial.print(rMax / 1000, 1); Serial.print("kΩ"); }
  else Serial.print("no contact");
  Serial.print("  |  State: "); Serial.println(stateName(currentState));
}

// ── HTML ─────────────────────────────────────────────────
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>The Claw</title>
<style>
  body{font-family:monospace;background:#111;color:#eee;padding:20px}
  h2{color:#0cf}
  .card{background:#1e1e1e;border-radius:8px;padding:16px;margin:10px 0}
  label{display:block;margin-bottom:6px;font-weight:bold;color:#0cf}
  input[type=range]{width:100%;accent-color:#0cf}
  input[type=number]{width:80px;background:#333;color:#eee;border:1px solid #555;padding:4px;border-radius:4px}
  button{background:#0cf;color:#111;border:none;padding:8px 18px;border-radius:6px;cursor:pointer;font-weight:bold;margin-left:8px}
  button:active{opacity:0.7}
  .btn-close{background:#f40;color:#fff;margin-left:0;width:100%;padding:12px;font-size:1.1em;border-radius:6px;cursor:pointer;font-weight:bold;border:none}
  .btn-open {background:#0a0;color:#fff;margin-left:0;width:100%;padding:12px;font-size:1.1em;border-radius:6px;cursor:pointer;font-weight:bold;border:none;margin-top:8px}
  .btn-stop {background:#555;color:#eee;width:100%;padding:10px;font-size:1em;border-radius:6px;cursor:pointer;font-weight:bold;border:none;margin-top:8px}
  .stat{font-size:1.1em;margin:6px 0}
  .bar-wrap{background:#333;border-radius:4px;height:14px;margin:4px 0;position:relative}
  .bar{height:14px;border-radius:4px;transition:width 0.1s}
  .thresh-line{position:absolute;top:0;height:14px;width:2px;background:#ff0;opacity:0.8}
  .thresh-line2{position:absolute;top:0;height:14px;width:2px;background:#f40;opacity:0.8}
  .sub{font-size:0.8em;color:#888;margin-top:8px}
  #status{color:#f80;font-size:0.85em;margin-top:6px}
  #stateDisplay{font-size:1.2em;font-weight:bold;margin-top:6px;padding:8px;border-radius:6px;text-align:center}
</style>
</head><body>
<h2>🦾 The Claw — Control Panel</h2>
<div id="status">⬤ Connecting...</div>

<div class="card">
  <label>System State</label>
  <div id="stateDisplay" style="background:#333;color:#fff">--</div>
</div>

<div class="card">
  <label>Auto Control</label>
  <button class="btn-close" onclick="send({autoClose:true})">⚡ Close (Auto)</button>
  <button class="btn-open"  onclick="send({open:true})">↑ Open</button>
  <button class="btn-stop"  onclick="send({stopAuto:true})">■ Stop</button>
</div>

<div class="card">
  <label>Manual Servo Angle</label>
  <input type="range" id="slider" min="1" max="60" value="60"
         oninput="sliderMove(this.value)">
  <div style="margin-top:8px">
    <input type="number" id="angleBox" min="1" max="60" value="60">
    <button onclick="sendAngle()">Go</button>
    <span id="angleDisplay" style="margin-left:12px;color:#0cf">60°</span>
  </div>
</div>

<div class="card">
  <label>FSR — max contact</label>
  <div class="stat" id="fsr_v">-- V</div>
  <div class="stat" id="fsr_r">--</div>
  <div class="bar-wrap">
    <div class="bar" id="barMax" style="width:0%;background:#0cf"></div>
    <div class="thresh-line"  id="contactLine"></div>
    <div class="thresh-line2" id="holdLine"></div>
  </div>
  <div class="sub">
    <span style="color:#ff0">▏contact @ 0.2V</span> &nbsp;
    <span style="color:#f40">▏hold @ 0.6V</span>
  </div>
  <div class="sub">A: <span id="fsrA_v">--</span> &nbsp; B: <span id="fsrB_v">--</span></div>
</div>

<script>
  let ws, reconnectTimer;
  const V_MIN = 0.05, V_MAX = 2.8;

  function pct(v) { return Math.min(Math.max((v - V_MIN) / (V_MAX - V_MIN) * 100, 0), 100); }
  document.getElementById('contactLine').style.left = pct(0.2) + '%';
  document.getElementById('holdLine').style.left    = pct(0.6) + '%';

  const stateColors = {
    IDLE:         {bg:'#333',  color:'#eee'},
    FAST_CLOSING: {bg:'#f40',  color:'#fff'},
    SLOW_CLOSING: {bg:'#fa0',  color:'#111'},
    HOLDING:      {bg:'#0a0',  color:'#fff'},
    OPENING:      {bg:'#09f',  color:'#111'},
  };

  function connect() {
    clearTimeout(reconnectTimer);
    ws = new WebSocket('ws://' + location.hostname + ':80/ws');

    ws.onopen = () => {
      document.getElementById('status').textContent = '⬤ Connected';
      document.getElementById('status').style.color = '#0f0';
    };
    ws.onclose = () => {
      document.getElementById('status').textContent = '⬤ Disconnected — retrying...';
      document.getElementById('status').style.color = '#f80';
      reconnectTimer = setTimeout(connect, 2000);
    };
    ws.onerror = () => ws.close();

    ws.onmessage = e => {
      const d = JSON.parse(e.data);

      const sc = stateColors[d.state] || {bg:'#333', color:'#eee'};
      const sd = document.getElementById('stateDisplay');
      sd.textContent = d.state;
      sd.style.background = sc.bg;
      sd.style.color = sc.color;

      document.getElementById('slider').value = d.angle;
      document.getElementById('angleBox').value = d.angle;
      document.getElementById('angleDisplay').textContent = d.angle + '°';

      document.getElementById('fsr_v').textContent = d.vMax.toFixed(3) + ' V';
      document.getElementById('fsr_r').textContent = d.rMax < 900000
        ? (d.rMax / 1000).toFixed(1) + ' kΩ' : 'no contact';
      document.getElementById('barMax').style.width = pct(d.vMax) + '%';
      document.getElementById('fsrA_v').textContent = d.vA.toFixed(3) + ' V';
      document.getElementById('fsrB_v').textContent = d.vB.toFixed(3) + ' V';
    };
  }

  connect();

  function send(obj) {
    if (ws && ws.readyState === WebSocket.OPEN)
      ws.send(JSON.stringify(obj));
  }

  function sliderMove(v) {
    document.getElementById('angleBox').value = v;
    document.getElementById('angleDisplay').textContent = v + '°';
    send({angle: parseInt(v)});
  }

  function sendAngle() {
    const v = parseInt(document.getElementById('angleBox').value);
    document.getElementById('slider').value = v;
    document.getElementById('angleDisplay').textContent = v + '°';
    send({angle: v});
  }
</script>
</body></html>
)rawliteral";

// ── WebSocket event ───────────────────────────────────────
void onWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
               AwsEventType type, void* arg, uint8_t* data, size_t len) {

  if (type == WS_EVT_CONNECT) {
    lastClientSeen = millis();
    wsEverConnected = true;
    Serial.println("[WS] Client connected");
  }

  if (type == WS_EVT_DISCONNECT) {
    Serial.println("[WS] Client disconnected");
  }

  if (type == WS_EVT_DATA) {
    lastClientSeen = millis();
    String msg = "";
    for (size_t i = 0; i < len; i++) msg += (char)data[i];
    Serial.print("[WS RX] "); Serial.println(msg);

    if (msg.indexOf("\"angle\"") != -1) {
      int colon = msg.indexOf(':', msg.indexOf("\"angle\""));
      if (colon != -1) {
        int angle = msg.substring(colon + 1).toInt();
        angle = constrain(angle, 1, 60);
        setState(IDLE);
        gripper.write(angle);
        currentAngle = angle;
      }
    }

    if (msg.indexOf("\"autoClose\"") != -1) {
      if (currentState == IDLE) setState(FAST_CLOSING);
    }

    if (msg.indexOf("\"open\"") != -1) {
      setState(OPENING);
    }

    if (msg.indexOf("\"stopAuto\"") != -1) {
      if (currentState != IDLE && currentState != HOLDING) setState(IDLE);
    }
  }
}

// ── Setup ─────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);

  gripper.attach(SERVO_PIN);
  gripper.write(currentAngle);
  Serial.println("[SERVO] Initialized at OPEN");

  connectWiFi();

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
    req->send_P(200, "text/html", INDEX_HTML);
  });
  server.begin();
  Serial.println("[HTTP] Server started");
}

// ── Loop ──────────────────────────────────────────────────
void loop() {

  // WiFi watchdog
  static unsigned long lastWiFiCheck = 0;
  if (millis() - lastWiFiCheck >= 5000) {
    lastWiFiCheck = millis();
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("[WiFi] Lost — reconnecting...");
      WiFi.disconnect();
      WiFi.begin(SSID, PASSWORD);
    }
  }

  // Read FSR every loop iteration (EMA needs frequent updates)
  float vA, vB;
  float vMax = readMaxFSR(vA, vB);
  float rMax = voltageToResistance(vMax);

  // ── Signal loss failsafe ────────────────────────────────
  if (wsEverConnected && ws.count() == 0) {
    if (currentState != IDLE && currentState != OPENING &&
        millis() - lastClientSeen >= SIGNAL_LOSS_TIMEOUT) {
      Serial.println("[FAILSAFE] Signal lost — opening");
      setState(OPENING);
    }
  } else if (ws.count() > 0) {
    lastClientSeen = millis();
  }

  // ── State machine ───────────────────────────────────────
  static unsigned long lastStep = 0;

  switch (currentState) {

    case IDLE:
      break;

    case FAST_CLOSING: {
      if (millis() - lastStep >= FAST_STEP_MS) {
        lastStep = millis();
        if (vMax >= FSR_CONTACT_THRESH) {
          setState(SLOW_CLOSING);
        } else if (currentAngle <= CLOSE_ANGLE) {
          setState(HOLDING);
        } else {
          currentAngle--;
          gripper.write(currentAngle);
        }
      }
      break;
    }

    case SLOW_CLOSING: {
      if (millis() - lastStep >= SLOW_STEP_MS) {
        lastStep = millis();
        if (vMax >= FSR_HOLD_THRESH) {
          setState(HOLDING);
        } else if (currentAngle <= CLOSE_ANGLE) {
          setState(HOLDING);
        } else if (vMax < FSR_CONTACT_THRESH) {
          setState(FAST_CLOSING);
        } else {
          currentAngle--;
          gripper.write(currentAngle);
        }
      }
      break;
    }

    case HOLDING:{
      static unsigned long belowSince = 0;
      if (vMax < FSR_HOLD_THRESH) {
        if (belowSince == 0) belowSince = millis();          // start the clock
        if (millis() - belowSince >= 1000) {                 // 1s continuously below
          belowSince = 0;
          setState(SLOW_CLOSING);
        }
      } else {
        belowSince = 0;                                       // recovered — reset clock
      }
      break;
    }

    case OPENING: {
      if (millis() - lastStep >= FAST_STEP_MS) {
        lastStep = millis();
        if (currentAngle >= OPEN_ANGLE) {
          setState(IDLE);
        } else {
          currentAngle++;
          gripper.write(currentAngle);
        }
      }
      break;
    }
  }

  // ── Broadcast at 10 Hz ──────────────────────────────────
  static unsigned long lastBroadcast = 0;
  if (millis() - lastBroadcast >= 100) {
    lastBroadcast = millis();

    printFSR(vA, vB, vMax);

    if (ws.count() > 0) {
      String json = "{\"state\":\""  + String(stateName(currentState)) + "\"" +
                    ",\"angle\":"    + String(currentAngle) +
                    ",\"vA\":"       + String(vA,   3) +
                    ",\"vB\":"       + String(vB,   3) +
                    ",\"vMax\":"     + String(vMax, 3) +
                    ",\"rMax\":"     + String(rMax, 1) + "}";
      ws.textAll(json);
    }
  }

  ws.cleanupClients();
}