// system_id_motor_core.ino
#include <Arduino.h>
#include <cmath>
#include <Wire.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>

// ============================================================
// Motor (pinos e PWM)
// ============================================================
#define PIN_PWM    13
#define PIN_DIR    14
#define PIN_BRAKE  17
const int PWM_Chan = 0;
const int PWM_Freq = 500;
const int PWM_Res  = 8;
const int PWM_MAX  = 255;

void releaseBrake() { digitalWrite(PIN_BRAKE, HIGH); }
void applyBrake()   { digitalWrite(PIN_BRAKE, LOW);  }

static inline void motorStopHold() {
  applyBrake();
  ledcWrite(PIN_PWM, PWM_MAX);
}

static inline void motorRunCW(uint8_t pwm_val) {
  digitalWrite(PIN_DIR, LOW);
  releaseBrake();
  ledcWrite(PIN_PWM, pwm_val);
}

static inline void motorRunCCW(uint8_t pwm_val) {
  digitalWrite(PIN_DIR, HIGH);
  releaseBrake();
  ledcWrite(PIN_PWM, pwm_val);
}

// ============================================================
// Encoder
// ============================================================
#define ENC1_1   26
#define ENC1_2   4
#define PPR      400

volatile long enc_count = 0;
volatile uint8_t encoder_state = 0;
static unsigned long last_rpm_ms = 0;
static long last_enc_count = 0;
static float wheel_rpm = 0.0f;
static const unsigned long RPM_PERIOD_MS = 100;   // <-- período de atualização do RPM: 100 ms
static const float RPM_EMA_ALPHA = 0.3f;

// Degrau
const int PWM_STEP = 0;
const unsigned long STEP_DELAY_MS = 5000;
bool step_applied = false;

// ENCODER ISR
void IRAM_ATTR encoder_ISR() {
  const uint8_t a = digitalRead(ENC1_1);
  const uint8_t b = digitalRead(ENC1_2);
  const uint8_t ab = (a << 1) | b;
  encoder_state = ((encoder_state << 2) | ab) & 0x0F;
  if      (encoder_state == 0x02 || encoder_state == 0x0D ||
           encoder_state == 0x04 || encoder_state == 0x0B) enc_count++;
  else if (encoder_state == 0x01 || encoder_state == 0x0E ||
           encoder_state == 0x08 || encoder_state == 0x07) enc_count--;
}

// CÁLCULO DE RPM — dispara a cada RPM_PERIOD_MS (100 ms)
static void updateRpm() {
  const unsigned long now = millis();
  if (now - last_rpm_ms < RPM_PERIOD_MS) return;
  last_rpm_ms = now;

  long count_now;
  noInterrupts();
  count_now = enc_count;
  interrupts();

  const long dcount = count_now - last_enc_count;
  last_enc_count = count_now;

  // dt em minutos para obter RPM
  const float dt_min = (float)RPM_PERIOD_MS / 60000.0f;
  const float rpm_inst = (dcount / (float)PPR) / dt_min;
  wheel_rpm = RPM_EMA_ALPHA * rpm_inst + (1.0f - RPM_EMA_ALPHA) * wheel_rpm;
}

// ============================================================
// Wi-Fi
// ============================================================
#define WIFI_SSID     "Apt102"
#define WIFI_PASSWORD "Gs120402@@"

void networkConnect() {
  btStop();
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000) {
    delay(200);
  }
}

// ============================================================
// Coleta de Dados
// ============================================================
#define MAX_SAMPLES 1000

struct DataPoint {
  unsigned long t_ms;    // tempo em ms desde início da coleta
  float pwm_cmd;
  float rpm_meas;
};

DataPoint dataBuffer[MAX_SAMPLES];
volatile uint16_t dataIndex = 0;
volatile bool isCollecting = false;
uint32_t collectionStartTime = 0;
static unsigned long lastRecordMs = 0;  // controla passo de 100 ms

void startDataCollection() {
  dataIndex = 0;
  isCollecting = true;
  collectionStartTime = millis();
  lastRecordMs = 0;
}

void stopDataCollection() {
  isCollecting = false;
  motorStopHold();
}

// Grava uma amostra a cada 100 ms exatos
void recordDataPoint(float pwm, float rpm) {
  if (!isCollecting) return;
  if (dataIndex >= MAX_SAMPLES) {
    stopDataCollection();
    return;
  }

  unsigned long elapsed_ms = millis() - collectionStartTime;

  // Passo fixo de 100 ms
  if (elapsed_ms - lastRecordMs < 100UL) return;
  lastRecordMs += 100UL;  // acumula em múltiplos exatos de 100 ms (evita deriva)

  dataBuffer[dataIndex].t_ms     = elapsed_ms;   // <-- tempo em ms
  dataBuffer[dataIndex].pwm_cmd  = pwm;
  dataBuffer[dataIndex].rpm_meas = rpm;
  dataIndex++;
}

// ============================================================
// Servidor Web e WebSocket
// ============================================================
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

String pendingCommand    = "";
bool   hasPendingCommand = false;

const char DASHBOARD_HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html lang="pt-BR">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Coleta de Dados — Motor</title>
<style>
body{font-family:Segoe UI,sans-serif;background:#0f0f1a;color:#e0e0e0;padding:20px}
h1{color:#7eb8f7;text-align:center}
#ws-dot{width:12px;height:12px;border-radius:50%;background:#ff4444;display:inline-block;margin-right:8px}
#ws-dot.ok{background:#44ff88}
.card{background:#1a1a2e;padding:14px;border-radius:12px;border:1px solid #2a2a4a;text-align:center}
.label{font-size:12px;color:#888;margin-bottom:4px}
.value{font-size:28px;font-weight:bold;color:#7eb8f7}
.unit{font-size:11px;color:#555}
.log-box{background:#0d0d1a;border-radius:8px;padding:8px;height:120px;overflow:auto;font-family:monospace;color:#66ff99}
canvas#dataChart{width:100%;height:300px}
button{padding:10px 18px;border-radius:8px;border:none;cursor:pointer;font-size:14px}
#btn-start{background:#44ff88;color:#0f0f1a}
#btn-stop{background:#ff4444;color:#fff}
#btn-export{background:#a855f7;color:#fff}
input[type=number]{background:#1a1a2e;color:#e0e0e0;border:1px solid #2a2a4a;border-radius:6px;padding:6px;width:100px;text-align:center}
input[type=range]{width:100%}
</style>
</head>
<body>
<h1>⚙️ Coleta de Dados — Motor</h1>
<div id="status-bar" style="text-align:center;margin-bottom:16px">
  <div id="ws-dot"></div><span id="ws-label">Desconectado</span>
</div>

<!-- Cards de telemetria -->
<div style="display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:12px;margin-bottom:18px">
  <div class="card"><div class="label">PWM Atual</div><div class="value" id="val-pwm">0</div><div class="unit">0–255</div></div>
  <div class="card"><div class="label">RPM Atual</div><div class="value" id="val-rpm">--</div><div class="unit">rpm</div></div>
  <!-- Tempo exibido em ms -->
  <div class="card"><div class="label">Tempo</div><div class="value" id="val-time">0</div><div class="unit">ms</div></div>
  <div class="card"><div class="label">Amostras</div><div class="value" id="val-samples">0</div><div class="unit">/ 1000</div></div>
</div>

<!-- Controle de PWM -->
<div style="background:#1a1a2e;border-radius:12px;padding:16px;margin-bottom:18px">
  <h2 style="color:#888">⚙️ Controle de PWM</h2>
  <div style="display:flex;align-items:center;gap:12px;justify-content:center;flex-wrap:wrap">
    <label style="color:#888">PWM (0–255):</label>
    <input type="number" id="pwm-input" min="0" max="255" step="1" value="0">
    <button onclick="applyPWM()" style="background:#7eb8f7;color:#0f0f1a;padding:10px 18px">Aplicar</button>
  </div>
  <div style="max-width:400px;margin:12px auto">
    <input type="range" id="pwm-slider" min="0" max="255" value="0" oninput="syncSlider()">
  </div>
</div>

<!-- Botões de controle -->
<div style="display:flex;gap:12px;justify-content:center;margin-bottom:18px;flex-wrap:wrap">
  <button id="btn-start" onclick="startCollection()">▶ INICIAR COLETA</button>
  <button id="btn-stop"  onclick="stopCollection()"  disabled>⏹ PARAR</button>
  <button id="btn-export" onclick="exportData()"     disabled>⬇ EXPORTAR CSV</button>
</div>

<!-- Gráfico -->
<div style="background:#1a1a2e;border-radius:12px;padding:12px;margin-bottom:18px">
  <h2 style="color:#888">PWM e RPM vs Tempo (ms)</h2>
  <canvas id="dataChart"></canvas>
</div>

<!-- Log -->
<div class="log-box" id="log"></div>

<script>
const wsUrl = `ws://${location.hostname}/ws`;
let sock;
// Armazena tempo em ms
let liveData = { time_ms: [], pwm: [], rpm: [] };
const MAX_CHART_POINTS = 500;

function connectWs() {
  sock = new WebSocket(wsUrl);
  sock.onopen = () => {
    document.getElementById('ws-dot').classList.add('ok');
    document.getElementById('ws-label').textContent = 'Conectado — ' + location.hostname;
    addLog('WebSocket conectado');
  };
  sock.onclose = () => {
    document.getElementById('ws-dot').classList.remove('ok');
    document.getElementById('ws-label').textContent = 'Desconectado';
    addLog('WebSocket desconectado');
    setTimeout(connectWs, 2000);
  };
  sock.onerror = () => addLog('Erro no WebSocket');
  sock.onmessage = (evt) => {
    try {
      const d = JSON.parse(evt.data);
      updateCards(d);
      updateChart(d);
    } catch(e) { addLog('JSON inválido'); }
  };
}

function send(cmd) {
  if (sock && sock.readyState === WebSocket.OPEN) sock.send(cmd);
  else addLog('WS não conectado');
}

function syncSlider() {
  document.getElementById('pwm-input').value = document.getElementById('pwm-slider').value;
}

function applyPWM() {
  const pwm = parseInt(document.getElementById('pwm-input').value);
  if (isNaN(pwm) || pwm < 0 || pwm > 255) { addLog('PWM deve estar entre 0 e 255'); return; }
  document.getElementById('pwm-slider').value = pwm;
  send('PWM=' + pwm);
  addLog('PWM aplicado: ' + pwm);
}

function startCollection() {
  send('START_COLLECT');
  document.getElementById('btn-start').disabled  = true;
  document.getElementById('btn-stop').disabled   = false;
  document.getElementById('btn-export').disabled = true;
  liveData = { time_ms: [], pwm: [], rpm: [] };
  addLog('Coleta iniciada');
}

function stopCollection() {
  send('STOP_COLLECT');
  document.getElementById('btn-start').disabled  = false;
  document.getElementById('btn-stop').disabled   = true;
  document.getElementById('btn-export').disabled = false;
  addLog('Coleta parada');
}

function exportData() {
  addLog('Iniciando download...');
  fetch('http://' + location.hostname + '/download_data')
    .then(r => { if (!r.ok) throw new Error('Erro ' + r.status); return r.blob(); })
    .then(blob => {
      const a = document.createElement('a');
      a.href = URL.createObjectURL(blob);
      a.download = 'motor_data.csv';
      document.body.appendChild(a);
      a.click();
      document.body.removeChild(a);
      URL.revokeObjectURL(a.href);
      addLog('Download concluído!');
    })
    .catch(e => addLog('Erro: ' + e.message));
}

// d.time_ms vem do firmware em ms
function updateCards(d) {
  if (d.pwm      !== undefined) document.getElementById('val-pwm').textContent     = d.pwm.toFixed(0);
  if (d.rpm      !== undefined) document.getElementById('val-rpm').textContent     = Math.abs(d.rpm).toFixed(1);
  if (d.time_ms  !== undefined) document.getElementById('val-time').textContent    = d.time_ms.toFixed(0);
  if (d.samples  !== undefined) document.getElementById('val-samples').textContent = d.samples;
}

function updateChart(d) {
  if (d.time_ms !== undefined && d.pwm !== undefined && d.rpm !== undefined) {
    liveData.time_ms.push(d.time_ms);
    liveData.pwm.push(d.pwm);
    liveData.rpm.push(Math.abs(d.rpm));
    if (liveData.time_ms.length > MAX_CHART_POINTS) {
      liveData.time_ms.shift(); liveData.pwm.shift(); liveData.rpm.shift();
    }
    drawChart();
  }
}

function drawChart() {
  const canvas = document.getElementById('dataChart');
  canvas.width  = canvas.offsetWidth;
  canvas.height = 300;
  const ctx = canvas.getContext('2d');
  const W = canvas.width, H = canvas.height;
  const pad = { top:20, right:20, bottom:50, left:60 };
  const cw = W - pad.left - pad.right;
  const ch = H - pad.top  - pad.bottom;
  ctx.clearRect(0, 0, W, H);
  if (liveData.time_ms.length < 2) return;

  const maxT   = Math.max(...liveData.time_ms) || 1;
  const maxRpm = Math.max(...liveData.rpm)      || 100;

  // Eixos
  ctx.strokeStyle = '#333'; ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(pad.left, pad.top); ctx.lineTo(pad.left, pad.top + ch);
  ctx.lineTo(pad.left + cw, pad.top + ch); ctx.stroke();

  // Label eixo X (ms)
  ctx.fillStyle = '#888'; ctx.font = '11px sans-serif'; ctx.textAlign = 'center';
  ctx.fillText('Tempo (ms)', pad.left + cw / 2, H - 6);

  // Ticks X
  for (let i = 0; i <= 5; i++) {
    const x = pad.left + (i / 5) * cw;
    ctx.fillStyle = '#555';
    ctx.fillText(Math.round((i / 5) * maxT), x, pad.top + ch + 16);
  }
  // Ticks Y RPM
  ctx.textAlign = 'right';
  for (let i = 0; i <= 4; i++) {
    const y = pad.top + ch - (i / 4) * ch;
    ctx.fillStyle = '#44ff88';
    ctx.fillText(Math.round((i / 4) * maxRpm), pad.left - 6, y + 4);
  }

  // Legenda
  ctx.textAlign = 'left';
  ctx.fillStyle = '#ff9944'; ctx.fillText('PWM', pad.left + cw - 80, pad.top + 14);
  ctx.fillStyle = '#44ff88'; ctx.fillText('RPM', pad.left + cw - 40, pad.top + 14);

  // Curva PWM
  ctx.beginPath(); ctx.strokeStyle = '#ff9944'; ctx.lineWidth = 2;
  liveData.time_ms.forEach((t, i) => {
    const x = pad.left + (t / maxT) * cw;
    const y = pad.top  + ch - (liveData.pwm[i] / 255) * ch;
    i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
  }); ctx.stroke();

  // Curva RPM
  ctx.beginPath(); ctx.strokeStyle = '#44ff88'; ctx.lineWidth = 2;
  liveData.time_ms.forEach((t, i) => {
    const x = pad.left + (t / maxT) * cw;
    const y = pad.top  + ch - (liveData.rpm[i] / maxRpm) * ch;
    i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
  }); ctx.stroke();
}

function addLog(msg) {
  const box = document.getElementById('log');
  const p = document.createElement('p');
  p.style.margin = '2px 0';
  p.textContent = `[${new Date().toLocaleTimeString()}] ${msg}`;
  box.appendChild(p);
  box.scrollTop = box.scrollHeight;
  if (box.children.length > 50) box.removeChild(box.firstChild);
}

connectWs();
</script>
</body>
</html>
)rawhtml";

void serverResponse() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", DASHBOARD_HTML);
  });

  // CSV com tempo em ms
  server.on("/download_data", HTTP_GET, [](AsyncWebServerRequest *request) {
    String csv = "t_ms,pwm,rpm\n";
    for (uint16_t i = 0; i < dataIndex; i++) {
      csv += String(dataBuffer[i].t_ms)        + ",";
      csv += String(dataBuffer[i].pwm_cmd,  0) + ",";
      csv += String(dataBuffer[i].rpm_meas, 2) + "\n";
    }
    AsyncWebServerResponse *resp = request->beginResponse(200, "text/csv", csv);
    resp->addHeader("Content-Disposition", "attachment; filename=\"motor_data.csv\"");
    request->send(resp);
  });
}

// ============================================================
// WebSocket handler
// ============================================================
void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    data[len] = 0;
    pendingCommand    = String((char*)data);
    hasPendingCommand = true;
  }
}

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_DATA) handleWebSocketMessage(arg, data, len);
}

// ============================================================
// Processamento de Comandos
// ============================================================
volatile uint8_t currentPwm = 0;

void processCommand(const String &line) {
  String s = line; s.trim();

  if (s.startsWith("PWM=")) {
    int pwm = s.substring(4).toInt();
    if (pwm >= 0 && pwm <= 255) {
      currentPwm = (uint8_t)pwm;
      motorRunCW(currentPwm);
    }
    return;
  }
  if (s == "START_COLLECT") { startDataCollection(); return; }
  if (s == "STOP_COLLECT")  { stopDataCollection();  return; }
}

// ============================================================
// Telemetria — envia time_ms em vez de time (segundos)
// ============================================================
void sendDataWs() {
  StaticJsonDocument<256> doc;
  doc["pwm"]     = (float)currentPwm;
  doc["rpm"]     = wheel_rpm;
  // tempo em ms desde o início da coleta (0 se não está coletando)
  doc["time_ms"] = isCollecting
                     ? (unsigned long)(millis() - collectionStartTime)
                     : 0UL;
  doc["samples"] = dataIndex;
  doc["ts"]      = millis();

  char buf[256];
  size_t len = serializeJson(doc, buf);
  ws.textAll(buf, len);
}

// ============================================================
// Setup
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(100);

  pinMode(PIN_DIR,   OUTPUT);
  pinMode(PIN_BRAKE, OUTPUT);
  ledcAttach(PIN_PWM, PWM_Freq, PWM_Res);
  motorStopHold();

  pinMode(ENC1_1, INPUT_PULLUP);
  pinMode(ENC1_2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC1_1), encoder_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC1_2), encoder_ISR, CHANGE);
  last_rpm_ms = millis();

  networkConnect();

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  serverResponse();
  server.begin();
}

// ============================================================
// Loop — cadência de 100 ms alinhada com o passo de gravação
// ============================================================
void loop() {
  static unsigned long last_loop_ms = 0;
  static const unsigned long LOOP_DT_MS = 10;   // tick geral: 10 ms

  unsigned long now_ms = millis();
  if (now_ms - last_loop_ms < LOOP_DT_MS) return;
  last_loop_ms = now_ms;

  // Atualiza RPM a cada 100 ms (controlado internamente por updateRpm)
  updateRpm();

  // Degrau automático após STEP_DELAY_MS
  if (!step_applied && now_ms >= STEP_DELAY_MS) {
    currentPwm = (uint8_t)PWM_STEP;
    ledcWrite(PIN_PWM, currentPwm);
    step_applied = true;
  }

  // Grava amostra a cada 100 ms
  if (isCollecting) recordDataPoint((float)currentPwm, wheel_rpm);

  // Envia WebSocket a cada 100 ms
  static unsigned long lastWsSend    = 0;
  static unsigned long lastWsCleanup = 0;

  if (now_ms - lastWsSend >= 100UL) {      // <-- 100 ms
    lastWsSend = now_ms;
    sendDataWs();
  }
  if (now_ms - lastWsCleanup >= 1000UL) {
    lastWsCleanup = now_ms;
    ws.cleanupClients();
  }

  if (hasPendingCommand) {
    processCommand(pendingCommand);
    hasPendingCommand = false;
    pendingCommand    = "";
  }
}
