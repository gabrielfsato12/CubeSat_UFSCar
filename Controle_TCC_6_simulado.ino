#include <Arduino.h>
#include <cmath>
#include <Wire.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>

// ============================================================
// MPU9250 — Registradores e constantes
// ============================================================
#define MPU9250_ADDR     0x68
#define REG_PWR_MGMT_1   0x6B
#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_CONFIG       0x1A
#define REG_SMPLRT_DIV   0x19
#define REG_GYRO_ZOUT_H  0x47

#define I2C_SDA 22
#define I2C_SCL 21

// ============================================================
// Parâmetros de controle — ajustáveis em tempo real
// ============================================================
float ctrl_Kp         = 1.968f;
float ctrl_Ki         = 0.04614f;
float ctrl_Kd         = 1.045f;
float ctrl_limiarStop = 2.0f;
float ctrl_gyroTh     = 0.5f;
float ctrl_slewRate   = 20.0f;   // máx variação de U por ciclo (10 ms)

float integral_error  = 0.0f;
float u_prev          = 0.0f;    // para o slew rate

float setpoint_deg = 90.0f;
float yaw_deg      = 0.0f;

// ============================================================
// MPU9250
// ============================================================
// ±500 °/s → sensitivity = 65.5 LSB/(°/s)
static const float GYRO_SCALE = 1.0f / 65.5f;
static float gyroZ_offset = 0.0f;

void mpu9250_writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t mpu9250_readReg(uint8_t reg) {
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU9250_ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0;
}

int16_t mpu9250_readInt16(uint8_t regHigh) {
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(regHigh);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU9250_ADDR, (uint8_t)2);
  if (Wire.available() < 2) return 0;
  return (int16_t)(Wire.read() << 8) | Wire.read();
}

float mpu9250_getGyroZ() {
  int16_t raw = mpu9250_readInt16(REG_GYRO_ZOUT_H);
  return -((raw * GYRO_SCALE) - gyroZ_offset);
}

void mpu9250_calibrateGyro(int samples = 500) {
  double sum = 0.0;
  for (int i = 0; i < samples; i++) {
    int16_t raw = mpu9250_readInt16(REG_GYRO_ZOUT_H);
    sum += raw * GYRO_SCALE;
    delay(5);
  }
  gyroZ_offset = (float)(sum / samples);
}

void mpu9250_init() {
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  mpu9250_writeReg(REG_PWR_MGMT_1, 0x00);
  delay(100);
  mpu9250_writeReg(REG_PWR_MGMT_1, 0x01);
  delay(20);
  mpu9250_writeReg(REG_GYRO_CONFIG, 0x08); // ±500 °/s
  mpu9250_writeReg(REG_CONFIG,      0x04); // DLPF 20 Hz
  mpu9250_writeReg(REG_SMPLRT_DIV,  0x09); // 100 Hz
  delay(100);
  mpu9250_calibrateGyro(500);
}

// ============================================================
// Wi-Fi
// ============================================================
float yaw_integrated = 0.0f;

String pendingCommand    = "";
bool   hasPendingCommand = false;

#define WIFI_SSID     "Apt102"
#define WIFI_PASSWORD "Gs120402@@"

void networkConnect() {
  btStop();
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000) {
    delay(500);
  }
}

// ============================================================
// Página HTML
// ============================================================
// HTML principal com link para o JavaScript
const char DASHBOARD_HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html lang="pt-BR">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>CubeSat Dashboard</title>
<style>
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body {
    font-family: 'Segoe UI', sans-serif;
    background: #0f0f1a;
    color: #e0e0e0;
    min-height: 100vh;
    padding: 20px;
  }
  h1 {
    text-align: center;
    font-size: 1.6rem;
    color: #7eb8f7;
    margin-bottom: 20px;
    letter-spacing: 2px;
    text-transform: uppercase;
  }
  #status-bar {
    display: flex; align-items: center;
    justify-content: center; gap: 10px;
    margin-bottom: 24px;
  }
  #ws-dot {
    width: 12px; height: 12px;
    border-radius: 50%; background: #ff4444;
    transition: background 0.3s;
  }
  #ws-dot.ok { background: #44ff88; }
  #ws-label { font-size: 0.85rem; color: #888; }
  .grid {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(150px, 1fr));
    gap: 16px;
    margin-bottom: 28px;
  }
  .card {
    background: #1a1a2e;
    border: 1px solid #2a2a4a;
    border-radius: 12px;
    padding: 18px 14px;
    text-align: center;
    transition: border-color 0.3s;
  }
  .card:hover { border-color: #7eb8f7; }
  .card .label {
    font-size: 0.72rem; text-transform: uppercase;
    letter-spacing: 1px; color: #888; margin-bottom: 8px;
  }
  .card .value { font-size: 2rem; font-weight: 700; color: #7eb8f7; }
  .card .unit  { font-size: 0.75rem; color: #666; margin-top: 4px; }
  .card.status-ok   .value { color: #44ff88; }
  .card.status-run  .value { color: #ffcc44; }
  .card.mode-coarse .value { color: #ff9944; }
  .card.dir-cw   .value { color: #44ff88; }
  .card.dir-ccw  .value { color: #ff9944; }
  .card.dir-stop .value { color: #888; }
 .gauge-wrapper {
  display: flex; justify-content: center; margin-bottom: 28px;
}
canvas#yawGauge { display: block; }
.charts-container {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 16px;
  margin-bottom: 28px;
}
.chart-box {
  background: #1a1a2e; border: 1px solid #2a2a4a;
  border-radius: 12px; padding: 16px;
}
.chart-box h2 {
  font-size: 0.8rem; text-transform: uppercase;
  letter-spacing: 1px; color: #888; margin-bottom: 12px;
}
canvas#lineChart { width: 100% !important; height: 180px !important; }
canvas#rpmChart  { width: 100% !important; height: 180px !important; }
  .controls {
    display: flex; flex-wrap: wrap; gap: 12px;
    justify-content: center; margin-bottom: 28px;
  }
  button {
    padding: 12px 24px; border: none; border-radius: 8px;
    font-size: 0.9rem; font-weight: 600; cursor: pointer;
    letter-spacing: 1px; transition: opacity 0.2s, transform 0.1s;
  }
  button:active { transform: scale(0.96); }
  button:hover  { opacity: 0.85; }
  #btn-start { background: #44ff88; color: #0f0f1a; }
  #btn-stop  { background: #ff4444; color: #fff; }
  #btn-reset { background: #7eb8f7; color: #0f0f1a; }
  #btn-download-log { background: #55aaff; color: #fff; } /* Novo estilo para o botão de download */
  .setpoint-row {
    display: flex; align-items: center; gap: 10px;
    justify-content: center; margin-bottom: 28px; flex-wrap: wrap;
  }
  .setpoint-row label { font-size: 0.85rem; color: #888; }
  .setpoint-row input[type=number] {
    width: 90px; padding: 10px; background: #1a1a2e;
    border: 1px solid #2a2a4a; border-radius: 8px;
    color: #e0e0e0; font-size: 1rem; text-align: center;
  }
  .setpoint-row button { background: #a855f7; color: #fff; padding: 10px 20px; }
  .params-box {
    background: #1a1a2e; border: 1px solid #2a2a4a;
    border-radius: 12px; padding: 20px; margin-bottom: 28px;
  }
  .params-box h2 {
    font-size: 0.8rem; text-transform: uppercase;
    letter-spacing: 1px; color: #888; margin-bottom: 16px;
  }
  .params-grid {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
    gap: 14px; margin-bottom: 16px;
  }
  .param-item { display: flex; flex-direction: column; gap: 6px; }
  .param-item label {
    font-size: 0.75rem; text-transform: uppercase;
    letter-spacing: 1px; color: #888;
  }
  .param-item input[type=number] {
    padding: 9px 12px; background: #0f0f1a;
    border: 1px solid #2a2a4a; border-radius: 8px;
    color: #e0e0e0; font-size: 0.95rem; transition: border-color 0.2s;
  }
  .param-item input[type=number]:focus { outline: none; border-color: #7eb8f7; }
  .param-item .current-val { font-size: 0.7rem; color: #555; }
  .params-box .send-btn {
    background: #ff9944; color: #0f0f1a;
    padding: 10px 28px; border-radius: 8px;
    font-weight: 700; font-size: 0.9rem;
  }
  .log-box {
    background: #1a1a2e; border: 1px solid #2a2a4a;
    border-radius: 12px; padding: 16px; margin-bottom: 28px;
    max-height: 160px; overflow-y: auto;
  }
  .log-box h2 {
    font-size: 0.8rem; text-transform: uppercase;
    letter-spacing: 1px; color: #888; margin-bottom: 10px;
  }
  #log p { font-size: 0.78rem; color: #666; margin-bottom: 4px; font-family: monospace; }

  /* Estilos para a nova seção de Identificação */
  .identification-box {
    background: #1a1a2e; border: 1px solid #2a2a4a;
    border-radius: 12px; padding: 20px; margin-bottom: 28px;
  }
  .identification-box h2 {
    font-size: 0.8rem; text-transform: uppercase;
    letter-spacing: 1px; color: #888; margin-bottom: 16px;
  }
  .identification-row {
    display: flex; align-items: center; gap: 10px;
    justify-content: center; margin-bottom: 16px; flex-wrap: wrap;
  }
  .identification-row label { font-size: 0.85rem; color: #888; }
  .identification-row input[type=number] {
    width: 90px; padding: 10px; background: #0f0f1a;
    border: 1px solid #2a2a4a; border-radius: 8px;
    color: #e0e0e0; font-size: 1rem; text-align: center;
  }
  .identification-row button { background: #f7a855; color: #0f0f1a; padding: 10px 20px; }
  #btn-stop-id { background: #ff4444; color: #fff; }
  #btn-download-id-log { background: #55aaff; color: #fff; }
</style>
</head>
<body>
<h1 style="margin-bottom: 10px;">CubeSat · ADCS Dashboard</h1>
<div id="status-bar">
  <div id="ws-dot"></div>
  <span id="ws-label">Desconectado</span>
</div>

<div class="grid">
  <div class="card" id="card-yaw">
    <div class="label">Yaw</div>
    <div class="value" id="cv-yaw">—</div>
    <div class="unit">graus</div>
  </div>
  <div class="card" id="card-sp">
    <div class="label">Setpoint</div>
    <div class="value" id="cv-sp">—</div>
    <div class="unit">graus</div>
  </div>
  <div class="card" id="card-err">
    <div class="label">Erro</div>
    <div class="value" id="cv-err">—</div>
    <div class="unit">graus</div>
  </div>
  <div class="card" id="card-gyro">
    <div class="label">Gyro Z</div>
    <div class="value" id="cv-gyro">—</div>
    <div class="unit">°/s</div>
  </div>
  <div class="card" id="card-rpm">
    <div class="label">RPM</div>
    <div class="value" id="cv-rpm">—</div>
    <div class="unit">rpm</div>
  </div>
</div>

<div class="gauge-wrapper">
  <canvas id="yawGauge" width="240" height="240"></canvas>
</div>

<div class="charts-container">
  <div class="chart-box">
    <h2>Yaw vs Setpoint</h2>
    <canvas id="lineChart"></canvas>
  </div>
  <div class="chart-box">
    <h2>RPM Roda de Reação</h2>
    <canvas id="rpmChart"></canvas>
  </div>
</div>

<div class="controls">
  <button id="btn-start">START PID</button>
  <button id="btn-stop">STOP</button>
  <button id="btn-reset">RESET IMU</button>
  <button id="btn-download-log">Download Log PID</button>
</div>

<div class="setpoint-row">
  <label>Setpoint (°)</label>
  <input type="number" id="sp-input" min="0" max="360" step="1" value="90">
  <button onclick="sendSP()">Enviar SP</button>
</div>

<div class="params-box">
  <h2>Parâmetros PID</h2>
  <div class="params-grid">
    <div class="param-item">
      <label>Kp</label>
      <input type="number" id="p-kp" step="0.001" value="1.553">
      <span class="current-val" id="cv-kp">atual: —</span>
    </div>
    <div class="param-item">
      <label>Ki</label>
      <input type="number" id="p-ki" step="0.001" value="0.02744">
      <span class="current-val" id="cv-ki">atual: —</span>
    </div>
    <div class="param-item">
      <label>Kd</label>
      <input type="number" id="p-kd" step="0.001" value="0.6901">
      <span class="current-val" id="cv-kd">atual: —</span>
    </div>
    <div class="param-item">
      <label>Limiar Stop (°)</label>
      <input type="number" id="p-lstop" step="0.1" value="5.0">
      <span class="current-val" id="cv-lstop">atual: —</span>
    </div>
  </div>
  <button class="send-btn" onclick="sendParams()">Enviar Parâmetros</button>
</div>

<!-- Nova seção para Identificação de Sistema -->
<div class="identification-box">
  <h2>Identificação de Sistema (Malha Aberta)</h2>
  <div class="identification-row">
    <label>PWM (0-255, 0=max, 255=stop)</label>
    <input type="number" id="id-pwm-input" min="0" max="255" step="1" value="246">
    <label>Duração (ms)</label>
    <input type="number" id="id-duration-input" min="1000" step="1000" value="10000">
  </div>
  <div class="controls">
    <button id="btn-start-id">Iniciar Identificação</button>
    <button id="btn-stop-id">Parar Identificação</button>
    <button id="btn-download-id-log">Download Log Identificação</button>
  </div>
</div>

<div class="log-box">
  <h2>Log de Eventos</h2>
  <div id="log"></div>
</div>

<script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
<script src="/script.js"></script> <!-- Carrega o JavaScript de um arquivo separado -->
</body>
</html>
)rawhtml";

// ============================================================
// JavaScript para o Dashboard
// ============================================================
// Este bloco de código JavaScript será servido como um arquivo separado.
const char DASHBOARD_JS[] PROGMEM = R"rawliteral(
let ws;
let logData = []; // Array para armazenar os dados do log do PID
let idLogData = []; // Array para armazenar os dados do log de identificação
let logStartTime = 0; // Tempo de início da manobra para calcular o 't'
let idLogStartTime = 0; // Tempo de início do teste de identificação

function connectWs() {
  ws = new WebSocket('ws://' + location.hostname + '/ws');
  ws.onopen = () => {
    document.getElementById('ws-dot').classList.add('ok');
    document.getElementById('ws-label').textContent = 'Conectado';
    addLog('WebSocket conectado');
    logData = []; // Limpa o log ao conectar
    idLogData = []; // Limpa o log de identificação
    logStartTime = 0; // Reseta o tempo de início
    idLogStartTime = 0; // Reseta o tempo de início da identificação
  };
  ws.onclose = () => {
    document.getElementById('ws-dot').classList.remove('ok');
    document.getElementById('ws-label').textContent = 'Desconectado';
    addLog('WebSocket desconectado — reconectando...');
    setTimeout(connectWs, 2000);
  };
  ws.onerror = () => ws.close();
  ws.onmessage = (evt) => {
  let d;
  try { d = JSON.parse(evt.data); } catch(e) { return; }

  // --- Lógica para o log de controle PID ---
  // d.st == 1 (RUN)
  if (d.st === 1 && logStartTime === 0) {
    logStartTime = d.ts; // Usa o timestamp do ESP32 em millis
    addLog('Manobra PID iniciada. Iniciando log de dados de controle.');
  } else if (d.st === 0 && logStartTime !== 0) { // IDLE
    addLog('Manobra PID finalizada. Log de dados de controle parado.');
    logStartTime = 0; // Reseta o tempo de início
  }

  // Armazena os dados no array de log se a manobra PID estiver em andamento
  if (d.st === 1 && logStartTime !== 0) {
    const currentTime = d.ts - logStartTime; // Tempo em milissegundos
    logData.push({
      t_ms: currentTime,        // Tempo em milissegundos
      yaw: d.y,
      setpoint: d.sp,
      erro: d.e,
      gyroZ: d.g,
      rpm: d.r,
      // Os campos abaixo foram removidos do dashboard, mas mantidos no log para debug se necessário
      u_saida: d.u,
      u_raw: d.ur,
      integral: d.i,
      estado: d.st,
      modo: d.m,
      direcao: d.dir,
      hz: d.hz
    });
  }

  // --- Lógica para o log de Identificação ---
  // d.st == 2 (IDENTIFICATION)
  if (d.st === 2 && idLogStartTime === 0) {
    idLogStartTime = d.ts; // Usa o timestamp do ESP32 em millis
    addLog('Teste de Identificação iniciado. Iniciando log de dados de identificação.');
  } else if (d.st !== 2 && idLogStartTime !== 0) { // Saiu do estado de identificação
    addLog('Teste de Identificação finalizado. Log de dados de identificação parado.');
    idLogStartTime = 0; // Reseta o tempo de início da identificação
  }

  // Armazena os dados no array de log se o teste de identificação estiver em andamento
  if (d.st === 2 && idLogStartTime !== 0) {
    const currentTime = d.ts - idLogStartTime; // Tempo em milissegundos
    idLogData.push({
      t_ms: currentTime,        // Tempo em milissegundos
      pwm_aplicado: d.id_pwm,   // PWM aplicado no modo de identificação
      yaw: d.y,                 // Ângulo de Yaw
      gyroZ: d.g,               // Velocidade angular Z
      rpm: d.r                  // RPM da roda
    });
  }

    document.getElementById('cv-yaw').textContent   = d.y   !== undefined ? d.y.toFixed(1)  : '—';
    document.getElementById('cv-sp').textContent    = d.sp  !== undefined ? d.sp.toFixed(1) : '—';
    document.getElementById('cv-err').textContent   = d.e   !== undefined ? d.e.toFixed(1)  : '—';
    document.getElementById('cv-gyro').textContent  = d.g   !== undefined ? d.g.toFixed(2)  : '—';
    document.getElementById('cv-rpm').textContent   = d.r   !== undefined ? d.r.toFixed(0)  : '—';

    if (d.kp    !== undefined) { document.getElementById('cv-kp').textContent    = 'atual: ' + d.kp.toFixed(4);    }
    if (d.ki    !== undefined) { document.getElementById('cv-ki').textContent    = 'atual: ' + d.ki.toFixed(5);    }
    if (d.kd    !== undefined) { document.getElementById('cv-kd').textContent    = 'atual: ' + d.kd.toFixed(4);    }
    if (d.lstop !== undefined) { document.getElementById('cv-lstop').textContent = 'atual: ' + d.lstop.toFixed(1); }

    updateGauge(d.y, d.sp);
    updateYawChart(d.y, d.sp);
    updateRpmChart(d.r);
  };
}

function sendCmd(cmd) {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(cmd);
    addLog('Enviado: ' + cmd);
  }
}

document.getElementById('btn-start').onclick = () => sendCmd('START');
document.getElementById('btn-stop').onclick  = () => sendCmd('STOP');
document.getElementById('btn-reset').onclick = () => sendCmd('RESET');
document.getElementById('btn-download-log').onclick = () => downloadLog(logData, 'cubesat_log_pid'); // Download do log PID

function sendSP() {
  const v = parseFloat(document.getElementById('sp-input').value);
  if (!isNaN(v)) sendCmd('S=' + v.toFixed(1));
}

function sendParams() {
  const kp   = parseFloat(document.getElementById('p-kp').value);
  const ki   = parseFloat(document.getElementById('p-ki').value);
  const kd   = parseFloat(document.getElementById('p-kd').value);
  const ls   = parseFloat(document.getElementById('p-lstop').value);
  sendCmd('P=' + kp + ',' + ki + ',' + kd + ',' + ls);
}

// Funções para a nova seção de Identificação
document.getElementById('btn-start-id').onclick = () => {
  const pwm = parseInt(document.getElementById('id-pwm-input').value);
  const duration = parseInt(document.getElementById('id-duration-input').value);
  if (!isNaN(pwm) && pwm >= 0 && pwm <= 255 && !isNaN(duration) && duration > 0) {
    sendCmd(`ID=${pwm},${duration}`);
  } else {
    alert('Por favor, insira um valor de PWM válido (0-255) e uma duração válida (ms).');
  }
};
document.getElementById('btn-stop-id').onclick = () => sendCmd('IDSTOP');
document.getElementById('btn-download-id-log').onclick = () => downloadLog(idLogData, 'cubesat_log_id'); // Download do log de identificação

function updateGauge(yaw, setpoint) {
  const canvas = document.getElementById('yawGauge');
  const ctx = canvas.getContext('2d');
  const cx = canvas.width/2, cy = canvas.height/2, r = 100;
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.beginPath(); ctx.arc(cx,cy,r,0,Math.PI*2);
  ctx.fillStyle='#1a1a2e'; ctx.fill();
  ctx.strokeStyle='#2a2a4a'; ctx.lineWidth=2; ctx.stroke();
  for (let i=0;i<12;i++) {
    const ang=(i*30-90)*Math.PI/180;
    ctx.beginPath();
    ctx.moveTo(cx+(r-12)*Math.cos(ang),cy+(r-12)*Math.sin(ang));
    ctx.lineTo(cx+r*Math.cos(ang),cy+r*Math.sin(ang));
    ctx.strokeStyle='#444'; ctx.lineWidth=1.5; ctx.stroke();
    if(i%3===0){
      const lx=cx+(r-26)*Math.cos(ang), ly=cy+(r-26)*Math.sin(ang);
      ctx.fillStyle='#555'; ctx.font='10px Segoe UI';
      ctx.textAlign='center'; ctx.textBaseline='middle';
      ctx.fillText(i*30+'°',lx,ly);
    }
  }
  if(yaw!==undefined){
    const startAng=-Math.PI/2, endAng=(yaw-90)*Math.PI/180;
    ctx.beginPath(); ctx.arc(cx,cy,r-20,startAng,endAng,false);
    ctx.strokeStyle='rgba(126,184,247,0.25)'; ctx.lineWidth=14; ctx.stroke();
  }
  if(setpoint!==undefined){
    const angSp=(setpoint-90)*Math.PI/180;
    ctx.beginPath(); ctx.moveTo(cx,cy);
    ctx.lineTo(cx+(r-18)*Math.cos(angSp),cy+(r-18)*Math.sin(angSp));
    ctx.strokeStyle='#a855f7'; ctx.lineWidth=3; ctx.stroke();
  }
  if(yaw!==undefined){
    const angY=(yaw-90)*Math.PI/180;
    ctx.beginPath(); ctx.moveTo(cx,cy);
    ctx.lineTo(cx+(r-22)*Math.cos(angY),cy+(r-22)*Math.sin(angY));
    ctx.strokeStyle='#7eb8f7'; ctx.lineWidth=4; ctx.stroke();
  }
  ctx.beginPath(); ctx.arc(cx,cy,6,0,Math.PI*2);
  ctx.fillStyle='#7eb8f7'; ctx.fill();
  ctx.font='11px Segoe UI'; ctx.textAlign='left'; ctx.textBaseline='alphabetic';
  ctx.fillStyle='#7eb8f7'; ctx.fillText('● Yaw',     cx-50,cy+r+18);
  ctx.fillStyle='#a855f7'; ctx.fillText('● Setpoint',cx-50,cy+r+32);
}

const MAX_POINTS=60;
const yawData={yaw:[],sp:[]};
function updateYawChart(yaw,sp){
  if(yaw!==undefined) yawData.yaw.push(yaw);
  if(sp !==undefined) yawData.sp.push(sp);
  if(yawData.yaw.length>MAX_POINTS) yawData.yaw.shift();
  if(yawData.sp.length >MAX_POINTS) yawData.sp.shift();
  drawYawChart();
}
function drawYawChart(){
  const canvas=document.getElementById('lineChart');
  canvas.width=canvas.offsetWidth; canvas.height=180;
  const ctx=canvas.getContext('2d');
  const W=canvas.width,H=canvas.height;
  const pad={top:10,right:10,bottom:20,left:40};
  const cw=W-pad.left-pad.right,ch=H-pad.top-pad.bottom;
  ctx.clearRect(0,0,W,H);
  for(let i=0;i<=4;i++){
    const y=pad.top+(ch/4)*i;
    ctx.strokeStyle='#2a2a4a'; ctx.lineWidth=2;
    ctx.beginPath(); ctx.moveTo(pad.left,y); ctx.lineTo(pad.left+cw,y); ctx.stroke();
    ctx.fillStyle='#555'; ctx.font='10px monospace'; ctx.textAlign='right';
    ctx.fillText(Math.round(360-(360/4)*i)+'°',pad.left-4,y+4);
  }
  function drawSeries(data,color){
    if(data.length<2) return;
    ctx.beginPath(); ctx.strokeStyle=color; ctx.lineWidth=6; ctx.lineJoin='round';
    data.forEach((v,i)=>{
      const x=pad.left+(i/(MAX_POINTS-1))*cw;
      const y=pad.top+ch-(v/360)*ch;
      i===0?ctx.moveTo(x,y):ctx.lineTo(x,y);
    });
    ctx.stroke();
  }
  drawSeries(yawData.sp,'#a855f7');
  drawSeries(yawData.yaw,'#7eb8f7');
}

const rpmHistory=[];
const RPM_MAX_DISPLAY=200;
function updateRpmChart(rpm){
  if(rpm===undefined) return;
  rpmHistory.push(rpm);
  if(rpmHistory.length>MAX_POINTS) rpmHistory.shift();
  drawRpmChart();
}
function drawRpmChart(){
  const canvas=document.getElementById('rpmChart');
  canvas.width=canvas.offsetWidth; canvas.height=140;
  const ctx=canvas.getContext('2d');
  const W=canvas.width,H=canvas.height;
  const pad={top:10,right:10,bottom:20,left:50};
  const cw=W-pad.left-pad.right,ch=H-pad.top-pad.bottom;
  ctx.clearRect(0,0,W,H);
  const zeroY=pad.top+ch/2;
  ctx.strokeStyle='#2a2a4a'; ctx.lineWidth=2;
  ctx.beginPath(); ctx.moveTo(pad.left,zeroY); ctx.lineTo(pad.left+cw,zeroY); ctx.stroke();
  [-1,-0.5,0.5,1].forEach(frac=>{
    const y=zeroY-frac*(ch/2);
    ctx.strokeStyle='#222'; ctx.lineWidth=6;
    ctx.beginPath(); ctx.moveTo(pad.left,y); ctx.lineTo(pad.left+cw,y); ctx.stroke();
    ctx.fillStyle='#555'; ctx.font='10px monospace'; ctx.textAlign='right';
    ctx.fillText(Math.round(frac*RPM_MAX_DISPLAY),pad.left-4,y+4);
  });
  if(rpmHistory.length<2) return;
  rpmHistory.forEach((v,i)=>{
    const x=pad.left+(i/(MAX_POINTS-1))*cw;
    const y=zeroY-(v/RPM_MAX_DISPLAY)*(ch/2);
    const color=v>=0?'#44ff88':'#ff9944';
    if(i===0){ ctx.beginPath(); ctx.strokeStyle=color; ctx.moveTo(x,y); }
    else {
      const prevV=rpmHistory[i-1];
      const prevColor=prevV>=0?'#44ff88':'#ff9944';
      if(color!==prevColor){
        ctx.stroke(); ctx.beginPath(); ctx.strokeStyle=color;
        const px=pad.left+((i-1)/(MAX_POINTS-1))*cw;
        const py=zeroY-(prevV/RPM_MAX_DISPLAY)*(ch/2);
        ctx.moveTo(px,py);
      }
      ctx.lineTo(x,y);
    }
  });
  ctx.stroke();
  ctx.fillStyle='#888'; ctx.font='10px Segoe UI'; ctx.textAlign='left';
  ctx.fillText('CW +', pad.left+4,pad.top+14);
  ctx.fillText('CCW -',pad.left+4,H-pad.bottom-4);
}

function addLog(msg){
  const box=document.getElementById('log');
  const p=document.createElement('p');
  p.textContent='['+new Date().toLocaleTimeString()+'] '+msg;
  box.appendChild(p); box.scrollTop=box.scrollHeight;
  if(box.children.length>60) box.removeChild(box.firstChild);
}

// Função genérica para baixar o log como CSV
function downloadLog(dataArray, filenamePrefix) {
  if (dataArray.length === 0) {
    alert('Nenhum dado de log para baixar.');
    return;
  }

  // Cabeçalho do CSV
  const headers = Object.keys(dataArray[0]).join(',');
  // Linhas de dados
  const csvRows = dataArray.map(row => Object.values(row).map(v => {
    // Se for número, mantém a precisão; se for string, mantém como está
    return typeof v === 'number' ? v : v;
  }).join(','));

  const csvContent = headers + '\n' + csvRows.join('\n');
  const blob = new Blob([csvContent], { type: 'text/csv;charset=utf-8;' });
  const link = document.createElement('a');
  const url = URL.createObjectURL(blob);
  link.setAttribute('href', url);
  link.setAttribute('download', filenamePrefix + '_' + new Date().toISOString().slice(0,19).replace('T','_').replace(/:/g,'-') + '.csv');
  link.style.visibility = 'hidden';
  document.body.appendChild(link);
  link.click();
  document.body.removeChild(link);
  addLog('Log de dados baixado como CSV (em milissegundos).');
}

connectWs();
)rawliteral";

// ============================================================
// Servidor Web e WebSocket
// ============================================================
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

void serverResponse() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", DASHBOARD_HTML);
  });
  // Novo manipulador para servir o arquivo JavaScript
  server.on("/script.js", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "application/javascript", DASHBOARD_JS);
  });
}

// ============================================================
// Telemetria
// ============================================================
volatile float    telem_yaw       = 0.0f;
volatile float    telem_setpoint  = 0.0f;
volatile float    telem_erro      = 0.0f;
volatile float    telem_gyro      = 0.0f;
volatile float    telem_rpm       = 0.0f;
volatile float    telem_u         = 0.0f;
volatile bool     telem_concluido = false;
volatile int      telem_estado    = 0; // 0: IDLE, 1: RUN, 2: IDENTIFICATION
volatile int      telem_modo      = 0;
volatile int      telem_direcao   = 0;
volatile uint32_t telem_hz        = 0;
volatile float    telem_integral  = 0.0f; // NOVO: para o valor do integral_error
volatile float    telem_uraw      = 0.0f; // NOVO: para o valor de u_raw

// Variáveis para o modo de identificação
volatile bool     id_active       = false;
volatile uint8_t  id_pwm_value    = 0;
volatile uint32_t id_duration_ms  = 0;
volatile uint32_t id_start_time   = 0;
volatile uint8_t  telem_id_pwm    = 0; // Para enviar o PWM aplicado na telemetria

void sendDataWs() {
  StaticJsonDocument<512> doc;
  doc["y"]     = (float)telem_yaw;
  doc["sp"]    = (float)telem_setpoint;
  doc["e"]     = (float)telem_erro;
  doc["g"]     = (float)telem_gyro;
  doc["r"]     = (float)telem_rpm;
  // Os campos abaixo foram removidos do dashboard, mas mantidos na telemetria para debug se necessário
  doc["u"]     = (float)telem_u;
  doc["i"]     = (float)telem_integral;
  doc["ur"]    = (float)telem_uraw;
  doc["c"]     = (int)telem_concluido;
  doc["st"]    = telem_estado;
  doc["m"]     = telem_modo;
  doc["dir"]   = telem_direcao;
  doc["ts"]    = millis();
  doc["hz"]    = telem_hz;
  doc["kp"]    = ctrl_Kp;
  doc["ki"]    = ctrl_Ki;
  doc["kd"]    = ctrl_Kd;
  doc["lstop"] = ctrl_limiarStop;

  // Adiciona dados específicos para o modo de identificação
  if (id_active) {
    doc["id_pwm"] = (int)telem_id_pwm;
  }

  char buf[512];
  size_t len = serializeJson(doc, buf);
  ws.textAll(buf, len);
}

// ============================================================
// WebSocket handler
// ============================================================
void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if (info->final && info->index == 0 && info->len == len
      && info->opcode == WS_TEXT) {
    char buf[128];
    size_t copyLen = (len < sizeof(buf)-1) ? len : sizeof(buf)-1;
    memcpy(buf, data, copyLen);
    buf[copyLen] = '\0';
    String cmd = String(buf);
    cmd.trim();
    pendingCommand    = cmd;
    hasPendingCommand = true;
  }
}

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_DATA) handleWebSocketMessage(arg, data, len);
}

// ============================================================
// Motor
// ============================================================
#define PIN_PWM   13
#define PIN_DIR   14
#define PIN_BRAKE 17

#define LEDC_FREQ_HZ  500
#define LEDC_RES_BITS 8
#define PWM_MAX       255

void releaseBrake() { digitalWrite(PIN_BRAKE, HIGH); }
void applyBrake()   { digitalWrite(PIN_BRAKE, LOW);  }

static inline void motorStopHold() {
  applyBrake();
  ledcWrite(PIN_PWM, PWM_MAX); // PWM_MAX (255) = 100% duty cycle = motor parado/travado
}
static inline void motorRunCW(uint8_t pwm_inv) {
  digitalWrite(PIN_DIR, LOW);
  releaseBrake();
  ledcWrite(PIN_PWM, pwm_inv); // pwm_inv = 0 (max speed) a 255 (min speed)
}
static inline void motorRunCCW(uint8_t pwm_inv) {
  digitalWrite(PIN_DIR, HIGH);
  releaseBrake();
  ledcWrite(PIN_PWM, pwm_inv); // pwm_inv = 0 (max speed) a 255 (min speed)
}
static inline uint8_t uToPwmInverted(float u) {
  // u é a saída do PID, onde 0 é parado e 255 é velocidade máxima.
  // pwm_inv é o valor para ledcWrite, onde 0 é velocidade máxima e 255 é parado.
  int v = PWM_MAX - (int)lroundf(u);
  return (uint8_t)constrain(v, 0, 255);
}

// ============================================================
// Encoder
// ============================================================
#define ENC1_1 26
#define ENC1_2 4
#define PPR    400

volatile long    enc_count     = 0;
volatile uint8_t encoder_state = 0;
static unsigned long last_rpm_ms   = 0;
static long          last_enc_count = 0;
static float         wheel_rpm     = 0.0f;
static const unsigned long RPM_PERIOD_MS = 100;
static const float         RPM_EMA_ALPHA = 0.3f;

void IRAM_ATTR encoder_ISR() {
  uint8_t ab = ((uint8_t)digitalRead(ENC1_1) << 1) | (uint8_t)digitalRead(ENC1_2);
  encoder_state = ((encoder_state << 2) | ab) & 0x0F;
  if      (encoder_state==0x02||encoder_state==0x0D||
           encoder_state==0x04||encoder_state==0x0B) enc_count++;
  else if (encoder_state==0x01||encoder_state==0x0E||
           encoder_state==0x08||encoder_state==0x07) enc_count--;
}

static void updateRpm() {
  const unsigned long now = millis();
  if (now - last_rpm_ms < RPM_PERIOD_MS) return;
  last_rpm_ms = now;
  long count_now;
  noInterrupts(); count_now = enc_count; interrupts();
  const long  dcount   = count_now - last_enc_count;
  last_enc_count = count_now;
  const float dt_min   = (float)RPM_PERIOD_MS / 60000.0f;
  const float rpm_inst = (dcount / (float)PPR) / dt_min;
  wheel_rpm = RPM_EMA_ALPHA * rpm_inst + (1.0f - RPM_EMA_ALPHA) * wheel_rpm;
}

// ============================================================
// Estado do sistema
// ============================================================
enum CtrlMode { MODE_IDLE = 0, MODE_COARSE = 1 };
enum SysState { IDLE = 0, RUN = 1, IDENTIFICATION = 2 }; // Adicionado estado IDENTIFICATION
SysState state            = IDLE;
bool     manobra_completa = false;

// ============================================================
// Utilidades
// ============================================================
static inline float wrap360(float a) {
  while (a >= 360.0f) a -= 360.0f;
  while (a <    0.0f) a += 360.0f;
  return a;
}
static inline float shortestPathError(float target, float current) {
  float e = target - current;
  if (e >  180.0f) e -= 360.0f;
  if (e < -180.0f) e += 360.0f;
  return e;
}

// ============================================================
// PID com slew rate
// ============================================================
float computePID(float err, float gz, float dt) {
  // Derivada diretamente pelo giroscópio
  float err_rate = -gz;

  // Integral com anti-windup
  integral_error += err * dt;
  float integral_max = 255.0f / ctrl_Ki;
  if (integral_error >  integral_max) integral_error =  integral_max;
  if (integral_error < -integral_max) integral_error = -integral_max;

  float u_raw = ctrl_Kp * err + ctrl_Ki * integral_error + ctrl_Kd * err_rate;

  // NOVO: Atualiza variáveis de telemetria para integral_error e u_raw
  telem_integral = integral_error;
  telem_uraw     = u_raw;

  // Slew rate: limita variação máxima de U por ciclo
  float du = u_raw - u_prev;
  if (du >  ctrl_slewRate) du =  ctrl_slewRate;
  if (du < -ctrl_slewRate) du = -ctrl_slewRate;
  float u = u_prev + du;
  u_prev = u;

  // Saturação física
  if (u >  255.0f) u =  255.0f;
  if (u < -255.0f) u = -255.0f;

  return u;
}

// ============================================================
// Aplica motor
// ============================================================
void applyMotor(float u) {
  float mag = fabsf(u);
  if (mag < 1.0f) { // Se o comando for muito pequeno, para o motor
    motorStopHold();
    telem_direcao = 0;
    return;
  }
  uint8_t pwm_inv = uToPwmInverted(mag); // Converte u (0-255, 0=parado, 255=max) para pwm_inv (0-255, 0=max, 255=parado)
  if (u >= 0.0f) { motorRunCW(pwm_inv);  telem_direcao =  1; } // u positivo -> CW
  else           { motorRunCCW(pwm_inv); telem_direcao = -1; } // u negativo -> CCW
}

// ============================================================
// Comandos
// ============================================================
void processCommand(String line) {
  line.trim();

  // P=kp,ki,kd,lstop
  if (line.startsWith("P=") || line.startsWith("p=")) {
    String vals = line.substring(2);
    float parsed[4];
    int idx = 0, pos = 0;
    while (idx < 4) {
      int comma = vals.indexOf(',', pos);
      String token = (comma == -1) ? vals.substring(pos) : vals.substring(pos, comma);
      parsed[idx++] = token.toFloat();
      if (comma == -1) break;
      pos = comma + 1;
    }
    if (idx == 4) {
      ctrl_Kp         = parsed[0];
      ctrl_Ki         = parsed[1];
      ctrl_Kd         = parsed[2];
      ctrl_limiarStop = parsed[3];
    }
    return;
  }

  // ID=PWM,DURATION (para identificação de sistema)
  if (line.startsWith("ID=")) {
    String vals = line.substring(3);
    int comma = vals.indexOf(',');
    if (comma != -1) {
      uint8_t pwm = vals.substring(0, comma).toInt();
      uint32_t duration = vals.substring(comma + 1).toInt();
      if (pwm >= 0 && pwm <= 255 && duration > 0) {
        id_pwm_value = pwm;
        id_duration_ms = duration;
        id_start_time = millis();
        id_active = true;
        state = IDENTIFICATION; // Entra no modo de identificação
        motorStopHold(); // Garante que o motor esteja parado antes de aplicar o PWM
        telem_id_pwm = id_pwm_value; // Atualiza telemetria para o PWM aplicado
        Serial.printf("Iniciando identificacao: PWM=%d, Duracao=%lu ms\n", id_pwm_value, id_duration_ms);
      }
    }
    return;
  }

  line.toUpperCase();

  if (line == "START") {
    manobra_completa = false;
    state = RUN;
    id_active = false; // Garante que o modo de identificação esteja desativado
  } else if (line == "STOP") {
    state = IDLE;
    manobra_completa = false;
    motorStopHold();
    telem_direcao  = 0;
    integral_error = 0.0f;
    id_active = false; // Garante que o modo de identificação esteja desativado
  } else if (line == "RESET") {
    yaw_integrated = 0.0f;
    yaw_deg        = 0.0f;
    manobra_completa = false;
    mpu9250_calibrateGyro(500);
    integral_error = 0.0f;
    id_active = false; // Garante que o modo de identificação esteja desativado
  } else if (line.startsWith("S=")) {
    setpoint_deg     = wrap360(line.substring(2).toFloat());
    manobra_completa = false;
  } else if (line == "IDSTOP") { // Comando para parar a identificação manualmente
    state = IDLE;
    id_active = false;
    motorStopHold();
    Serial.println("Identificacao parada manualmente.");
  }
}

bool readLineNonBlocking(String &out) {
  static String buf;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') { out = buf; buf = ""; return true; }
    buf += c;
    if (buf.length() > 80) buf.remove(0, buf.length() - 80);
  }
  return false;
}

// ============================================================
// Setup
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(100);

  pinMode(PIN_DIR,   OUTPUT);
  pinMode(PIN_BRAKE, OUTPUT);

  ledcAttach(PIN_PWM, LEDC_FREQ_HZ, LEDC_RES_BITS);
  motorStopHold();

  pinMode(ENC1_1, INPUT_PULLUP);
  pinMode(ENC1_2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC1_1), encoder_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC1_2), encoder_ISR, CHANGE);
  last_rpm_ms = millis();

  mpu9250_init();
  networkConnect();

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  serverResponse();
  server.begin();

  state = IDLE;
}

// ============================================================
// Loop
// ============================================================
void loop() {
  static uint32_t last_us = micros();
  static const uint32_t LOOP_DT_US = 10000; // 100 Hz

  uint32_t now_us = micros();
  if (now_us - last_us < LOOP_DT_US) return;
  float dt_s = (now_us - last_us) / 1e6f;
  last_us = now_us;

  float gz = mpu9250_getGyroZ();
  if (fabs(gz) < 0.05f) gz = 0.0f;

  yaw_integrated += gz * dt_s;
  yaw_integrated  = wrap360(yaw_integrated);
  yaw_deg         = yaw_integrated;

  updateRpm();

  float err = shortestPathError(setpoint_deg, yaw_deg);

  static CtrlMode ctrl_mode = MODE_IDLE;

  switch (state) {

    case IDLE:
      motorStopHold();
      ctrl_mode      = MODE_IDLE;
      telem_direcao  = 0;
      integral_error = 0.0f;
      telem_integral = 0.0f; // Zera também a telemetria do integral
      telem_uraw     = 0.0f; // Zera também a telemetria do u_raw
      telem_id_pwm   = 0;    // Zera o PWM de identificação na telemetria
      break;

    case RUN:
      if (manobra_completa) {
        motorStopHold();
        ctrl_mode     = MODE_IDLE;
        telem_direcao = 0;
        break;
      }
      // O usuário considera que ângulos a partir de 87° já são aceitos como atingidos.
      // Assumindo que ctrl_limiarStop já leva isso em conta ou será ajustado.
      if (fabs(err) < ctrl_limiarStop && fabs(gz) < ctrl_gyroTh) {
        manobra_completa = true;
        motorStopHold();
        ctrl_mode     = MODE_IDLE;
        telem_direcao = 0;
        break;
      }
      ctrl_mode = MODE_COARSE;
      {
        float u = computePID(err, gz, dt_s);
        telem_u = u;
        applyMotor(u);
      }
      telem_id_pwm = 0; // Garante que o PWM de identificação esteja zerado na telemetria
      break;

    case IDENTIFICATION:
      // Aplica o PWM definido para identificação
      // O PWM é um valor invertido (0=max, 255=stop)
      // Para aplicar um PWM constante, precisamos decidir a direção.
      // Por simplicidade, vamos aplicar em uma direção (ex: CW)
      // Se precisar de CCW, o usuário pode enviar um PWM negativo ou um comando específico.
      // Para este teste, vamos assumir que o PWM aplicado é sempre no sentido CW.
      // Se o PWM for 255, significa motor parado.
      if (id_pwm_value < PWM_MAX) { // Se não for PWM_MAX (parado)
        motorRunCW(id_pwm_value); // Aplica o PWM invertido diretamente
        telem_direcao = 1; // Indica direção CW
      } else {
        motorStopHold();
        telem_direcao = 0;
      }
      telem_id_pwm = id_pwm_value; // Atualiza telemetria com o PWM aplicado

      // Verifica se a duração do teste de identificação terminou
      if (millis() - id_start_time >= id_duration_ms) {
        state = IDLE; // Volta para o estado IDLE
        id_active = false;
        motorStopHold();
        Serial.println("Teste de identificacao concluido.");
      }
      break;
  }

  telem_yaw       = yaw_deg;
  telem_setpoint  = setpoint_deg;
  telem_erro      = err;
  telem_gyro      = gz;
  telem_rpm       = wheel_rpm;
  telem_concluido = manobra_completa;
  telem_estado    = (int)state; // Atualiza o estado para a telemetria
  telem_modo      = (int)ctrl_mode;

  static uint32_t loopCount  = 0;
  static uint32_t lastReport = 0;
  loopCount++;
  uint32_t now_ms = millis();
  if (now_ms - lastReport >= 1000) {
    lastReport = now_ms;
    telem_hz   = loopCount;
    loopCount  = 0;
  }

  static uint32_t lastWsSend    = 0;
  static uint32_t lastWsCleanup = 0;
  if (now_ms - lastWsSend >= 20) { // Envia dados a cada 20ms (50 Hz)
    lastWsSend = now_ms;
    sendDataWs();
  }
  if (now_ms - lastWsCleanup >= 1000) {
    lastWsCleanup = now_ms;
    ws.cleanupClients();
  }

  if (hasPendingCommand) {
    processCommand(pendingCommand);
    hasPendingCommand = false;
    pendingCommand    = "";
  }
  String line;
  if (readLineNonBlocking(line)) processCommand(line);
}