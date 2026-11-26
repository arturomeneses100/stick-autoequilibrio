/*
 * ============================================================================
 *  Stick de autoequilibrio de un eje (rueda de reacción, motor DC) — ESP32
 *  Autor: Arturo Elfego Meneses Flores
 *  Basado en la idea de remrc/One-Axis-Self-Balancing-Stick-DC (Arduino Nano)
 *  Port y optimización: lazo de control en tiempo real a 500 Hz (FreeRTOS),
 *  realimentación de estado completo, parámetros ajustables en caliente
 *  por USB-Serial o Bluetooth y guardados en NVS (memoria no volátil).
 * ============================================================================
 *
 *  HARDWARE
 *    ESP32 DevKit (WROOM/WROVER, ESP32 "clásico" si se quiere Bluetooth SPP)
 *    MPU6050 (I2C 0x68) · AS5600 (I2C 0x36) · TB6612FNG · motor DC · LiPo 3S
 *
 *  CONEXIONES (cámbialas en la sección PINES)
 *    MPU6050 / AS5600   SDA -> GPIO21   SCL -> GPIO22   VCC -> 3V3   GND
 *    TB6612   PWMA -> GPIO25   AIN1 -> GPIO26   AIN2 -> GPIO27   STBY -> GPIO33
 *             VCC -> 3V3       VM -> batería (+)  A01/A02 -> motor
 *    Divisor batería: Vbat -- 100k --+-- 22k -- GND ; el punto medio -> GPIO34
 *    Buzzer activo (opcional) -> GPIO32 ; LED de estado -> GPIO2
 *    ¡AS5600 a 3.3 V! (algunos módulos traen puente 3V3/5V).
 *
 *  ANTI-RUIDO (causa nº1 de "se cae a ratos" con motor DC):
 *    - 1 capacitor cerámico 100 nF entre terminales del motor y uno de cada
 *      terminal a la carcasa. Cables del motor trenzados y lejos del I2C.
 *    - 470–1000 µF en VM del TB6612. Pull-ups I2C de 4.7k a 3V3 (cables cortos).
 *
 *  CONVENCIÓN DE SIGNOS  (todas las ganancias K1..K4 son POSITIVAS)
 *      u = K1*theta[deg] + K2*omega[deg/s] + K3*vel_rueda[rps] + K4*pos_rueda[rev]
 *    Comprueba en este orden (con la telemetría: "tel 20"):
 *    1) "m 0.3"  -> la rueda gira y ws debe salir POSITIVO. Si no: "inv enc".
 *    2) Inclina el stick a mano: mientras th sube, w debe ser positivo.
 *       Si w va al revés: "inv gyro".
 *    3) Sostén el stick suelto y da "m 0.4 0.3": el golpe de reacción debe
 *       empujar el cuerpo hacia theta NEGATIVO. Si lo empuja a positivo:
 *       "inv imu". Después "save".
 *
 *  CALIBRACIÓN DEL PUNTO DE EQUILIBRIO (igual que el original)
 *    "c+" -> sostén el stick en su punto de equilibrio exacto -> "c-"
 *    (guarda el offset en NVS y empieza a equilibrar). Con "trim" > 0 el
 *    offset se autoajusta lentamente; "save" lo hace permanente.
 *
 *  SINTONÍA (orden recomendado)
 *    1) K3 = K4 = 0. Sube K1 hasta que reaccione firme; sube K2 hasta que
 *       deje de oscilar (K2 ~ K1/10). Se sostendrá unos segundos mientras la
 *       rueda acelera hasta saturar: normal.
 *    2) Sube K3 hasta que la rueda deje de desbocarse.
 *    3) K4 pequeño para que la rueda regrese a su posición inicial.
 *    4) "dbfind" con el stick acostado mide la zona muerta del motor -> "db".
 *    Comandos estilo original también funcionan: p+ p- i+ i- a+ a- s+ s-
 *    (K1,K2,K3,K4 respectivamente; "p+p+p+" = tres pasos de 5 %).
 * ============================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <math.h>
#include "esp_timer.h"

// ---------------------------------------------------------------------------
//  OPCIONES DE COMPILACIÓN
// ---------------------------------------------------------------------------
#define USE_BLUETOOTH 1   // 0 = solo USB. Requiere ESP32 clásico (no S2/S3/C3)
#define BT_NAME "Stick-ESP32"

#if USE_BLUETOOTH && defined(CONFIG_BT_ENABLED) && defined(CONFIG_BLUEDROID_ENABLED) && defined(CONFIG_BT_SPP_ENABLED)
  #include "BluetoothSerial.h"
  #define BT_OK 1
  BluetoothSerial SerialBT;
#else
  #define BT_OK 0
#endif

// ---------------------------------------------------------------------------
//  PINES
// ---------------------------------------------------------------------------
constexpr int PIN_SDA    = 21;
constexpr int PIN_SCL    = 22;
constexpr int PIN_PWMA   = 25;
constexpr int PIN_AIN1   = 26;
constexpr int PIN_AIN2   = 27;
constexpr int PIN_STBY   = 33;   // -1 si STBY va fijo a 3V3
constexpr int PIN_VBAT   = 34;   // -1 si no hay divisor (desactiva compensación)
constexpr int PIN_BUZZER = 32;   // -1 si no hay buzzer
constexpr int PIN_LED    = 2;    // -1 si no hay LED

constexpr float VBAT_DIVIDER = (100.0f + 22.0f) / 22.0f;  // (R1+R2)/R2
constexpr int   BAT_CELLS    = 3;

// ---------------------------------------------------------------------------
//  CONSTANTES DE TIEMPO REAL / SENSORES
// ---------------------------------------------------------------------------
constexpr uint32_t LOOP_US   = 2000;     // 500 Hz (el original: 10 ms = 100 Hz)
constexpr uint32_t I2C_FREQ  = 400000;   // 400 kHz: lectura IMU+encoder ~0.6 ms
constexpr uint32_t PWM_FREQ  = 20000;    // 20 kHz: inaudible
constexpr uint8_t  PWM_BITS  = 11;       // 0..2047
constexpr int      PWM_MAX   = (1 << PWM_BITS) - 1;

// Orientación del MPU6050: 0 = X, 1 = Y, 2 = Z
constexpr int   AX_TILT  = 1;    // eje del acelerómetro horizontal en el plano de caída
constexpr int   AX_UP    = 0;    // eje del acelerómetro que apunta hacia ARRIBA en vertical
constexpr float UP_SIGN  = 1.0f; // -1 si ese eje apunta hacia abajo (ver comando "imu")
constexpr int   AX_GYRO  = 2;    // eje del giroscopio paralelo al eje de la rueda

constexpr uint8_t MPU_ADDR  = 0x68;
constexpr uint8_t MPU_DLPF  = 2;        // 1=188Hz 2=98Hz 3=44Hz (más alto = menos retardo)
constexpr float   GYRO_LSB  = 65.5f;    // ±500 °/s
constexpr float   ACC_LSB   = 16384.0f; // ±2 g

constexpr uint8_t AS5600_ADDR = 0x36;
constexpr uint8_t AS5600_SF   = 1;      // filtro interno: 0=16x 1=8x 2=4x 3=2x (más rápido)
constexpr float   ENC_CPR     = 4096.0f;

constexpr float RAD2DEG = 57.29578f;
constexpr float SOFT_START_S   = 0.15f;  // rampa de arranque
constexpr float TRIM_RANGE_DEG = 3.0f;   // máximo autoajuste del offset
constexpr uint32_t UPRIGHT_MS  = 200;    // tiempo vertical antes de enganchar

// ---------------------------------------------------------------------------
//  PARÁMETROS AJUSTABLES (se guardan en NVS)
// ---------------------------------------------------------------------------
constexpr uint32_t PARAM_MAGIC = 0x53544B03;   // cambia si modificas la estructura

struct Params {
  float K1, K2, K3, K4;   // ganancias de realimentación de estado
  float offset;           // ángulo de equilibrio [deg]
  float tau;              // constante del filtro complementario [s]
  float wsLpfHz;          // filtro de velocidad de rueda [Hz]
  float gyroLpfHz;        // filtro extra del término D [Hz] (0 = off)
  float deadband;         // compensación de zona muerta del motor [0..0.5]
  float maxAngle;         // ángulo de caída [deg]
  float startAngle;       // ángulo para enganchar [deg]
  float trimK;            // autoajuste de offset [deg/(rps*s)] (0 = off)
  float maxRps;           // velocidad máxima de rueda antes de soltar [rps]
  float vNom;             // voltaje de referencia para compensación [V]
  float vComp;            // 1 = compensar voltaje de batería
  float fallBrake;        // 1 = frenar la rueda al caer, 0 = rueda libre
  float invMotor, invEnc, invImu, invGyro;
  uint32_t magic;
};

Params P;

void setDefaults() {
  // Punto de partida obtenido por LQR para un stick típico (~20 cm, rueda ~10 cm,
  // motor DC 12 V) y probado en simulación con zona muerta y retardo de un ciclo.
  P.K1 = 0.45f;  P.K2 = 0.045f;  P.K3 = 0.08f;  P.K4 = 0.015f;
  P.offset = 0.0f;
  P.tau = 0.5f;  P.wsLpfHz = 25.0f;  P.gyroLpfHz = 0.0f;
  P.deadband = 0.05f;
  P.maxAngle = 12.0f;  P.startAngle = 0.5f;
  P.trimK = 0.02f;  P.maxRps = 80.0f;
  P.vNom = 11.1f;  P.vComp = 1;  P.fallBrake = 1;
  P.invMotor = 0; P.invEnc = 0; P.invImu = 0; P.invGyro = 0;
  P.magic = PARAM_MAGIC;
}

struct PDesc { const char* name; float* v; float lo, hi, step; const char* help; };
PDesc PT[] = {
  {"k1",      &P.K1,        0, 10,   0.01f,  "ganancia angulo [duty/deg]"},
  {"k2",      &P.K2,        0, 2,    0.001f, "ganancia vel. angular [duty/(deg/s)]"},
  {"k3",      &P.K3,        0, 2,    0.002f, "ganancia vel. rueda [duty/rps]"},
  {"k4",      &P.K4,        0, 1,    0.0005f,"ganancia pos. rueda [duty/rev]"},
  {"off",     &P.offset,  -45, 45,   0.05f,  "angulo de equilibrio [deg]"},
  {"tau",     &P.tau,    0.05, 5,    0.05f,  "filtro complementario [s]"},
  {"wlpf",    &P.wsLpfHz,   1, 200,  1.0f,   "filtro vel. rueda [Hz]"},
  {"glpf",    &P.gyroLpfHz, 0, 200,  1.0f,   "filtro termino D [Hz] (0=off)"},
  {"db",      &P.deadband,  0, 0.5f, 0.005f, "zona muerta motor [0..0.5]"},
  {"maxang",  &P.maxAngle,  2, 45,   0.5f,   "angulo de caida [deg]"},
  {"startang",&P.startAngle,0.1f,10, 0.1f,   "angulo para arrancar [deg]"},
  {"trim",    &P.trimK,     0, 1,    0.005f, "autoajuste offset (0=off)"},
  {"maxrps",  &P.maxRps,    5, 300,  5.0f,   "vel. max rueda [rps]"},
  {"vnom",    &P.vNom,      3, 30,   0.1f,   "voltaje nominal [V]"},
  {"vcomp",   &P.vComp,     0, 1,    1.0f,   "compensar bateria (0/1)"},
  {"brake",   &P.fallBrake, 0, 1,    1.0f,   "frenar al caer (0/1)"},
};
constexpr int NPT = sizeof(PT) / sizeof(PT[0]);

Preferences prefs;

void saveParams()  { prefs.putBytes("p", &P, sizeof(P)); }
bool loadParams() {
  Params tmp;
  if (prefs.getBytesLength("p") != sizeof(Params)) return false;
  prefs.getBytes("p", &tmp, sizeof(tmp));
  if (tmp.magic != PARAM_MAGIC) return false;
  P = tmp;
  return true;
}

// ---------------------------------------------------------------------------
//  ESTADO COMPARTIDO ENTRE TAREAS
// ---------------------------------------------------------------------------
enum State : uint8_t { ST_IDLE, ST_WAIT, ST_BAL, ST_GYROCAL, ST_CAL, ST_MOTORTEST, ST_DBFIND, ST_FAULT };
const char* STATE_NAME[] = {"IDLE", "ESPERA", "EQUILIBRANDO", "CAL_GIRO", "CAL_OFFSET", "PRUEBA_MOTOR", "DBFIND", "FALLA"};

volatile State state = ST_GYROCAL;

struct Telem {
  float theta, thetaRaw, omega, ws, wp, u, trim;
  int16_t acc[3], gyr[3];
  uint8_t state;
};
Telem tel;
portMUX_TYPE telMux = portMUX_INITIALIZER_UNLOCKED;

// Peticiones desde la tarea de comunicación
volatile float    testU = 0;
volatile uint32_t testUntilMs = 0;
volatile bool     reqCalFinish = false;
volatile bool     reqZeroTrim = false;
volatile bool     reqMagStatus = false;
volatile bool     reqReinit = false;

// Eventos desde la tarea de control
volatile uint8_t evGyroCal = 0;    // 1 ok, 2 movimiento
volatile bool    evCalDone = false;
volatile bool    evFell = false, evEngaged = false, evFault = false, evRunaway = false;
volatile uint8_t evReinit = 0;     // 1 ok, 2 falló
volatile float   evDbResult = -1;
volatile uint8_t magStatus = 0, magAgc = 0;

// Estadísticas
volatile uint32_t statExecMax = 0, statPeriodMax = 0, statPeriodMin = UINT32_MAX;
volatile uint32_t statOverruns = 0, statI2cErr = 0, statLoops = 0;

volatile float vbat = 0;

TaskHandle_t ctrlHandle = nullptr;
esp_timer_handle_t loopTimer = nullptr;

// ---------------------------------------------------------------------------
//  I2C
// ---------------------------------------------------------------------------
bool i2cWrite(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg); Wire.write(val);
  return Wire.endTransmission() == 0;
}
bool i2cRead(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t n) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint16_t)addr, (size_t)n, true) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

bool mpuInit(uint8_t* who) {
  if (!i2cWrite(MPU_ADDR, 0x6B, 0x80)) return false;  // reset
  delay(100);
  bool ok = i2cWrite(MPU_ADDR, 0x6B, 0x01)              // reloj PLL giro X
         && i2cWrite(MPU_ADDR, 0x19, 0x00)              // 1 kHz
         && i2cWrite(MPU_ADDR, 0x1A, MPU_DLPF)          // filtro digital
         && i2cWrite(MPU_ADDR, 0x1B, 0x08)              // ±500 °/s
         && i2cWrite(MPU_ADDR, 0x1C, 0x00);             // ±2 g
  i2cRead(MPU_ADDR, 0x75, who, 1);
  return ok;
}
bool mpuRead(int16_t acc[3], int16_t gyr[3]) {
  uint8_t b[14];
  if (!i2cRead(MPU_ADDR, 0x3B, b, 14)) return false;
  for (int i = 0; i < 3; i++) {
    acc[i] = (int16_t)((b[2 * i] << 8) | b[2 * i + 1]);
    gyr[i] = (int16_t)((b[8 + 2 * i] << 8) | b[9 + 2 * i]);
  }
  return true;
}

bool as5600Init() {
  uint8_t conf[2];
  if (!i2cRead(AS5600_ADDR, 0x07, conf, 2)) return false;
  conf[0] = (conf[0] & ~0x3F) | (AS5600_SF & 0x03);  // SF, FTH=0, WD=0
  conf[1] &= ~0x0C;                                  // histéresis OFF
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(0x07); Wire.write(conf[0]); Wire.write(conf[1]);
  return Wire.endTransmission() == 0;               // (volátil, no quema OTP)
}
bool as5600Read(uint16_t& raw) {
  uint8_t b[2];
  if (!i2cRead(AS5600_ADDR, 0x0C, b, 2)) return false;
  raw = ((uint16_t)(b[0] & 0x0F) << 8) | b[1];
  return true;
}

// ---------------------------------------------------------------------------
//  MOTOR (TB6612)
// ---------------------------------------------------------------------------
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  inline void pwmInit()             { ledcAttach(PIN_PWMA, PWM_FREQ, PWM_BITS); }
  inline void pwmWrite(uint32_t d)  { ledcWrite(PIN_PWMA, d); }
#else
  inline void pwmInit()             { ledcSetup(0, PWM_FREQ, PWM_BITS); ledcAttachPin(PIN_PWMA, 0); }
  inline void pwmWrite(uint32_t d)  { ledcWrite(0, d); }
#endif

int8_t motorDir = 0;   // 1, -1, 0 = libre, 2 = freno

void motorCoast() {
  if (motorDir != 0) { digitalWrite(PIN_AIN1, LOW); digitalWrite(PIN_AIN2, LOW); motorDir = 0; }
  pwmWrite(0);
}
void motorBrake() {
  if (motorDir != 2) { digitalWrite(PIN_AIN1, HIGH); digitalWrite(PIN_AIN2, HIGH); motorDir = 2; }
  pwmWrite(PWM_MAX);
}
// u en [-1, 1], ya compensado. Positivo = acelera la rueda en sentido positivo.
void motorWrite(float u) {
  if (P.invMotor > 0.5f) u = -u;
  u = constrain(u, -1.0f, 1.0f);
  uint32_t duty = (uint32_t)(fabsf(u) * PWM_MAX + 0.5f);
  if (duty == 0) { motorCoast(); return; }
  int8_t d = (u > 0) ? 1 : -1;
  if (d != motorDir) {
    digitalWrite(PIN_AIN1, d > 0 ? HIGH : LOW);
    digitalWrite(PIN_AIN2, d > 0 ? LOW : HIGH);
    motorDir = d;
  }
  pwmWrite(duty);
}
void motorStopFall() { if (P.fallBrake > 0.5f) motorBrake(); else motorCoast(); }

// Compensación de zona muerta suave (sin salto brusco cerca de cero)
inline float applyDeadband(float u) {
  float a = fminf(fabsf(u), 1.0f);
  if (a < 1e-4f) return 0.0f;
  float out = P.deadband * fminf(a / 0.02f, 1.0f) + (1.0f - P.deadband) * a;
  return copysignf(out, u);
}
inline float lpfAlpha(float fc, float dt) { return 1.0f - expf(-6.2831853f * fc * dt); }

// ---------------------------------------------------------------------------
//  TAREA DE CONTROL (núcleo 1, prioridad alta, disparada por esp_timer)
// ---------------------------------------------------------------------------
static void onLoopTimer(void*) { if (ctrlHandle) xTaskNotifyGive(ctrlHandle); }

void controlTask(void*) {
  int64_t tPrev = esp_timer_get_time();
  int64_t tLast = tPrev;

  float thetaRaw = 0, omegaF = 0, ws = 0, trimDelta = 0, ramp = 0;
  bool angleInit = false, encInit = false;
  float gyroBias = 0;
  uint16_t encPrev = 0;
  int32_t encCount = 0, encZero = 0;
  uint8_t errRun = 0;
  uint32_t uprightSince = 0;
  double gSum = 0, gSq = 0; uint32_t gN = 0;
  double cSum = 0; uint32_t cN = 0;
  float dbU = 0;
  State prevState = ST_IDLE;

  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    int64_t t0 = esp_timer_get_time();
    uint32_t period = (uint32_t)(t0 - tLast); tLast = t0;
    float dt = (t0 - tPrev) * 1e-6f; tPrev = t0;
    if (dt <= 0.0f || dt > 0.02f) dt = LOOP_US * 1e-6f;
    uint32_t nowMs = (uint32_t)(t0 / 1000);

    if (reqReinit) {                  // reintento tras FALLA (Wire solo se usa en esta tarea)
      reqReinit = false;
      motorCoast();
      uint8_t who;
      bool okInit = mpuInit(&who) && as5600Init();
      evReinit = okInit ? 1 : 2;
      if (okInit) { encInit = false; angleInit = false; errRun = 0; state = ST_GYROCAL; }
      tPrev = tLast = esp_timer_get_time();
      continue;
    }

    // ---------- 1. Sensores ----------
    int16_t acc[3], gyr[3]; uint16_t enc;
    bool ok = mpuRead(acc, gyr) && as5600Read(enc);
    if (!ok) {
      statI2cErr++;
      if (++errRun >= 10 && state != ST_FAULT) { state = ST_FAULT; evFault = true; }
      if (errRun >= 3) motorCoast();
      continue;                       // conserva la última salida 1-2 ciclos
    }
    errRun = 0;

    if (reqMagStatus) {
      uint8_t s, a;
      if (i2cRead(AS5600_ADDR, 0x0B, &s, 1)) magStatus = s;
      if (i2cRead(AS5600_ADDR, 0x1A, &a, 1)) magAgc = a;
      reqMagStatus = false;
    }

    // ---------- 2. Ángulo (filtro complementario con compuerta de aceleración) ----------
    float sImu = (P.invImu > 0.5f) ? -1.0f : 1.0f;
    float sGyr = (P.invGyro > 0.5f) ? -sImu : sImu;
    float omega = sGyr * ((float)gyr[AX_GYRO] - gyroBias) / GYRO_LSB;
    float accAng = sImu * atan2f((float)acc[AX_TILT], UP_SIGN * (float)acc[AX_UP]) * RAD2DEG;
    float ax = acc[0] / ACC_LSB, ay = acc[1] / ACC_LSB, az = acc[2] / ACC_LSB;
    float gNorm = sqrtf(ax * ax + ay * ay + az * az);

    if (!angleInit) { thetaRaw = accAng; angleInit = true; }
    else {
      thetaRaw += omega * dt;
      if (fabsf(gNorm - 1.0f) < 0.15f) {         // ignora el acelerómetro en golpes
        float alpha = P.tau / (P.tau + dt);
        thetaRaw = alpha * thetaRaw + (1.0f - alpha) * accAng;
      }
    }
    omegaF = (P.gyroLpfHz > 0.0f) ? omegaF + lpfAlpha(P.gyroLpfHz, dt) * (omega - omegaF) : omega;

    // ---------- 3. Rueda (desenvolvimiento multivuelta + velocidad filtrada) ----------
    if (!encInit) { encPrev = enc; encInit = true; }
    int32_t d = (int32_t)enc - (int32_t)encPrev;
    encPrev = enc;
    if (d > 2048) d -= 4096; else if (d < -2048) d += 4096;
    if (P.invEnc > 0.5f) d = -d;
    encCount += d;
    float wsRaw = (d / ENC_CPR) / dt;                       // rps
    ws += lpfAlpha(P.wsLpfHz, dt) * (wsRaw - ws);
    float wp = (encCount - encZero) / ENC_CPR;              // rev

    if (reqZeroTrim) { trimDelta = 0; reqZeroTrim = false; }
    float theta = thetaRaw - P.offset - trimDelta;
    float u = 0;

    // ---------- 4. Máquina de estados ----------
    State st = state;
    if (st != prevState) {               // acciones de entrada a estado
      if (st == ST_GYROCAL) { gSum = gSq = 0; gN = 0; }
      if (st == ST_CAL)     { cSum = 0; cN = 0; }
      if (st == ST_DBFIND)  { dbU = 0; }
      if (st == ST_WAIT)    { uprightSince = 0; }
      prevState = st;
    }

    switch (st) {
      case ST_GYROCAL: {
        motorCoast();
        double g = gyr[AX_GYRO];
        gSum += g; gSq += g * g; gN++;
        if (gN >= 1000) {
          double mean = gSum / gN, var = gSq / gN - mean * mean;
          if (var < (1.5 * GYRO_LSB) * (1.5 * GYRO_LSB)) {
            gyroBias = (float)mean; angleInit = false;
            evGyroCal = 1; state = ST_WAIT;
          } else { evGyroCal = 2; gSum = gSq = 0; gN = 0; }
        }
      } break;

      case ST_CAL:
        motorCoast();
        cSum += thetaRaw; cN++;
        if (reqCalFinish) {
          reqCalFinish = false;
          if (cN > 0) P.offset = (float)(cSum / cN);
          trimDelta = 0;
          evCalDone = true;
          state = ST_WAIT;
        }
        break;

      case ST_WAIT:
        motorStopFall();
        if (fabsf(theta) < P.startAngle) {
          if (!uprightSince) uprightSince = nowMs ? nowMs : 1;
          if (nowMs - uprightSince >= UPRIGHT_MS) {
            encZero = encCount; ramp = 0; omegaF = omega;
            state = ST_BAL; evEngaged = true;
          }
        } else uprightSince = 0;
        break;

      case ST_BAL: {
        if (fabsf(theta) > P.maxAngle || fabsf(ws) > P.maxRps) {
          if (fabsf(ws) > P.maxRps) evRunaway = true; else evFell = true;
          motorStopFall(); state = ST_WAIT; break;
        }
        u = P.K1 * theta + P.K2 * omegaF + P.K3 * ws + P.K4 * wp;

        if (P.trimK > 0.0f) {                      // autoajuste lento del offset
          trimDelta -= P.trimK * (ws + 0.5f * wp) * dt;   // vel + pos/2: corrige el offset completo
          trimDelta = constrain(trimDelta, -TRIM_RANGE_DEG, TRIM_RANGE_DEG);
        }
        ramp = fminf(1.0f, ramp + dt / SOFT_START_S);
        u *= ramp;

        float uo = applyDeadband(u);
        float vb = vbat;
        if (P.vComp > 0.5f && vb > 5.0f) uo *= P.vNom / vb;
        motorWrite(uo);
      } break;

      case ST_MOTORTEST:
        if ((int32_t)(nowMs - testUntilMs) >= 0) { motorCoast(); state = ST_IDLE; }
        else { u = testU; motorWrite(u); }
        break;

      case ST_DBFIND:                               // rampa 5 %/s hasta que gire
        dbU += 0.05f * dt;
        u = dbU; motorWrite(u);
        if (fabsf(ws) > 0.3f) { evDbResult = dbU; motorCoast(); state = ST_IDLE; }
        else if (dbU > 0.5f)  { evDbResult = 0;   motorCoast(); state = ST_IDLE; }
        break;

      case ST_IDLE:
      case ST_FAULT:
      default:
        motorCoast();
        break;
    }

    // ---------- 5. Telemetría y estadísticas ----------
    portENTER_CRITICAL(&telMux);
    tel.theta = theta; tel.thetaRaw = thetaRaw; tel.omega = omegaF;
    tel.ws = ws; tel.wp = wp; tel.u = u; tel.trim = trimDelta; tel.state = (uint8_t)state;
    for (int i = 0; i < 3; i++) { tel.acc[i] = acc[i]; tel.gyr[i] = gyr[i]; }
    portEXIT_CRITICAL(&telMux);

    uint32_t exec = (uint32_t)(esp_timer_get_time() - t0);
    if (exec > statExecMax) statExecMax = exec;
    if (exec > LOOP_US) statOverruns++;
    if (statLoops > 10) {
      if (period > statPeriodMax) statPeriodMax = period;
      if (period < statPeriodMin) statPeriodMin = period;
    }
    statLoops++;
  }
}

// ---------------------------------------------------------------------------
//  COMUNICACIÓN (Serial USB + Bluetooth) — corre en loop(), prioridad baja
// ---------------------------------------------------------------------------
Print* telOut = nullptr;
uint32_t telPeriodMs = 0, telLast = 0;
uint32_t buzzUntil = 0;

void beep(uint32_t ms) {
  if (PIN_BUZZER < 0) return;
  digitalWrite(PIN_BUZZER, HIGH); buzzUntil = millis() + ms;
}

void say(const char* fmt, ...) {
  char buf[200];
  va_list ap; va_start(ap, fmt); vsnprintf(buf, sizeof(buf), fmt, ap); va_end(ap);
  Serial.print(buf);
#if BT_OK
  if (SerialBT.hasClient()) SerialBT.print(buf);
#endif
}

Telem getTel() { Telem t; portENTER_CRITICAL(&telMux); t = tel; portEXIT_CRITICAL(&telMux); return t; }

void printParams(Print& o) {
  o.printf("--- Parametros ---\n");
  for (int i = 0; i < NPT; i++) o.printf("%-9s = %-10.5g ; %s\n", PT[i].name, *PT[i].v, PT[i].help);
  o.printf("inv: motor=%d enc=%d imu=%d gyro=%d\n",
           (int)P.invMotor, (int)P.invEnc, (int)P.invImu, (int)P.invGyro);
  o.printf("estado=%s  vbat=%.2f V\n", STATE_NAME[state], vbat);
}

void printHelp(Print& o) {
  o.print(
    "--- Comandos ---\n"
    " get | help | stat | imu\n"
    " <param> <valor>   ej: k1 0.5  (param: k1 k2 k3 k4 off tau wlpf glpf db\n"
    "                    maxang startang trim maxrps vnom vcomp brake)\n"
    " <param>+ / <param>-   +/-5%      p+ i+ a+ s+ (estilo original)\n"
    " start | stop      habilita / deshabilita el equilibrio\n"
    " c+ ... c-         calibra punto de equilibrio (guarda)\n"
    " zero              offset = angulo actual (sin guardar)\n"
    " inv motor|enc|imu|gyro   invierte signos\n"
    " m <u> [seg]       prueba de motor, u en -1..1\n"
    " dbfind            mide zona muerta (stick acostado, rueda libre)\n"
    " gyrocal           recalibra el giroscopio (quieto)\n"
    " tel <Hz>          telemetria para Serial Plotter (0 = off)\n"
    " save | load | defaults | reboot\n");
}

void printStat(Print& o) {
  reqMagStatus = true; delay(10);
  uint8_t ms = magStatus;
  o.printf("lazo: periodo %lu..%lu us (obj %lu), ejecucion max %lu us, sobrecargas %lu\n",
           (unsigned long)statPeriodMin, (unsigned long)statPeriodMax, (unsigned long)LOOP_US,
           (unsigned long)statExecMax, (unsigned long)statOverruns);
  o.printf("errores I2C: %lu   iman AS5600: %s%s%s AGC=%u   vbat=%.2f V\n",
           (unsigned long)statI2cErr,
           (ms & 0x20) ? "detectado " : "NO DETECTADO ",
           (ms & 0x10) ? "(debil) " : "", (ms & 0x08) ? "(fuerte) " : "", magAgc, vbat);
  statExecMax = 0; statPeriodMax = 0; statPeriodMin = UINT32_MAX;
}

void printImu(Print& o) {
  Telem t = getTel();
  o.printf("acc[g]  X=%.2f Y=%.2f Z=%.2f   gyro[raw] X=%d Y=%d Z=%d\n",
           t.acc[0] / ACC_LSB, t.acc[1] / ACC_LSB, t.acc[2] / ACC_LSB, t.gyr[0], t.gyr[1], t.gyr[2]);
  o.printf("theta=%.2f (crudo %.2f) omega=%.1f  -> en vertical AX_UP debe leer ~+1 g\n",
           t.theta, t.thetaRaw, t.omega);
}

void stepParam(float* v, float minStep, int sign) {
  float s = fmaxf(fabsf(*v) * 0.05f, minStep);
  *v = fmaxf(0.0f, *v + sign * s);
}

bool isLegacy(const char* s) {
  size_t n = strlen(s);
  if (n < 2 || (n & 1)) return false;
  for (size_t i = 0; i < n; i += 2)
    if (!strchr("piasc", s[i]) || (s[i + 1] != '+' && s[i + 1] != '-')) return false;
  return true;
}

void startCal(Print& o)  { state = ST_CAL; o.println("CAL: sostén el stick en su punto de equilibrio y envía c-"); }
void finishCal(Print& o) {
  if (state != ST_CAL) { o.println("primero envía c+"); return; }
  reqCalFinish = true;
}

void processLine(char* line, Print& o) {
  for (char* p = line; *p; p++) *p = tolower(*p);
  while (*line == ' ') line++;
  size_t n = strlen(line);
  while (n && (line[n - 1] == ' ' || line[n - 1] == '\r')) line[--n] = 0;
  if (!n) return;

  if (isLegacy(line)) {                             // p+ i- a+ s+ c+ c- (y repeticiones)
    for (size_t i = 0; i < n; i += 2) {
      int sg = line[i + 1] == '+' ? 1 : -1;
      switch (line[i]) {
        case 'p': stepParam(&P.K1, PT[0].step, sg); break;
        case 'i': stepParam(&P.K2, PT[1].step, sg); break;
        case 'a': stepParam(&P.K3, PT[2].step, sg); break;
        case 's': stepParam(&P.K4, PT[3].step, sg); break;
        case 'c': if (sg > 0) startCal(o); else finishCal(o); break;
      }
    }
    o.printf("K1=%.4f K2=%.4f K3=%.4f K4=%.4f\n", P.K1, P.K2, P.K3, P.K4);
    return;
  }

  char* cmd = strtok(line, " =\t");
  char* a1  = strtok(nullptr, " =\t");
  char* a2  = strtok(nullptr, " =\t");
  if (!cmd) return;

  // Parámetros de la tabla: "k1", "k1 0.5", "k1+", "k1-"
  size_t cl = strlen(cmd);
  int sg = 0;
  if (cl > 1 && (cmd[cl - 1] == '+' || cmd[cl - 1] == '-')) { sg = cmd[cl - 1] == '+' ? 1 : -1; cmd[cl - 1] = 0; }
  for (int i = 0; i < NPT; i++) {
    if (strcmp(cmd, PT[i].name) == 0) {
      if (sg) *PT[i].v = constrain(*PT[i].v + sg * fmaxf(fabsf(*PT[i].v) * 0.05f, PT[i].step), PT[i].lo, PT[i].hi);
      else if (a1) *PT[i].v = constrain((float)atof(a1), PT[i].lo, PT[i].hi);
      if (PT[i].v == &P.offset && (sg || a1)) reqZeroTrim = true;
      o.printf("%s = %.5g\n", PT[i].name, *PT[i].v);
      return;
    }
  }

  if (!strcmp(cmd, "help") || !strcmp(cmd, "?")) printHelp(o);
  else if (!strcmp(cmd, "get"))      printParams(o);
  else if (!strcmp(cmd, "stat"))     printStat(o);
  else if (!strcmp(cmd, "imu"))      printImu(o);
  else if (!strcmp(cmd, "start")) {
    if (state == ST_FAULT) { reqReinit = true; o.println("reiniciando sensores..."); }
    else { state = ST_WAIT; o.println("ESPERA: pon el stick vertical"); }
  }
  else if (!strcmp(cmd, "stop"))     { state = ST_IDLE; o.println("IDLE"); }
  else if (!strcmp(cmd, "cal"))      startCal(o);
  else if (!strcmp(cmd, "zero"))     { P.offset = getTel().thetaRaw; reqZeroTrim = true; o.printf("off = %.3f\n", P.offset); }
  else if (!strcmp(cmd, "gyrocal"))  { state = ST_GYROCAL; o.println("calibrando giroscopio, no lo muevas (2 s)"); }
  else if (!strcmp(cmd, "save")) {
    P.offset += getTel().trim; reqZeroTrim = true;   // consolida el autoajuste
    saveParams(); o.printf("guardado (off = %.3f)\n", P.offset);
  }
  else if (!strcmp(cmd, "load"))     { state = ST_IDLE; delay(5); o.println(loadParams() ? "cargado (IDLE; usa start)" : "no hay datos"); }
  else if (!strcmp(cmd, "defaults")) { state = ST_IDLE; setDefaults(); o.println("valores por defecto (usa save para guardarlos)"); }
  else if (!strcmp(cmd, "reboot"))   { motorCoast(); delay(50); ESP.restart(); }
  else if (!strcmp(cmd, "inv") && a1) {
    state = ST_IDLE;
    float* f = !strcmp(a1, "motor") ? &P.invMotor : !strcmp(a1, "enc") ? &P.invEnc :
               !strcmp(a1, "imu") ? &P.invImu : !strcmp(a1, "gyro") ? &P.invGyro : nullptr;
    if (f) { *f = (*f > 0.5f) ? 0 : 1; o.printf("inv %s = %d (IDLE; usa start)\n", a1, (int)*f); }
    else o.println("uso: inv motor|enc|imu|gyro");
  }
  else if (!strcmp(cmd, "m") && a1) {
    if (state == ST_BAL) { o.println("primero stop"); return; }
    float u = constrain((float)atof(a1), -1.0f, 1.0f);
    float sec = a2 ? constrain((float)atof(a2), 0.05f, 10.0f) : 2.0f;
    testU = u; testUntilMs = millis() + (uint32_t)(sec * 1000);
    state = (u == 0) ? ST_IDLE : ST_MOTORTEST;
    o.printf("motor u=%.2f por %.2f s\n", u, sec);
  }
  else if (!strcmp(cmd, "dbfind")) {
    if (state == ST_BAL) { o.println("primero stop"); return; }
    state = ST_DBFIND; o.println("buscando zona muerta... (rampa lenta)");
  }
  else if (!strcmp(cmd, "tel")) {
    float hz = a1 ? atof(a1) : 20;
    telPeriodMs = hz > 0 ? (uint32_t)(1000.0f / constrain(hz, 1.0f, 100.0f)) : 0;
    telOut = &o;
  }
  else o.println("comando desconocido (help)");
}

struct LineBuf { char b[96]; uint8_t n = 0; };
LineBuf serBuf;
#if BT_OK
LineBuf btBuf;
#endif

void pollStream(Stream& s, LineBuf& lb) {
  while (s.available()) {
    char c = s.read();
    if (c == '\n' || c == '\r') {
      if (lb.n) { lb.b[lb.n] = 0; processLine(lb.b, s); lb.n = 0; }
    } else if (lb.n < sizeof(lb.b) - 1) lb.b[lb.n++] = c;
  }
}

void handleEvents() {
  if (evGyroCal == 1) { evGyroCal = 0; say("giroscopio calibrado -> ESPERA (pon el stick vertical)\n"); beep(80); }
  if (evGyroCal == 2) { evGyroCal = 0; say("se movió durante la calibración, repitiendo...\n"); }
  if (evCalDone)  { evCalDone = false; saveParams(); say("offset = %.3f deg guardado -> ESPERA\n", P.offset); beep(150); }
  if (evEngaged)  { evEngaged = false; say("equilibrando\n"); }
  if (evFell)     { evFell = false; say("caída detectada -> ESPERA\n"); }
  if (evRunaway)  { evRunaway = false; say("rueda saturada (>maxrps): sube K3 o revisa off\n"); beep(300); }
  if (evReinit)   { say(evReinit == 1 ? "sensores OK -> calibrando giroscopio\n" : "los sensores siguen sin responder\n"); evReinit = 0; }
  if (evFault)    { evFault = false; say("FALLA I2C: revisa cableado/ruido del motor. 'start' para reintentar\n"); beep(500); }
  if (evDbResult >= 0) {
    float r = evDbResult; evDbResult = -1;
    if (r > 0) { P.deadband = r * 0.9f; say("zona muerta medida %.3f -> db = %.3f (usa save)\n", r, P.deadband); }
    else say("la rueda no giró hasta 50%%: revisa motor/encoder\n");
  }
}

void updateLed() {
  if (PIN_LED < 0) return;
  uint32_t ms = millis(); bool on;
  switch (state) {
    case ST_BAL:   on = true; break;
    case ST_WAIT:  on = (ms / 150) & 1; break;
    case ST_FAULT: on = (ms / 50) & 1; break;
    default:       on = (ms / 600) & 1; break;
  }
  digitalWrite(PIN_LED, on);
}

void updateBattery() {
  static uint32_t last = 0, lastWarn = 0;
  if (PIN_VBAT < 0 || millis() - last < 50) return;
  last = millis();
  float v = analogReadMilliVolts(PIN_VBAT) * 0.001f * VBAT_DIVIDER;
  vbat = (vbat < 1.0f) ? v : vbat + 0.1f * (v - vbat);
  if (vbat > 5.0f && vbat < 3.5f * BAT_CELLS && millis() - lastWarn > 5000) {
    lastWarn = millis(); say("BATERIA BAJA: %.2f V\n", vbat); beep(400);
  }
}

// ---------------------------------------------------------------------------
//  SETUP / LOOP
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(PIN_AIN1, OUTPUT); pinMode(PIN_AIN2, OUTPUT);
  digitalWrite(PIN_AIN1, LOW); digitalWrite(PIN_AIN2, LOW);
  pwmInit(); pwmWrite(0);
  if (PIN_STBY >= 0)   { pinMode(PIN_STBY, OUTPUT); digitalWrite(PIN_STBY, HIGH); }
  if (PIN_LED >= 0)    pinMode(PIN_LED, OUTPUT);
  if (PIN_BUZZER >= 0) { pinMode(PIN_BUZZER, OUTPUT); digitalWrite(PIN_BUZZER, LOW); }
  if (PIN_VBAT >= 0)   analogReadResolution(12);   // atenuación por defecto 11 dB (0..~3.1 V)

  prefs.begin("stick", false);
  setDefaults();
  bool loaded = loadParams();

  Wire.begin(PIN_SDA, PIN_SCL, I2C_FREQ);
  Wire.setTimeOut(3);
  uint8_t who = 0;
  bool mpuOk = mpuInit(&who);
  bool encOk = as5600Init();

  Serial.printf("\n=== Stick autoequilibrio ESP32 ===\n");
  Serial.printf("parametros: %s\n", loaded ? "cargados de NVS" : "por defecto");
  Serial.printf("MPU6050: %s (WHO_AM_I=0x%02X)   AS5600: %s\n",
                mpuOk ? "OK" : "NO RESPONDE", who, encOk ? "OK" : "NO RESPONDE");
  if (mpuOk && who != 0x68) Serial.println("  aviso: WHO_AM_I distinto de 0x68 (clon), normalmente funciona");

#if BT_OK
  SerialBT.begin(BT_NAME);
  Serial.printf("Bluetooth SPP: \"%s\"\n", BT_NAME);
#endif

  if (!mpuOk || !encOk) state = ST_FAULT;
  else { state = ST_GYROCAL; Serial.println("calibrando giroscopio, no lo muevas (2 s)..."); }

  xTaskCreatePinnedToCore(controlTask, "ctrl", 4096, nullptr, 20, &ctrlHandle, 1);
  esp_timer_create_args_t ta = {};
  ta.callback = &onLoopTimer;
  ta.name = "loop";
  esp_timer_create(&ta, &loopTimer);
  esp_timer_start_periodic(loopTimer, LOOP_US);

  printHelp(Serial);
}

void loop() {
  pollStream(Serial, serBuf);
#if BT_OK
  pollStream(SerialBT, btBuf);
#endif
  handleEvents();
  updateBattery();
  updateLed();

  if (PIN_BUZZER >= 0 && buzzUntil && (int32_t)(millis() - buzzUntil) >= 0) {
    digitalWrite(PIN_BUZZER, LOW); buzzUntil = 0;
  }

  if (telPeriodMs && telOut && millis() - telLast >= telPeriodMs) {
    telLast = millis();
    Telem t = getTel();
    telOut->printf("th:%.2f,w:%.1f,ws:%.2f,wp:%.2f,u:%.3f,trim:%.2f\n",
                   t.theta, t.omega, t.ws, t.wp, t.u, t.trim);
  }
  vTaskDelay(1);
}
