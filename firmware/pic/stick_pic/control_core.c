/* ============================================================================
 *  control_core.c  —  Núcleo de control del stick de autoequilibrio
 *  Aritmética de PUNTO FIJO (enteros de 32 bits): sin flotantes en el lazo.
 *
 *  Es C portable: compila en CCS PIC C (PCH) y en gcc (banco de pruebas en
 *  /sim). No usa nada del hardware: el programa principal le entrega las
 *  lecturas crudas (in_acc, in_gyr, in_enc, in_ok) y lee la salida
 *  (out_mode, out_pwm) una vez por ciclo.
 *
 *  Misma ley de control y MISMAS UNIDADES DE GANANCIA que la versión ESP32:
 *      u[duty -1..1] = K1*theta[deg] + K2*omega[deg/s] + K3*vel_rueda[rps]
 *                    + K4*pos_rueda[rev]
 *
 *  Formatos internos (QN = valor * 2^N):
 *      thetaQ16  ángulo [deg]      Q16      omega, theta   Q8 [deg/s], [deg]
 *      wsQ12     vel. rueda [rps]  Q12      wsQ8, wpQ8     Q8 [rps], [rev]
 *      trimQ20   autoajuste [deg]  Q20      k1q..k4q       duty*PWM_MAX*16
 *      u = (k1q*theta + k2q*omega + k3q*ws + k4q*wp) >> 12   [cuentas PWM]
 * ==========================================================================*/
#ifndef CONTROL_CORE_C
#define CONTROL_CORE_C

#if defined(__PCH__) || defined(__PCM__)
  typedef signed int8    s8;
  typedef unsigned int8  u8;
  typedef signed int16   s16;
  typedef unsigned int16 u16;
  typedef signed int32   s32;
  typedef unsigned int32 u32;
#else
  #include <stdint.h>
  #include <math.h>
  typedef int8_t   s8;
  typedef uint8_t  u8;
  typedef int16_t  s16;
  typedef uint16_t u16;
  typedef int32_t  s32;
  typedef uint32_t u32;
#endif

/* ---------- Configuración (se puede definir antes del #include) ---------- */
#ifndef LOOP_HZ
  #define LOOP_HZ 500          /* frecuencia del lazo [Hz] */
#endif
#ifndef PWM_MAX
  #define PWM_MAX 800          /* cuenta de PWM al 100 % */
#endif
#ifndef AX_TILT
  #define AX_TILT 1            /* eje acel. horizontal en el plano de caída (0=X 1=Y 2=Z) */
#endif
#ifndef AX_UP
  #define AX_UP   0            /* eje acel. que apunta hacia ARRIBA con el stick vertical */
#endif
#ifndef UP_SIGN
  #define UP_SIGN 1            /* -1 si ese eje apunta hacia abajo */
#endif
#ifndef AX_GYRO
  #define AX_GYRO 2            /* eje del giroscopio paralelo al eje de la rueda */
#endif

#define UPRIGHT_TICKS  (LOOP_HZ / 5)   /* 200 ms vertical antes de enganchar */

/* ---------- Estados ---------- */
#define ST_IDLE      0
#define ST_WAIT      1
#define ST_BAL       2
#define ST_GYROCAL   3
#define ST_CAL       4
#define ST_MOTORTEST 5
#define ST_DBFIND    6
#define ST_FAULT     7

#define MODE_COAST 0
#define MODE_DRIVE 1
#define MODE_BRAKE 2

/* ---------- Parámetros de usuario (flotantes, solo fuera del lazo) ---------- */
typedef struct {
  float K1, K2, K3, K4;     /* ganancias */
  float offset;             /* ángulo de equilibrio [deg] */
  float tau;                /* filtro complementario [s] */
  float wsLpfHz;            /* filtro vel. rueda [Hz] */
  float gyroLpfHz;          /* filtro término D [Hz] (0 = off) */
  float deadband;           /* zona muerta del motor [0..0.5] */
  float maxAngle;           /* ángulo de caída [deg] */
  float startAngle;         /* ángulo para enganchar [deg] */
  float trimK;              /* autoajuste offset [deg/(rps*s)] */
  float maxRps;             /* vel. máx. rueda [rps] */
  float vNom;               /* voltaje nominal [V] */
  float vComp;              /* 1 = compensar batería */
  float fallBrake;          /* 1 = frenar al caer */
  float invMotor, invEnc, invImu, invGyro;
} UserParams;

UserParams P;

/* ---------- Parámetros internos (enteros) ---------- */
s32 k1q, k2q, k3q, k4q;
s32 offsetQ16, compK, wsAlpha, gAlpha;
s32 dbCnt, dbScaleQ12, dbRampCnt;
s32 maxAngQ8, startAngQ8, maxRpsQ8;
s32 trimKq, gyroIncQ6, rampStepQ8, dbStepQ8, vNomMv;
s32 vcompQ12 = 4096;
s8  sImu = 1, sGyr = 1, invMot = 0, invEncF = 0, brakeOnFall = 1;

/* ---------- Entradas / salidas ---------- */
s16 in_acc[3], in_gyr[3];
u16 in_enc;
u8  in_ok;
u8  out_mode = MODE_COAST;
s32 out_pwm = 0;

/* ---------- Estado / peticiones / eventos ---------- */
u8  core_state = ST_GYROCAL;
u8  req_cal_finish = 0, req_zero_trim = 0;
u8  ev_gyrocal = 0, ev_caldone = 0, ev_engaged = 0, ev_fell = 0, ev_runaway = 0, ev_fault = 0;
s32 ev_db = -1;
s32 test_pwm = 0, test_ticks = 0;

/* ---------- Telemetría ---------- */
s32 tl_theta, tl_omega, tl_ws, tl_wp, tl_u, tl_trimQ16, tl_thetaRawQ16;

/* ---------- Variables internas del lazo ---------- */
s32 thetaQ16 = 0, omegaF = 0, wsQ12 = 0, trimQ20 = 0, rampQ8 = 0, biasQ4 = 0;
s32 encCount = 0, encZero = 0, upright = 0;
s32 gcSum, gcN, gcMin, gcMax, calSum, calN, dbUQ8;
u16 encPrev = 0;
u8  angleInit = 0, encInit = 0, errRun = 0, prevState = 255;

/* ========================= utilidades ========================= */
s32 asr(s32 x, u8 n) {               /* desplazamiento aritmético portable */
  if (x < 0) return -((-x) >> n);
  return x >> n;
}
s32 abs32(s32 x) { return (x < 0) ? -x : x; }
s32 clamp32(s32 x, s32 lo, s32 hi) { if (x < lo) return lo; if (x > hi) return hi; return x; }
s32 f2l(float f) { if (f >= 0) return (s32)(f + 0.5); return (s32)(f - 0.5); }

/* ===================== parámetros ===================== */
void params_default(void) {
  P.K1 = 0.45;  P.K2 = 0.045;  P.K3 = 0.08;  P.K4 = 0.015;
  P.offset = 0.0;
  P.tau = 0.5;  P.wsLpfHz = 25.0;  P.gyroLpfHz = 0.0;
  P.deadband = 0.05;
  P.maxAngle = 12.0;  P.startAngle = 0.5;
  P.trimK = 0.02;  P.maxRps = 80.0;
  P.vNom = 11.1;  P.vComp = 1;  P.fallBrake = 1;
  P.invMotor = 0;  P.invEnc = 0;  P.invImu = 0;  P.invGyro = 0;
}

/* Convierte los flotantes de usuario a enteros. Llamar tras cualquier cambio. */
void params_apply(void) {
  float f;
  k1q = f2l(P.K1 * (PWM_MAX * 16.0));
  k2q = f2l(P.K2 * (PWM_MAX * 16.0));
  k3q = f2l(P.K3 * (PWM_MAX * 16.0));
  k4q = f2l(P.K4 * (PWM_MAX * 16.0));
  offsetQ16 = f2l(P.offset * 65536.0);
  compK = f2l(65536.0 / (P.tau * LOOP_HZ));
  f = 1.0 - exp(-6.2831853 * P.wsLpfHz / LOOP_HZ);
  wsAlpha = f2l(f * 1024.0);                         /* Q10 */
  if (wsAlpha < 1) wsAlpha = 1;
  if (P.gyroLpfHz > 0.0) {
    f = 1.0 - exp(-6.2831853 * P.gyroLpfHz / LOOP_HZ);
    gAlpha = f2l(f * 4096.0);                        /* Q12 */
    if (gAlpha < 1) gAlpha = 1;
  } else gAlpha = 0;
  dbCnt = f2l(P.deadband * PWM_MAX);
  dbScaleQ12 = f2l((1.0 - P.deadband) * 4096.0);
  dbRampCnt = PWM_MAX / 50;  if (dbRampCnt < 1) dbRampCnt = 1;
  maxAngQ8   = f2l(P.maxAngle * 256.0);
  startAngQ8 = f2l(P.startAngle * 256.0);
  maxRpsQ8   = f2l(P.maxRps * 256.0);
  trimKq     = f2l(P.trimK * 1048576.0 / LOOP_HZ);
  gyroIncQ6  = f2l(4194304.0 / (65.5 * LOOP_HZ));    /* 2^16*2^6/(65.5*F) */
  rampStepQ8 = f2l(256.0 / (0.15 * LOOP_HZ));  if (rampStepQ8 < 1) rampStepQ8 = 1;
  dbStepQ8   = f2l(PWM_MAX * 0.05 * 256.0 / LOOP_HZ);  if (dbStepQ8 < 1) dbStepQ8 = 1;
  vNomMv     = f2l(P.vNom * 1000.0);
  sImu = (P.invImu > 0.5) ? -1 : 1;
  sGyr = (P.invGyro > 0.5) ? -sImu : sImu;
  invMot = (P.invMotor > 0.5) ? 1 : 0;
  invEncF = (P.invEnc > 0.5) ? 1 : 0;
  brakeOnFall = (P.fallBrake > 0.5) ? 1 : 0;
}

/* Voltaje de batería medido [mV] -> factor de compensación (llamar ~20 Hz). */
void core_set_vbat_mv(s32 mv) {
  if (P.vComp > 0.5 && mv > 5000) vcompQ12 = clamp32((vNomMv * 4096) / mv, 2048, 8192);
  else vcompQ12 = 4096;
}

/* ===================== salida al motor ===================== */
void out_coast(void) { out_mode = MODE_COAST; out_pwm = 0; }
void out_stop_fall(void) { if (brakeOnFall) { out_mode = MODE_BRAKE; out_pwm = 0; } else out_coast(); }
void out_drive(s32 v) {
  v = clamp32(v, -PWM_MAX, PWM_MAX);
  if (invMot) v = -v;
  out_pwm = v;
  out_mode = (v == 0) ? MODE_COAST : MODE_DRIVE;
}

/* Zona muerta suave + compensación de batería */
s32 shape_output(s32 u) {
  s32 a, o;
  u = clamp32(u, -PWM_MAX, PWM_MAX);
  a = abs32(u);
  if (a == 0) return 0;
  o = asr(a * dbScaleQ12, 12);
  if (a >= dbRampCnt) o += dbCnt;
  else o += (dbCnt * a) / dbRampCnt;
  o = asr(o * vcompQ12, 12);
  if (o > PWM_MAX) o = PWM_MAX;
  return (u < 0) ? -o : o;
}

s32 clamp_term(s32 t) { return clamp32(t, -(s32)PWM_MAX * 16384, (s32)PWM_MAX * 16384); }

/* ===================== paso de control (1 por ciclo) ===================== */
void control_step(void) {
  s32 gQ4, omega, accQ16, at, au, xq, x2, den, angq12;
  s32 ax, ay, az, n2, d, wsRaw, wsQ8, wpQ8, theta, u, sum, t;
  u8 st;

  if (!in_ok) {                                   /* error de I2C */
    if (errRun < 255) errRun++;
    if (errRun >= 3) out_coast();                 /* 1-2 ciclos: mantiene salida */
    if (errRun >= 10 && core_state != ST_FAULT) { core_state = ST_FAULT; ev_fault = 1; }
    return;
  }
  errRun = 0;

  /* ---- 1. Giroscopio: sesgo en Q4 para precisión sub-LSB ---- */
  gQ4 = (s32)in_gyr[AX_GYRO] * 16 - biasQ4;
  if (sGyr < 0) gQ4 = -gQ4;
  omega = clamp32(asr(gQ4 * 250, 10), -128000, 128000);   /* Q8 deg/s (±500) */
  if (gAlpha) omegaF += asr((omega - omegaF) * gAlpha, 12);
  else omegaF = omega;

  /* ---- 2. Ángulo del acelerómetro: atan(x) ~ x/(1+0.28x^2) ---- */
  at = (s32)in_acc[AX_TILT];
  au = (s32)in_acc[AX_UP] * UP_SIGN;
  if (au < 2000) accQ16 = (at >= 0) ? 2949120 : -2949120;   /* acostado: ±45° */
  else {
    xq = clamp32((at * 4096) / au, -4096, 4096);             /* Q12 */
    x2 = asr(xq * xq, 12);
    den = 4096 + asr(x2 * 1147, 12);
    angq12 = (xq * 4096) / den;                              /* rad Q12 */
    accQ16 = angq12 * 917;                                   /* deg Q16 */
  }
  if (sImu < 0) accQ16 = -accQ16;

  /* ---- 3. Filtro complementario con compuerta |a| ~ 1 g ---- */
  ax = asr((s32)in_acc[0], 4);  ay = asr((s32)in_acc[1], 4);  az = asr((s32)in_acc[2], 4);
  n2 = ax * ax + ay * ay + az * az;                          /* 1 g = 1048576 */
  if (!angleInit) { thetaQ16 = accQ16; angleInit = 1; }
  else {
    thetaQ16 += asr(gQ4 * gyroIncQ6, 10);
    if (n2 > 757000 && n2 < 1387000) {
      d = clamp32(accQ16 - thetaQ16, -131072, 131072);
      thetaQ16 += asr(d * compK, 16);
    }
  }

  /* ---- 4. Rueda: multivuelta + velocidad filtrada ---- */
  if (!encInit) { encPrev = in_enc; encInit = 1; }
  d = (s32)in_enc - (s32)encPrev;
  encPrev = in_enc;
  if (d > 2048) d -= 4096; else if (d < -2048) d += 4096;
  if (invEncF) d = -d;
  encCount += d;
  wsRaw = d * LOOP_HZ;                                       /* rps Q12 exacto */
  wsQ12 += asr(clamp32(wsRaw - wsQ12, -2000000, 2000000) * wsAlpha, 10);
  wsQ8 = asr(wsQ12, 4);
  wpQ8 = clamp32(asr(encCount - encZero, 4), -50000, 50000);

  if (req_zero_trim) { trimQ20 = 0; req_zero_trim = 0; }
  theta = asr(thetaQ16 - offsetQ16 - asr(trimQ20, 4), 8);   /* Q8 deg */
  u = 0;

  /* ---- 5. Máquina de estados ---- */
  st = core_state;
  if (st != prevState) {                                     /* acciones de entrada */
    if (st == ST_GYROCAL) { gcSum = 0; gcN = 0; gcMin = 32767; gcMax = -32768; }
    if (st == ST_CAL)     { calSum = 0; calN = 0; }
    if (st == ST_DBFIND)  dbUQ8 = 0;
    if (st == ST_WAIT)    upright = 0;
    prevState = st;
  }

  switch (st) {
    case ST_GYROCAL:
      out_coast();
      t = (s32)in_gyr[AX_GYRO];
      gcSum += t;  gcN++;
      if (t < gcMin) gcMin = t;
      if (t > gcMax) gcMax = t;
      if (gcN >= 512) {
        if (gcMax - gcMin < 200) {                           /* quieto: < ~3 °/s p-p */
          biasQ4 = asr(gcSum, 5);                            /* media*16 */
          angleInit = 0;  ev_gyrocal = 1;  core_state = ST_WAIT;
        } else {
          ev_gyrocal = 2;  gcSum = 0;  gcN = 0;  gcMin = 32767;  gcMax = -32768;
        }
      }
      break;

    case ST_CAL:
      out_coast();
      if (calN < 4096) { calSum += asr(thetaQ16, 4); calN++; }
      if (req_cal_finish) {
        req_cal_finish = 0;
        if (calN > 0) { offsetQ16 = (calSum / calN) * 16; P.offset = offsetQ16 / 65536.0; }
        trimQ20 = 0;  ev_caldone = 1;  core_state = ST_WAIT;
      }
      break;

    case ST_WAIT:
      out_stop_fall();
      if (abs32(theta) < startAngQ8) {
        if (++upright >= UPRIGHT_TICKS) {
          encZero = encCount;  rampQ8 = 0;  omegaF = omega;
          core_state = ST_BAL;  ev_engaged = 1;
        }
      } else upright = 0;
      break;

    case ST_BAL:
      if (abs32(theta) > maxAngQ8 || abs32(wsQ8) > maxRpsQ8) {
        if (abs32(wsQ8) > maxRpsQ8) ev_runaway = 1; else ev_fell = 1;
        out_stop_fall();  core_state = ST_WAIT;
        break;
      }
      sum  = clamp_term(k1q * theta);
      sum += clamp_term(k2q * omegaF);
      sum += clamp_term(k3q * wsQ8);
      sum += clamp_term(k4q * wpQ8);
      u = asr(sum, 12);

      if (trimKq) {                                          /* autoajuste del offset */
        trimQ20 -= asr(trimKq * (wsQ8 + asr(wpQ8, 1)), 8);   /* vel + pos/2 */
        trimQ20 = clamp32(trimQ20, -3145728, 3145728);       /* ±3° */
      }
      rampQ8 += rampStepQ8;  if (rampQ8 > 256) rampQ8 = 256;  /* arranque suave */
      u = asr(u * rampQ8, 8);
      out_drive(shape_output(u));
      break;

    case ST_MOTORTEST:
      if (test_ticks > 0) { test_ticks--;  u = test_pwm;  out_drive(u); }
      else { out_coast();  core_state = ST_IDLE; }
      break;

    case ST_DBFIND:                                          /* rampa 5 %/s */
      dbUQ8 += dbStepQ8;
      u = asr(dbUQ8, 8);
      out_drive(u);
      if (abs32(wsQ8) > 77)        { ev_db = u;  out_coast();  core_state = ST_IDLE; }  /* 0.3 rps */
      else if (u > PWM_MAX / 2)    { ev_db = 0;  out_coast();  core_state = ST_IDLE; }
      break;

    default:                                                 /* IDLE, FAULT */
      out_coast();
      break;
  }

  /* ---- 6. Telemetría ---- */
  tl_theta = theta;  tl_omega = omegaF;  tl_ws = wsQ8;  tl_wp = wpQ8;  tl_u = u;
  tl_trimQ16 = asr(trimQ20, 4);  tl_thetaRawQ16 = thetaQ16;
}

/* Reinicia filtros y contadores (tras reinicializar sensores) */
void core_reset_filters(void) { angleInit = 0; encInit = 0; errRun = 0; wsQ12 = 0; }

#endif
