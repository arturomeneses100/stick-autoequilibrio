/* ============================================================================
 *  Stick de autoequilibrio de un eje (rueda de reacción, motor DC) — PIC18
 *  Compilador: CCS PIC C (PCH)            Autor: Arturo Elfego Meneses Flores
 *  Basado en la idea de remrc/One-Axis-Self-Balancing-Stick-DC
 *
 *  Microcontroladores soportados (mismo pinout, 40 pines):
 *    - PIC18F45K22  @ 64 MHz (oscilador interno, sin cristal)  <- recomendado
 *    - PIC18F4550   @ 48 MHz (cristal de 20 MHz + PLL)
 *
 *  Lazo de control a 500 Hz disparado por Timer2 (el mismo que genera el PWM
 *  de 20 kHz). Todo el control es en punto fijo (control_core.c), así que
 *  corre en ~1 ms incluyendo la lectura I2C de MPU6050 + AS5600.
 *
 *  CONEXIONES (PIC a 5 V)
 *    RC2/CCP1 -> PWMA (TB6612)     RD0 -> AIN1     RD1 -> AIN2     RD2 -> STBY
 *    RD3 -> LED de estado          RD4 -> buzzer activo (opcional)
 *    RC6/TX -> RX del HC-05        RC7/RX <- TX del HC-05 (o adaptador USB-TTL)
 *    I2C  PIC18F45K22: SDA=RC4  SCL=RC3      PIC18F4550: SDA=RB0  SCL=RB1
 *    Divisor batería: Vbat -- 100k --+-- 47k -- GND ; punto medio -> RA0/AN0
 *
 *    I2C a 3.3 V: los módulos GY-521 y AS5600 traen pull-ups a 3.3 V. El
 *    firmware activa los niveles SMBus del MSSP (VIH = 2.1 V), así el PIC a
 *    5 V lee bien el bus de 3.3 V. NO pongas pull-ups a 5 V en SDA/SCL.
 *
 *  Comandos por serie (9600 baud por defecto, igual que el HC-05): "help".
 *  Las ganancias tienen las MISMAS unidades que la versión ESP32.
 * ==========================================================================*/

/* ======================= SELECCIÓN DE MICRO ======================= */
#define USE_PIC18F45K22
//#define USE_PIC18F4550

#if defined(USE_PIC18F45K22)
  #include <18F45K22.h>
  #device ADC=10
  #device PASS_STRINGS=IN_RAM
  #fuses NOWDT, NOLVP, NOPBADEN, PUT, BROWNOUT, MCLR, NOPROTECT, NOXINST
  #use delay(internal=64MHz)
  #define PWM_MAX   800                 /* 4*(PR2+1) */
  #define T2_DIV    T2_DIV_BY_4         /* 64e6/(4*4*200) = 20 kHz */
  #define T2_PR     199
  #bit SSP_CKE = getenv("SFR:SSP1STAT").6
#elif defined(USE_PIC18F4550)
  #include <18F4550.h>
  #device ADC=10
  #device PASS_STRINGS=IN_RAM
  #fuses HSPLL, PLL5, CPUDIV1, NOWDT, NOLVP, NOPBADEN, PUT, BROWNOUT, MCLR, NOPROTECT, NOXINST, NOVREGEN
  #use delay(clock=48MHz)
  #define PWM_MAX   600                 /* 48e6/(4*4*150) = 20 kHz */
  #define T2_DIV    T2_DIV_BY_4
  #define T2_PR     149
  #bit SSP_CKE = getenv("SFR:SSPSTAT").6
#else
  #error "Selecciona USE_PIC18F45K22 o USE_PIC18F4550"
#endif

/* ======================= CONFIGURACIÓN ======================= */
#define BAUD        9600       /* HC-05 de fábrica. Recomendado: 115200 (AT+UART) */
#define LOOP_HZ     500        /* 500, 400, 250, 200 o 100 (divisor de 2 kHz) */
#define T2_PER_LOOP (2000 / LOOP_HZ)
#define USE_VBAT    1          /* 0 si no conectas el divisor de batería */
#define BAT_CELLS   3
#define VBAT_MV_NUM 15638      /* 5000 mV * (100k+47k)/47k */

/* Orientación del MPU6050 (ver comando "imu") */
#define AX_TILT 1
#define AX_UP   0
#define UP_SIGN 1
#define AX_GYRO 2

#define PIN_AIN1   PIN_D0
#define PIN_AIN2   PIN_D1
#define PIN_STBY   PIN_D2
#define PIN_LED    PIN_D3
#define PIN_BUZZER PIN_D4

#use i2c(MASTER, I2C1, FAST=400000, FORCE_HW)
#use rs232(baud=BAUD, UART1, RECEIVE_BUFFER=32, TRANSMIT_BUFFER=128, TXISR, ERRORS)

#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>

#include "control_core.c"

#define MPU_ADDR    0x68
#define AS5600_ADDR 0x36
#define PARAM_VER   0x04

/* ======================= variables del sistema ======================= */
volatile u8 tick_pending = 0;
u8  t2div = 0;
u32 loops = 0;
u32 stat_i2c_err = 0, stat_overruns = 0;
s32 vbat_mv = 0;
u8  req_reinit = 0;
u16 tel_div = 0;
u32 buzz_until = 0;
char line[40];
u8  line_len = 0;
char delim[4] = " =\t";

/* ======================= interrupción del lazo ======================= */
#INT_TIMER2
void isr_timer2(void) {                 /* 20 kHz / postescala 10 = 2 kHz */
  if (++t2div >= T2_PER_LOOP) { t2div = 0; tick_pending++; }
}

/* ======================= I2C ======================= */
int1 i2c_wr(u8 addr, u8 reg, u8 val) {
  u8 ack;
  i2c_start();
  ack = i2c_write(addr << 1);
  if (ack == 0) { i2c_write(reg); i2c_write(val); }
  i2c_stop();
  return (ack == 0);
}

int1 i2c_rd(u8 addr, u8 reg, u8 *buf, u8 n) {
  u8 i;
  i2c_start();
  if (i2c_write(addr << 1) != 0) { i2c_stop(); return 0; }
  i2c_write(reg);
  i2c_start();                                   /* reinicio */
  if (i2c_write((addr << 1) | 1) != 0) { i2c_stop(); return 0; }
  for (i = 0; i < n; i++) buf[i] = i2c_read(i < (n - 1));   /* ACK salvo el último */
  i2c_stop();
  return 1;
}

int1 mpu_init(u8 *who) {
  int1 ok;
  if (!i2c_wr(MPU_ADDR, 0x6B, 0x80)) return 0;  /* reset */
  delay_ms(100);
  ok = i2c_wr(MPU_ADDR, 0x6B, 0x01);            /* reloj PLL */
  ok &= i2c_wr(MPU_ADDR, 0x19, 0x00);           /* 1 kHz */
  ok &= i2c_wr(MPU_ADDR, 0x1A, 0x02);           /* DLPF 98 Hz */
  ok &= i2c_wr(MPU_ADDR, 0x1B, 0x08);           /* ±500 °/s */
  ok &= i2c_wr(MPU_ADDR, 0x1C, 0x00);           /* ±2 g */
  i2c_rd(MPU_ADDR, 0x75, who, 1);
  return ok;
}

int1 as5600_init(void) {
  u8 c[2];
  if (!i2c_rd(AS5600_ADDR, 0x07, c, 2)) return 0;
  c[0] = (c[0] & 0xC0) | 0x01;                  /* filtro 8x, FTH=0, WD=0 */
  c[1] &= 0xF3;                                 /* histéresis OFF */
  i2c_start();
  if (i2c_write(AS5600_ADDR << 1) != 0) { i2c_stop(); return 0; }
  i2c_write(0x07); i2c_write(c[0]); i2c_write(c[1]);
  i2c_stop();
  return 1;
}

int1 sensors_read(void) {
  u8 b[14], i;
  if (!i2c_rd(MPU_ADDR, 0x3B, b, 14)) return 0;
  for (i = 0; i < 3; i++) {
    in_acc[i] = (s16)make16(b[2 * i], b[2 * i + 1]);
    in_gyr[i] = (s16)make16(b[8 + 2 * i], b[9 + 2 * i]);
  }
  if (!i2c_rd(AS5600_ADDR, 0x0C, b, 2)) return 0;
  in_enc = make16(b[0] & 0x0F, b[1]);
  return 1;
}

int1 sensors_init(int1 verbose);

/* ======================= motor (TB6612) ======================= */
void motor_apply(void) {
  s32 v = out_pwm;
  if (out_mode == MODE_BRAKE) {                 /* freno: ambos en alto */
    output_high(PIN_AIN1); output_high(PIN_AIN2);
    set_pwm1_duty((int16)PWM_MAX);
    return;
  }
  if (out_mode == MODE_COAST || v == 0) {       /* rueda libre */
    output_low(PIN_AIN1); output_low(PIN_AIN2);
    set_pwm1_duty((int16)0);
    return;
  }
  if (v > 0) { output_high(PIN_AIN1); output_low(PIN_AIN2); }
  else       { output_low(PIN_AIN1); output_high(PIN_AIN2); v = -v; }
  set_pwm1_duty((int16)v);
}

/* ======================= salida de texto (sin printf de flotantes) ======================= */
void put_u32(u32 x) {
  char b[11];
  u8 n = 0;
  do { b[n++] = '0' + (char)(x % 10); x /= 10; } while (x);
  while (n) putc(b[--n]);
}
void put_fx(s32 v, u8 dec) {                    /* v ya escalado por 10^dec */
  u32 x, p = 1;
  u8 i;
  if (v < 0) { putc('-'); x = (u32)(-v); } else x = (u32)v;
  for (i = 0; i < dec; i++) p *= 10;
  put_u32(x / p);
  if (dec) {
    putc('.');
    x %= p;  p /= 10;
    while (p) { putc('0' + (char)(x / p)); x %= p; p /= 10; }
  }
}
void put_float(float f, u8 dec) {
  float m = 1;
  u8 i;
  for (i = 0; i < dec; i++) m *= 10;
  put_fx(f2l(f * m), dec);
}
void put_q8(s32 q, u8 dec) {                    /* valor Q8 con 'dec' decimales */
  s32 m = 1;
  u8 i;
  for (i = 0; i < dec; i++) m *= 10;
  put_fx((q * m) / 256, dec);
}
void nl(void) { putc('\r'); putc('\n'); }
void put_str(char *s) { while (*s) putc(*s++); }

void beep(u16 ms) {
  output_high(PIN_BUZZER);
  buzz_until = loops + ((u32)ms * LOOP_HZ) / 1000;
}

/* ======================= tabla de parámetros ======================= */
#define NPARAM 16
const float PLO[NPARAM]   = {0, 0, 0, 0, -45, 0.1, 1, 0, 0, 2, 0.1, 0, 5, 3, 0, 0};
const float PHI[NPARAM]   = {5, 0.5, 1, 0.5, 45, 5, 200, 200, 0.5, 45, 10, 1, 150, 30, 1, 1};
const float PSTEP[NPARAM] = {0.01, 0.001, 0.002, 0.0005, 0.05, 0.05, 1, 1, 0.005, 0.5, 0.1, 0.005, 5, 0.1, 1, 1};

float *pptr(u8 i) {
  switch (i) {
    case 0:  return &P.K1;          case 1:  return &P.K2;
    case 2:  return &P.K3;          case 3:  return &P.K4;
    case 4:  return &P.offset;      case 5:  return &P.tau;
    case 6:  return &P.wsLpfHz;     case 7:  return &P.gyroLpfHz;
    case 8:  return &P.deadband;    case 9:  return &P.maxAngle;
    case 10: return &P.startAngle;  case 11: return &P.trimK;
    case 12: return &P.maxRps;      case 13: return &P.vNom;
    case 14: return &P.vComp;       default: return &P.fallBrake;
  }
}

s8 pfind(char *s) {
  if (!strcmp(s, "k1")) return 0;        if (!strcmp(s, "k2")) return 1;
  if (!strcmp(s, "k3")) return 2;        if (!strcmp(s, "k4")) return 3;
  if (!strcmp(s, "off")) return 4;       if (!strcmp(s, "tau")) return 5;
  if (!strcmp(s, "wlpf")) return 6;      if (!strcmp(s, "glpf")) return 7;
  if (!strcmp(s, "db")) return 8;        if (!strcmp(s, "maxang")) return 9;
  if (!strcmp(s, "startang")) return 10; if (!strcmp(s, "trim")) return 11;
  if (!strcmp(s, "maxrps")) return 12;   if (!strcmp(s, "vnom")) return 13;
  if (!strcmp(s, "vcomp")) return 14;    if (!strcmp(s, "brake")) return 15;
  return -1;
}

void pname(u8 i) {
  switch (i) {
    case 0: printf("k1"); break;        case 1: printf("k2"); break;
    case 2: printf("k3"); break;        case 3: printf("k4"); break;
    case 4: printf("off"); break;       case 5: printf("tau"); break;
    case 6: printf("wlpf"); break;      case 7: printf("glpf"); break;
    case 8: printf("db"); break;        case 9: printf("maxang"); break;
    case 10: printf("startang"); break; case 11: printf("trim"); break;
    case 12: printf("maxrps"); break;   case 13: printf("vnom"); break;
    case 14: printf("vcomp"); break;    default: printf("brake"); break;
  }
}

void params_clamp(void) {
  u8 i;
  float *v;
  for (i = 0; i < NPARAM; i++) {
    v = pptr(i);
    if (!(*v >= PLO[i])) *v = PLO[i];           /* también atrapa basura/NaN */
    if (*v > PHI[i]) *v = PHI[i];
  }
  if (P.invMotor != 1) P.invMotor = 0;
  if (P.invEnc != 1)   P.invEnc = 0;
  if (P.invImu != 1)   P.invImu = 0;
  if (P.invGyro != 1)  P.invGyro = 0;
}

/* ======================= EEPROM ======================= */
void params_save(void) {
  u8 i, b;
  u8 *p = (u8 *)&P;
  write_eeprom(0, 0xA5);
  write_eeprom(1, PARAM_VER);
  for (i = 0; i < sizeof(UserParams); i++) {
    b = p[i];
    if (read_eeprom(2 + i) != b) write_eeprom(2 + i, b);   /* solo lo que cambió */
  }
}

int1 params_load(void) {
  u8 i;
  u8 *p = (u8 *)&P;
  if (read_eeprom(0) != 0xA5 || read_eeprom(1) != PARAM_VER) return 0;
  for (i = 0; i < sizeof(UserParams); i++) p[i] = read_eeprom(2 + i);
  return 1;
}

/* ======================= impresión ======================= */
void print_state(void) {
  switch (core_state) {
    case ST_IDLE: printf("IDLE"); break;            case ST_WAIT: printf("ESPERA"); break;
    case ST_BAL: printf("EQUILIBRANDO"); break;     case ST_GYROCAL: printf("CAL_GIRO"); break;
    case ST_CAL: printf("CAL_OFFSET"); break;       case ST_MOTORTEST: printf("PRUEBA_MOTOR"); break;
    case ST_DBFIND: printf("DBFIND"); break;        default: printf("FALLA"); break;
  }
}

void print_params(void) {
  u8 i;
  printf("--- Parametros ---\r\n");
  for (i = 0; i < NPARAM; i++) { pname(i); printf(" = "); put_float(*pptr(i), 5); nl(); }
  printf("inv motor="); put_u32((u32)P.invMotor); printf(" enc="); put_u32((u32)P.invEnc);
  printf(" imu="); put_u32((u32)P.invImu); printf(" gyro="); put_u32((u32)P.invGyro); nl();
  printf("estado="); print_state(); printf("  vbat="); put_fx(vbat_mv / 10, 2); printf(" V"); nl();
}

void print_help(void) {
  printf("--- Comandos ---\r\n");
  printf(" get | help | stat | imu\r\n");
  printf(" <param> <valor>  ej: k1 0.5   (k1 k2 k3 k4 off tau wlpf glpf db\r\n");
  printf("                   maxang startang trim maxrps vnom vcomp brake)\r\n");
  printf(" <param>+ / <param>-  +/-5%%    p+ i+ a+ s+ (estilo original)\r\n");
  printf(" start | stop     habilita / deshabilita\r\n");
  printf(" c+ ... c-        calibra punto de equilibrio (guarda)\r\n");
  printf(" zero             offset = angulo actual\r\n");
  printf(" inv motor|enc|imu|gyro\r\n");
  printf(" m <u> [seg]      prueba de motor, u en -1..1\r\n");
  printf(" dbfind           mide zona muerta (stick acostado)\r\n");
  printf(" gyrocal          recalibra giroscopio (quieto)\r\n");
  printf(" tel <Hz>         telemetria (0 = off)\r\n");
  printf(" save | load | defaults | reboot\r\n");
}

void print_stat(void) {
  u8 s = 0, agc = 0;
  i2c_rd(AS5600_ADDR, 0x0B, &s, 1);
  i2c_rd(AS5600_ADDR, 0x1A, &agc, 1);
  printf("lazo "); put_u32(LOOP_HZ); printf(" Hz, sobrecargas "); put_u32(stat_overruns);
  printf(", errores I2C "); put_u32(stat_i2c_err); nl();
  printf("iman AS5600: ");
  if (s & 0x20) printf("detectado "); else printf("NO DETECTADO ");
  if (s & 0x10) printf("(debil) ");
  if (s & 0x08) printf("(fuerte) ");
  printf("AGC="); put_u32(agc); printf("  vbat="); put_fx(vbat_mv / 10, 2); printf(" V"); nl();
  stat_overruns = 0;
}

void print_imu(void) {
  u8 i;
  printf("acc[g]");
  for (i = 0; i < 3; i++) { printf(" "); put_fx(((s32)in_acc[i] * 100) / 16384, 2); }
  printf("  gyro[raw]");
  for (i = 0; i < 3; i++) { printf(" "); put_fx((s32)in_gyr[i], 0); }
  nl();
  printf("theta="); put_q8(tl_theta, 2); printf(" crudo="); put_fx(tl_thetaRawQ16 / 655, 2);
  printf("  (en vertical AX_UP debe leer ~+1 g)"); nl();
}

void print_gains(void) {
  printf("K1="); put_float(P.K1, 4); printf(" K2="); put_float(P.K2, 4);
  printf(" K3="); put_float(P.K3, 4); printf(" K4="); put_float(P.K4, 4); nl();
}

/* ======================= comandos ======================= */
void step_param(u8 i, s8 sg) {
  float *v = pptr(i), s;
  s = fabs(*v) * 0.05;
  if (s < PSTEP[i]) s = PSTEP[i];
  if (sg > 0) *v += s; else *v -= s;
  if (*v < PLO[i]) *v = PLO[i];
  if (*v > PHI[i]) *v = PHI[i];
}

int1 is_legacy(char *s) {
  u8 n = strlen(s), i;
  if (n < 2 || (n & 1)) return 0;
  for (i = 0; i < n; i += 2) {
    if (s[i] != 'p' && s[i] != 'i' && s[i] != 'a' && s[i] != 's' && s[i] != 'c') return 0;
    if (s[i + 1] != '+' && s[i + 1] != '-') return 0;
  }
  return 1;
}

int1 busy_balancing(void) {
  if (core_state == ST_BAL) { printf("primero stop\r\n"); return 1; }
  return 0;
}

void start_cal(void) { core_state = ST_CAL; printf("CAL: sosten el stick en su punto de equilibrio y envia c-\r\n"); }
void finish_cal(void) {
  if (core_state != ST_CAL) { printf("primero envia c+\r\n"); return; }
  req_cal_finish = 1;
}

void process_line(void) {
  char *cmd, *a1, *a2;
  u8 n, i, cl;
  s8 k, sg;
  float u, sec, hz;
  float *f;

  n = strlen(line);
  for (i = 0; i < n; i++) line[i] = tolower(line[i]);
  while (n && line[n - 1] == ' ') line[--n] = 0;
  if (!n) return;

  if (is_legacy(line)) {                         /* p+ i- a+ s+ c+ c- */
    for (i = 0; i < n; i += 2) {
      sg = (line[i + 1] == '+') ? 1 : -1;
      if (line[i] == 'p') step_param(0, sg);
      else if (line[i] == 'i') step_param(1, sg);
      else if (line[i] == 'a') step_param(2, sg);
      else if (line[i] == 's') step_param(3, sg);
      else if (sg > 0) start_cal(); else finish_cal();
    }
    params_apply();
    print_gains();
    return;
  }

  cmd = strtok(line, delim);
  a1 = strtok(0, delim);
  a2 = strtok(0, delim);
  if (!cmd) return;

  cl = strlen(cmd);                              /* "k1+", "k1-" */
  sg = 0;
  if (cl > 1 && (cmd[cl - 1] == '+' || cmd[cl - 1] == '-')) { sg = (cmd[cl - 1] == '+') ? 1 : -1; cmd[cl - 1] = 0; }
  k = pfind(cmd);
  if (k >= 0) {
    if (sg) step_param((u8)k, sg);
    else if (a1) {
      *pptr((u8)k) = atof(a1);
      if (*pptr((u8)k) < PLO[k]) *pptr((u8)k) = PLO[k];
      if (*pptr((u8)k) > PHI[k]) *pptr((u8)k) = PHI[k];
    }
    if (k == 4 && (sg || a1)) req_zero_trim = 1;
    params_apply();
    pname((u8)k); printf(" = "); put_float(*pptr((u8)k), 5); nl();
    return;
  }

  if (!strcmp(cmd, "help") || !strcmp(cmd, "?")) { if (!busy_balancing()) print_help(); }
  else if (!strcmp(cmd, "get"))   { if (!busy_balancing()) print_params(); }
  else if (!strcmp(cmd, "stat"))  print_stat();
  else if (!strcmp(cmd, "imu"))   print_imu();
  else if (!strcmp(cmd, "start")) {
    if (core_state == ST_FAULT) { req_reinit = 1; printf("reiniciando sensores...\r\n"); }
    else { core_state = ST_WAIT; printf("ESPERA: pon el stick vertical\r\n"); }
  }
  else if (!strcmp(cmd, "stop"))    { core_state = ST_IDLE; printf("IDLE\r\n"); }
  else if (!strcmp(cmd, "cal"))     start_cal();
  else if (!strcmp(cmd, "zero"))    {
    P.offset = tl_thetaRawQ16 / 65536.0; req_zero_trim = 1; params_apply();
    printf("off = "); put_float(P.offset, 3); nl();
  }
  else if (!strcmp(cmd, "gyrocal")) { core_state = ST_GYROCAL; printf("calibrando giroscopio, no lo muevas\r\n"); }
  else if (!strcmp(cmd, "save"))    {
    if (busy_balancing()) return;               /* la EEPROM tarda ~4 ms por byte */
    P.offset += tl_trimQ16 / 65536.0; req_zero_trim = 1; params_apply();
    params_save(); printf("guardado (off = "); put_float(P.offset, 3); printf(")\r\n");
  }
  else if (!strcmp(cmd, "load"))    {
    core_state = ST_IDLE;
    if (params_load()) { params_clamp(); params_apply(); printf("cargado (IDLE; usa start)\r\n"); }
    else printf("no hay datos\r\n");
  }
  else if (!strcmp(cmd, "defaults")) { core_state = ST_IDLE; params_default(); params_apply(); printf("por defecto (usa save)\r\n"); }
  else if (!strcmp(cmd, "reboot"))   { out_coast(); motor_apply(); delay_ms(20); reset_cpu(); }
  else if (!strcmp(cmd, "inv") && a1) {
    core_state = ST_IDLE;
    f = 0;
    if (!strcmp(a1, "motor")) f = &P.invMotor;
    else if (!strcmp(a1, "enc")) f = &P.invEnc;
    else if (!strcmp(a1, "imu")) f = &P.invImu;
    else if (!strcmp(a1, "gyro")) f = &P.invGyro;
    if (f) {
      *f = (*f > 0.5) ? 0 : 1;
      params_apply();
      printf("inv "); put_str(a1); printf(" = "); put_u32((u32)*f); printf(" (IDLE; usa start)\r\n");
    } else printf("uso: inv motor|enc|imu|gyro\r\n");
  }
  else if (!strcmp(cmd, "m") && a1) {
    if (busy_balancing()) return;
    u = atof(a1);
    if (u > 1) u = 1;
    if (u < -1) u = -1;
    sec = a2 ? atof(a2) : 2.0;
    if (sec < 0.05) sec = 0.05;
    if (sec > 10) sec = 10;
    test_pwm = f2l(u * PWM_MAX);
    test_ticks = f2l(sec * LOOP_HZ);
    core_state = (test_pwm == 0) ? ST_IDLE : ST_MOTORTEST;
    printf("motor u="); put_float(u, 2); printf(" por "); put_float(sec, 2); printf(" s\r\n");
  }
  else if (!strcmp(cmd, "dbfind")) {
    if (busy_balancing()) return;
    core_state = ST_DBFIND; printf("buscando zona muerta...\r\n");
  }
  else if (!strcmp(cmd, "tel")) {
    hz = a1 ? atof(a1) : 10.0;
    if (hz <= 0) tel_div = 0;
    else { if (hz > 50) hz = 50; if (hz < 1) hz = 1; tel_div = (u16)(LOOP_HZ / hz); }
  }
  else printf("comando desconocido (help)\r\n");
}

/* ======================= tareas de fondo ======================= */
void poll_serial(void) {
  char c;
  if (!kbhit()) return;
  c = getc();
  if (c == '\r' || c == '\n') {
    if (line_len) { line[line_len] = 0; process_line(); line_len = 0; }
  } else if (line_len < sizeof(line) - 1) line[line_len++] = c;
}

void handle_events(void) {
  float r;
  if (ev_gyrocal == 1) { ev_gyrocal = 0; printf("giroscopio calibrado -> ESPERA\r\n"); beep(80); }
  if (ev_gyrocal == 2) { ev_gyrocal = 0; printf("se movio durante la calibracion, repitiendo...\r\n"); }
  if (ev_caldone) {
    ev_caldone = 0;
    params_save();
    printf("offset = "); put_float(P.offset, 3); printf(" deg guardado -> ESPERA\r\n"); beep(150);
  }
  if (ev_engaged) { ev_engaged = 0; printf("equilibrando\r\n"); }
  if (ev_fell)    { ev_fell = 0; printf("caida -> ESPERA\r\n"); }
  if (ev_runaway) { ev_runaway = 0; printf("rueda saturada: sube K3 o revisa off\r\n"); beep(300); }
  if (ev_fault)   { ev_fault = 0; printf("FALLA I2C: revisa cables/ruido. 'start' para reintentar\r\n"); beep(500); }
  if (ev_db >= 0) {
    r = (float)ev_db / PWM_MAX;
    ev_db = -1;
    if (r > 0) { P.deadband = r * 0.9; params_apply(); printf("zona muerta "); put_float(r, 3);
                 printf(" -> db = "); put_float(P.deadband, 3); printf(" (usa save)\r\n"); }
    else printf("la rueda no giro hasta 50%%: revisa motor/encoder\r\n");
  }
}

u8 adc_phase = 0;
u32 last_bat = 0, last_warn = 0;

void update_battery(void) {
#if USE_VBAT
  s32 mv;
  if (loops - last_bat < LOOP_HZ / 40) return;        /* 40 Hz: arranca y lee en fases */
  last_bat = loops;
  if (adc_phase == 0) { read_adc(ADC_START_ONLY); adc_phase = 1; return; }
  mv = ((s32)read_adc(ADC_READ_ONLY) * VBAT_MV_NUM) / 1023;
  adc_phase = 0;
  if (vbat_mv < 1000) vbat_mv = mv; else vbat_mv += (mv - vbat_mv) / 8;
  core_set_vbat_mv(vbat_mv);
  if (vbat_mv > 5000 && vbat_mv < 3500L * BAT_CELLS && loops - last_warn > 5L * LOOP_HZ) {
    last_warn = loops; printf("BATERIA BAJA: "); put_fx(vbat_mv / 10, 2); printf(" V\r\n"); beep(400);
  }
#endif
}

void update_led(void) {
  u32 t = loops / (LOOP_HZ / 100);                   /* centésimas de segundo */
  int1 on;
  if (core_state == ST_BAL) on = 1;
  else if (core_state == ST_WAIT) on = (t / 15) & 1;
  else if (core_state == ST_FAULT) on = (t / 5) & 1;
  else on = (t / 60) & 1;
  output_bit(PIN_LED, on);
  if (buzz_until && loops >= buzz_until) { output_low(PIN_BUZZER); buzz_until = 0; }
}

u32 last_tel = 0;
void send_telemetry(void) {
  if (!tel_div || loops - last_tel < tel_div) return;
  last_tel = loops;
  printf("th:");   put_q8(tl_theta, 2);
  printf(",w:");   put_q8(tl_omega, 1);
  printf(",ws:");  put_q8(tl_ws, 2);
  printf(",wp:");  put_q8(tl_wp, 2);
  printf(",u:");   put_fx((tl_u * 1000) / PWM_MAX, 3);
  printf(",trim:"); put_fx(tl_trimQ16 / 655, 2);
  nl();
}

void background(void) {
  poll_serial();
  handle_events();
  update_battery();
  update_led();
  send_telemetry();
}

/* ======================= inicialización ======================= */
int1 sensors_init(int1 verbose) {
  u8 who = 0;
  int1 m, e;
  m = mpu_init(&who);
  e = as5600_init();
  if (verbose) {
    printf("MPU6050: "); if (m) printf("OK"); else printf("NO RESPONDE");
    printf(" (WHO_AM_I="); put_u32(who); printf(")   AS5600: ");
    if (e) printf("OK"); else printf("NO RESPONDE");
    nl();
  }
  return m && e;
}

void hw_init(void) {
  output_low(PIN_AIN1); output_low(PIN_AIN2);
  output_high(PIN_STBY);
  output_low(PIN_LED); output_low(PIN_BUZZER);

  setup_ccp1(CCP_PWM);
  setup_timer_2(T2_DIV, T2_PR, 10);                   /* PWM 20 kHz, interrupción 2 kHz */
  set_pwm1_duty((int16)0);

#if USE_VBAT
  #if defined(USE_PIC18F45K22)
    setup_adc_ports(sAN0);
  #else
    setup_adc_ports(AN0);
  #endif
  setup_adc(ADC_CLOCK_DIV_64);
  set_adc_channel(0);
#endif

  SSP_CKE = 1;                                        /* niveles SMBus: bus I2C de 3.3 V */
}

void main(void) {
  int1 loaded, ok;

  hw_init();
  params_default();
  loaded = params_load();
  params_clamp();
  params_apply();

  enable_interrupts(GLOBAL);                          /* necesario para el buffer de TX */
  delay_ms(100);
  printf("\r\n=== Stick autoequilibrio PIC18 ===\r\n");
  printf("parametros: "); if (loaded) printf("EEPROM\r\n"); else printf("por defecto\r\n");
  ok = sensors_init(1);
  core_state = ok ? ST_GYROCAL : ST_FAULT;
  if (ok) printf("calibrando giroscopio, no lo muevas...\r\n");
  printf("escribe help\r\n");

  tick_pending = 0;
  enable_interrupts(INT_TIMER2);

  while (TRUE) {
    while (tick_pending == 0) background();
    if (tick_pending > 1) stat_overruns += tick_pending - 1;
    tick_pending = 0;

    if (req_reinit) {                                 /* reintento tras FALLA */
      req_reinit = 0;
      out_coast(); motor_apply();
      if (sensors_init(1)) { core_reset_filters(); core_state = ST_GYROCAL; }
      tick_pending = 0;
      continue;
    }

    in_ok = sensors_read();
    if (!in_ok) stat_i2c_err++;
    control_step();
    motor_apply();
    loops++;
  }
}
