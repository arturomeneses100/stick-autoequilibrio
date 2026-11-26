/* ============================================================================
 *  Banco de pruebas del núcleo de control PIC (control_core.c) en la PC.
 *  Simula el stick (cuerpo + rueda + motor DC) y genera lecturas crudas
 *  de MPU6050 y AS5600 con ruido y sesgo, tal como las vería el PIC.
 *
 *  Compilar y ejecutar (Linux/macOS/MSYS2/WSL):
 *      gcc -O2 -o test_core test_control_core.c -lm && ./test_core
 *  Windows (MinGW):  gcc -O2 -o test_core.exe test_control_core.c && test_core
 * ==========================================================================*/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#ifndef LOOP_HZ
#define LOOP_HZ 500
#endif
#ifndef PWM_MAX
#define PWM_MAX 800
#endif
#include "../firmware/pic/stick_pic/control_core.c"

/* -------- Planta: parámetros típicos (cámbialos por los de tu stick) -------- */
typedef struct { double m, l, Jw, Kt, R, V; } Plant;
static Plant PL = {0.30, 0.10, 1.5e-4, 0.012, 4.0, 11.1};
static double th, thd, phd, ph;          /* cuerpo [rad], [rad/s]; rueda rel. [rad/s], [rad] */

static double gauss(void) {              /* ruido normal (Box-Muller) */
  double u1 = (rand() + 1.0) / (RAND_MAX + 2.0), u2 = (rand() + 1.0) / (RAND_MAX + 2.0);
  return sqrt(-2 * log(u1)) * cos(6.283185307 * u2);
}

static void plant_step(double dt, int mode, double u) {
  double Ib = PL.m * PL.l * PL.l * 1.4, a = PL.m * 9.81 * PL.l, tau, thdd, phdd;
  int k;
  for (k = 0; k < 8; k++) {
    if (mode == MODE_DRIVE)      tau = PL.Kt / PL.R * (PL.V * u - PL.Kt * phd);
    else if (mode == MODE_BRAKE) tau = PL.Kt / PL.R * (-PL.Kt * phd);
    else                         tau = 0;
    thdd = (a * sin(th) - tau) / Ib;
    phdd = tau / PL.Jw - thdd;
    th += thd * dt / 8;  thd += thdd * dt / 8;
    ph += phd * dt / 8;  phd += phdd * dt / 8;
  }
}

static double offsetTrue = 0;            /* error de montaje del IMU [deg] */
static double gyroBiasRaw = 0, noise = 1;

static void sensors(void) {
  double thm = th + offsetTrue / 57.29578;
  long e;
  in_acc[AX_TILT] = (s16)(16384 * sin(thm) + 60 * noise * gauss());
  in_acc[AX_UP]   = (s16)(16384 * cos(thm) + 60 * noise * gauss());
  in_acc[2]       = (s16)(60 * noise * gauss());
  in_gyr[0] = 0;  in_gyr[1] = 0;
  in_gyr[AX_GYRO] = (s16)(thd * 57.29578 * 65.5 + gyroBiasRaw + 4 * noise * gauss());
  e = (long)floor(ph / 6.283185307 * 4096.0);
  in_enc = (u16)(((e % 4096) + 4096) % 4096);
  in_ok = 1;
}

/* Corre un escenario: sostiene el stick (calibración + espera) y lo suelta */
static int run(const char* name, double T, double holdDeg, double pushAt, double pushDps,
               double* maxAbs, double* wsEnd, double* trimEnd) {
  double dt = 1.0 / LOOP_HZ, t = 0, mx = 0;
  int held = 1, fell = 0;
  long i, N = (long)(T * LOOP_HZ);
  th = holdDeg / 57.29578;  thd = 0;  phd = 0;  ph = 0;
  core_state = ST_GYROCAL;  prevState = 255;  angleInit = 0;  encInit = 0;
  wsQ12 = 0;  trimQ20 = 0;  encCount = 0;
  for (i = 0; i < N; i++, t += dt) {
    sensors();
    control_step();
    if (held) {                              /* la mano lo sostiene quieto */
      th = holdDeg / 57.29578;  thd = 0;
      plant_step(dt, out_mode, (double)out_pwm / PWM_MAX);
      thd = 0;
      if (core_state == ST_BAL) held = 0;
    } else {
      plant_step(dt, out_mode, (double)out_pwm / PWM_MAX);
      if (pushAt > 0 && fabs(t - pushAt) < dt / 2) thd += pushDps / 57.29578;
      if (fabs(th) * 57.29578 > mx) mx = fabs(th) * 57.29578;
      if (core_state != ST_BAL) { fell = 1; break; }
    }
  }
  *maxAbs = mx;  *wsEnd = phd / 6.283185307;  *trimEnd = tl_trimQ16 / 65536.0;
  printf("%-34s %-6s |th|max=%5.2f deg  ws=%6.2f rps  trim=%6.3f deg  estado=%d\n",
         name, fell ? "CAYO" : "OK", mx, *wsEnd, *trimEnd, core_state);
  return !fell;
}

int main(void) {
  double mx, ws, tr;
  int ok = 1;
  srand(1);
  params_default();
  params_apply();
  printf("LOOP_HZ=%d  PWM_MAX=%d\n", LOOP_HZ, PWM_MAX);

  ok &= run("1. equilibrio nominal (20 s)", 20, 0.2, 0, 0, &mx, &ws, &tr);
  ok &= run("2. empujon de 40 deg/s en t=8 s", 20, 0.2, 8, 40, &mx, &ws, &tr);

  offsetTrue = 1.0;                                  /* IMU montado 1° chueco */
  ok &= run("3. offset mal calibrado 1 deg + trim", 60, -1.0 + 0.2, 0, 0, &mx, &ws, &tr);
  if (fabs(tr - 1.0) > 0.15) { printf("   -> el trim no convergio a +1 deg\n"); ok = 0; }
  offsetTrue = 0;

  gyroBiasRaw = 150;  noise = 3;                     /* sesgo de 2.3 °/s y ruido x3 */
  ok &= run("4. sesgo de giro 2.3 deg/s + ruido x3", 20, 0.2, 0, 0, &mx, &ws, &tr);
  gyroBiasRaw = 0;  noise = 1;

  P.vComp = 1;  params_apply();  PL.V = 10.0;  core_set_vbat_mv(10000);  /* batería baja */
  ok &= run("5. bateria a 10 V con compensacion", 20, 0.2, 0, 0, &mx, &ws, &tr);
  PL.V = 11.1;  core_set_vbat_mv(11100);

  P.K1 = 0;  P.K2 = 0;  P.K3 = 0;  P.K4 = 0;  params_apply();
  {
    int fellOk = !run("6. ganancias en cero (debe caer)", 5, 0.2, 0, 0, &mx, &ws, &tr);
    printf("   deteccion de caida y paso a ESPERA: %s\n", (fellOk && core_state == ST_WAIT) ? "OK" : "FALLA");
    ok &= fellOk && core_state == ST_WAIT;
  }
  printf("\nRESULTADO: %s\n", ok ? "TODAS LAS PRUEBAS PASARON" : "HAY FALLAS");
  return ok ? 0 : 1;
}
