"""
modelo_lqr.py — Modelo del stick de rueda de reacción y cálculo de ganancias LQR
en las MISMAS unidades que usan los firmwares (ESP32 y PIC):

    u[duty -1..1] = K1*theta[deg] + K2*omega[deg/s] + K3*vel_rueda[rps] + K4*pos_rueda[rev]

Uso:
    python modelo_lqr.py                      # parámetros típicos
    python modelo_lqr.py --m 0.35 --l 0.12 --jw 2e-4 --kt 0.015 --r 3.5 --v 11.1
    python modelo_lqr.py --plot               # grafica la respuesta (requiere matplotlib)

Requisitos: numpy, scipy  (pip install numpy scipy matplotlib)

Modelo (ángulo theta del cuerpo desde la vertical, phi = ángulo de la rueda
relativo al cuerpo, que es lo que mide el AS5600):
    Ib * theta''          = m*g*l*sin(theta) - tau
    Jw * (theta'' + phi'') = tau
    tau = Kt/R * (V*u - Ke*phi')          (motor DC, Ke = Kt en SI)
"""
import argparse
import itertools
import numpy as np
import scipy.linalg as sl

G = 9.81


def dinamica(x, u, p):
    th, thd, phd, ph = x
    Ib = p["m"] * p["l"] ** 2 * 1.4          # cuerpo + rueda como masa en la punta (aprox.)
    tau = p["kt"] / p["r"] * (p["v"] * u - p["kt"] * phd)
    thdd = (p["m"] * G * p["l"] * np.sin(th) - tau) / Ib
    phdd = tau / p["jw"] - thdd
    return np.array([thd, thdd, phdd, phd])


def linealizar(p):
    A = np.zeros((4, 4)); B = np.zeros((4, 1)); e = 1e-6
    for i in range(4):
        dx = np.zeros(4); dx[i] = e
        A[:, i] = (dinamica(dx, 0, p) - dinamica(-dx, 0, p)) / (2 * e)
    B[:, 0] = (dinamica(np.zeros(4), e, p) - dinamica(np.zeros(4), -e, p)) / (2 * e)
    return A, B


def lqr_firmware(p, th_max=2.0, w_max=30.0, ws_max=5.0, pos_max=20.0, u_max=0.5):
    """Ganancias LQR con pesos de Bryson (1/valor_máximo^2), en unidades del firmware."""
    A, B = linealizar(p)
    Q = np.diag([1 / np.radians(th_max) ** 2, 1 / np.radians(w_max) ** 2,
                 1 / (2 * np.pi * ws_max) ** 2, 1 / (2 * np.pi * pos_max) ** 2])
    R = np.array([[1 / u_max ** 2]])
    P = sl.solve_continuous_are(A, B, Q, R)
    K = (np.linalg.inv(R) @ B.T @ P).ravel()                       # u = -K x (SI)
    conv = np.array([np.radians(1), np.radians(1), 2 * np.pi, 2 * np.pi])
    Kfw = -K * conv                                                 # u = +Kfw z (firmware)
    polos = np.linalg.eigvals(A - B @ K.reshape(1, 4))
    return Kfw, polos


def simular(Kfw, p, loop_hz=500, T=8.0, th0=3.0, db=0.05, guardar=False):
    """Lazo discreto con un ciclo de retardo, saturación y zona muerta (como el firmware)."""
    dt = 1.0 / loop_hz
    x = np.array([np.radians(th0), 0, 0, 0]); u_prev = 0.0
    hist = []
    for k in range(int(T / dt)):
        z = np.array([np.degrees(x[0]), np.degrees(x[1]), x[2] / (2 * np.pi), x[3] / (2 * np.pi)])
        u = float(np.clip(Kfw @ z, -1, 1))
        if abs(u) > 1e-4:
            u = np.sign(u) * (db * min(abs(u) / 0.02, 1) + (1 - db) * abs(u))
        for _ in range(4):
            x = x + dinamica(x, u_prev, p) * dt / 4
        u_prev = u
        if guardar:
            hist.append((k * dt, *z, u))
        if abs(x[0]) > np.radians(20):
            return False, np.array(hist)
    return True, np.array(hist)


def main():
    ap = argparse.ArgumentParser(description="Ganancias LQR para el stick de rueda de reacción")
    ap.add_argument("--m", type=float, default=0.30, help="masa total [kg]")
    ap.add_argument("--l", type=float, default=0.10, help="pivote -> centro de masa [m]")
    ap.add_argument("--jw", type=float, default=1.5e-4, help="inercia de la rueda [kg m^2] (anillo: m*r^2)")
    ap.add_argument("--kt", type=float, default=0.012, help="constante de par del motor [N m/A]")
    ap.add_argument("--r", type=float, default=4.0, help="resistencia del motor [ohm]")
    ap.add_argument("--v", type=float, default=11.1, help="voltaje de batería [V]")
    ap.add_argument("--hz", type=int, default=500, help="frecuencia del lazo [Hz]")
    ap.add_argument("--plot", action="store_true", help="graficar la respuesta")
    a = ap.parse_args()
    p = dict(m=a.m, l=a.l, jw=a.jw, kt=a.kt, r=a.r, v=a.v)

    Kfw, polos = lqr_firmware(p)
    print("Ganancias LQR para el firmware:")
    for n, k in zip(["k1", "k2", "k3", "k4"], Kfw):
        print(f"  {n} {k:.4f}")
    print("Polos en lazo cerrado:", np.round(polos, 2))

    ok, _ = simular(Kfw, p, a.hz)
    print(f"Simulación desde 3° con zona muerta 5 %: {'ESTABLE' if ok else 'CAE'}")

    Kdef = np.array([0.45, 0.045, 0.08, 0.015])
    est = tot = 0
    for m, l, jw, kt in itertools.product([0.2, 0.3, 0.45], [0.07, 0.10, 0.14],
                                          [0.8e-4, 1.5e-4, 3e-4], [0.008, 0.012, 0.02]):
        q = dict(m=m, l=l, jw=jw, kt=kt, r=a.r, v=a.v)
        est += simular(Kdef, q, a.hz, th0=2)[0]; tot += 1
    print(f"Ganancias por defecto del firmware estables en {est}/{tot} combinaciones de planta")

    if a.plot:
        import matplotlib.pyplot as plt
        _, h = simular(Kfw, p, a.hz, guardar=True)
        fig, ax = plt.subplots(4, 1, sharex=True, figsize=(8, 8))
        for i, (lab) in enumerate(["theta [deg]", "omega [deg/s]", "rueda [rps]", "u [duty]"]):
            col = [1, 2, 3, 5][i]
            ax[i].plot(h[:, 0], h[:, col]); ax[i].set_ylabel(lab); ax[i].grid(True)
        ax[-1].set_xlabel("t [s]")
        fig.suptitle("Respuesta con LQR desde 3°")
        plt.tight_layout(); plt.show()


if __name__ == "__main__":
    main()
