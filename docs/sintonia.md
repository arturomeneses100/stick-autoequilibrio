# Puesta en marcha y sintonía

Ambos firmwares (ESP32 y PIC) usan los mismos comandos y las mismas unidades. Un terminal serie basta: Arduino IDE, PuTTY, o una app de terminal Bluetooth en el celular.

## 1. Encendido

Deja el stick **quieto** al encender: calibra el giroscopio (2 s en ESP32, 1 s en PIC). Si se mueve, repite la calibración solo. Al terminar pasa a `ESPERA`: en cuanto lo pongas vertical durante 200 ms, empieza a equilibrar.

LED: parpadeo lento = IDLE, rápido = ESPERA, fijo = equilibrando, muy rápido = falla.

## 2. Verificar signos (indispensable)

Activa la telemetría con `tel 20`, o `tel 10` en el PIC a 9600 baud. En Arduino IDE, el Serial Plotter grafica `th`, `w`, `ws`, `wp` y `u`.

| # | Prueba | Correcto | Si está mal |
|---|---|---|---|
| 1 | `m 0.3` (la rueda gira 2 s) | `ws` positivo | `inv enc` |
| 2 | Inclina el stick a mano | `w` positivo mientras `th` aumenta | `inv gyro` |
| 3 | Sostén el stick suelto y envía `m 0.4 0.3` | el golpe empuja el cuerpo hacia `th` **negativo** | `inv imu` |

Al final, `save`. Si una prueba falla, ninguna ganancia va a funcionar.

Si `th` marca cerca de 180° o ±45° con el stick vertical, la orientación del MPU6050 no coincide. Usa `imu` para ver qué eje lee +1 g y ajusta `AX_TILT`, `AX_UP`, `UP_SIGN` y `AX_GYRO` en el código.

## 3. Calibrar el punto de equilibrio

1. `c+` → el motor se apaga.
2. Sostén el stick en su punto de equilibrio exacto: el punto donde, al soltarlo, no se cae hacia ningún lado.
3. `c-` → guarda el offset y empieza a equilibrar.

Con `trim > 0` (por defecto 0.02), el offset se sigue ajustando solo mientras equilibra. Después de un minuto estable, `save` lo vuelve permanente. El comando `zero` toma el ángulo actual como offset sin guardar.

## 4. Sintonía de ganancias

Cambia las ganancias en caliente: `k1 0.5`, `k1+` / `k1-` (±5 %), o con los comandos del original (`p+`, `i+`, `a+`, `s+`).

1. **Ángulo primero:** `k3 0`, `k4 0`. Sube `k1` hasta que reaccione firme al inclinarse. Sube `k2` (≈ k1/10) hasta que deje de vibrar u oscilar. Se sostendrá algunos segundos mientras la rueda acelera hasta saturarse. Es normal en esta etapa.
2. **Velocidad de la rueda:** sube `k3` hasta que la rueda deje de desbocarse y se quede casi quieta.
3. **Posición de la rueda:** un `k4` pequeño hace que la rueda regrese a donde empezó. Si oscila lento (varios segundos), bájalo.
4. **Zona muerta:** acuesta el stick con la rueda libre y envía `dbfind`. Mide cuánto PWM necesita el motor para empezar a girar y ajusta `db`.
5. `save`.

| Síntoma | Causa probable | Qué ajustar |
|---|---|---|
| Vibración rápida, zumbido | K2 alto o ruido en el giroscopio | baja `k2`, prueba `glpf 40` |
| Oscila de lado a lado (~1–2 Hz) | K2 bajo | sube `k2` |
| Se cae sin luchar | K1 bajo o signo invertido | revisa los signos, sube `k1` |
| La rueda se acelera hasta saturar | K3 bajo u offset malo | sube `k3`, recalibra con `c+`/`c-` |
| Deriva lenta de ida y vuelta | K4 alto | baja `k4` |
| Se cae a ratos, `stat` con errores I2C | ruido de escobillas | ver anti-ruido en `conexiones.md` |
| Funciona con batería llena y falla con batería baja | sin compensación | `vcomp 1`, revisa el divisor |

Para calcular ganancias de partida para tu stick: `python sim/modelo_lqr.py --m 0.35 --l 0.12 --jw 2e-4`.

## 5. Lista de comandos

| Comando | Descripción |
|---|---|
| `help`, `get` | ayuda / todos los parámetros |
| `stat` | tiempos del lazo, errores I2C, imán AS5600, batería |
| `imu` | lecturas crudas del MPU6050 y ángulo |
| `<param> <valor>` | asigna un parámetro (tabla siguiente) |
| `<param>+`, `<param>-` | ±5 % |
| `p± i± a± s±` | K1..K4 estilo original (`p+p+p+` = 3 pasos) |
| `start`, `stop` | habilita / deshabilita (en FALLA, `start` reinicia los sensores) |
| `c+` … `c-`, `cal` | calibración del punto de equilibrio |
| `zero` | offset = ángulo actual |
| `inv motor\|enc\|imu\|gyro` | invierte un signo |
| `m <u> [seg]` | prueba de motor, `u` de −1 a 1 |
| `dbfind` | mide la zona muerta del motor |
| `gyrocal` | recalibra el giroscopio |
| `tel <Hz>` | telemetría (0 = apagada) |
| `save`, `load`, `defaults`, `reboot` | memoria no volátil |

| Parámetro | Por defecto | Unidades | Significado |
|---|---|---|---|
| `k1` | 0.45 | duty/° | ganancia de ángulo |
| `k2` | 0.045 | duty/(°/s) | ganancia de velocidad angular |
| `k3` | 0.08 | duty/rps | ganancia de velocidad de la rueda |
| `k4` | 0.015 | duty/rev | ganancia de posición de la rueda |
| `off` | 0 | ° | ángulo de equilibrio |
| `tau` | 0.5 | s | constante del filtro complementario |
| `wlpf` | 25 | Hz | filtro de velocidad de la rueda |
| `glpf` | 0 | Hz | filtro del término derivativo (0 = apagado) |
| `db` | 0.05 | 0–0.5 | compensación de zona muerta |
| `maxang` | 12 | ° | ángulo de caída |
| `startang` | 0.5 | ° | ángulo para enganchar |
| `trim` | 0.02 | °/(rps·s) | autoajuste del offset (0 = apagado) |
| `maxrps` | 80 | rps | velocidad máxima de la rueda antes de soltar |
| `vnom` | 11.1 | V | voltaje de referencia para la compensación |
| `vcomp` | 1 | 0/1 | compensación de batería |
| `brake` | 1 | 0/1 | frenar la rueda al caer (0 = rueda libre) |

En el PIC, los límites de `k2` (≤ 0.5), `k4` (≤ 0.5) y `maxrps` (≤ 150) son menores que en el ESP32 para que la aritmética de 32 bits no se desborde.
