# Teoría de control del stick de rueda de reacción

## 1. Modelo físico

El stick es un péndulo invertido que pivota sobre una esquina. En la punta lleva una rueda de inercia movida por un motor DC. Variables:

- θ: ángulo del cuerpo respecto a la vertical (lo mide el MPU6050)
- φ: ángulo de la rueda **relativo al cuerpo** (lo mide el AS5600, montado en el cuerpo)
- τ: par del motor sobre la rueda; el cuerpo recibe −τ (reacción)

```
Ib · θ̈          = m·g·l·sin θ − τ          (cuerpo)
Jw · (θ̈ + φ̈)   = τ                          (rueda, velocidad absoluta)
τ = (Kt/R) · (V·u − Ke·φ̇)                   (motor DC con fuerza contraelectromotriz)
```

`Ib` es la inercia del conjunto respecto al pivote, `Jw` la inercia de giro de la rueda, `l` la distancia del pivote al centro de masa y `u ∈ [−1, 1]` el ciclo útil del PWM con signo.

Sin control, el polo inestable está en `+√(m·g·l / Ib)` (unos 8 rad/s en un stick típico). Por eso el lazo tiene que ser rápido: con 100 Hz el retardo de un ciclo ya es apreciable; con 500 Hz es despreciable.

## 2. Realimentación de estado

El estado es `x = [θ, θ̇, φ̇, φ]` y la ley de control es lineal:

```
u = K1·θ + K2·θ̇ + K3·φ̇ + K4·φ
```

- **K1 (ángulo)** y **K2 (velocidad angular)** forman un PD: sostienen el stick.
- **K3 (velocidad de la rueda)** evita que la rueda se acelere hasta saturarse. Su signo es contraintuitivo: para frenar la rueda, el controlador primero tiene que inclinar el cuerpo un poco hacia el lado contrario, de modo que la gravedad aporte el par necesario. Por eso K3 tiene el **mismo** signo que K1.
- **K4 (posición de la rueda)** regresa la rueda a su posición inicial y elimina la deriva lenta.

Con la convención de signos del firmware (prueba 3 de `sintonia.md`), las cuatro ganancias son positivas. Esto se verifica en `sim/modelo_lqr.py`.

### LQR

Las ganancias por defecto salen de un regulador cuadrático lineal (LQR) con pesos de Bryson (`1/valor_máximo²`): θ ≤ 2°, θ̇ ≤ 30 °/s, φ̇ ≤ 5 rps, φ ≤ 20 rev, u ≤ 0.5. El script linealiza el modelo, resuelve la ecuación de Riccati y convierte K a unidades del firmware (°, °/s, rps, rev). Después las prueba con un lazo discreto de 500 Hz, retardo de un ciclo, saturación y zona muerta, en 81 combinaciones de masa, largo, inercia y motor.

## 3. Estimación del ángulo: filtro complementario

- El **giroscopio** es preciso a corto plazo pero deriva.
- El **acelerómetro** es exacto a largo plazo, pero ruidoso, y la rueda lo sacude.

```
θ ← α·(θ + ω·dt) + (1 − α)·θ_acc        α = τ / (τ + dt)
```

Con τ = 0.5 s el cruce está en 1/(2π·τ) ≈ 0.3 Hz: arriba de esa frecuencia manda el giroscopio y abajo el acelerómetro, que solo corrige la deriva. El firmware además **ignora el acelerómetro** cuando la magnitud medida se aleja más de 15 % de 1 g, como pasa durante un golpe de la rueda.

El sesgo del giroscopio se mide al encender (promedio con el sensor quieto) y se guarda con 4 bits fraccionarios en el PIC para no perder resolución.

## 4. Velocidad de la rueda

El AS5600 da 4096 cuentas por vuelta. Cada ciclo se calcula la diferencia con "desenvolvimiento" (si el salto es mayor a media vuelta, cruzó por cero), se acumula la posición multivuelta y se deriva la velocidad. A 500 Hz, una cuenta por ciclo equivale a 0.12 rps, así que la velocidad pasa por un filtro pasa-bajas de primer orden (`wlpf`, 25 Hz).

## 5. No linealidades del actuador

- **Zona muerta:** un motor DC con escobillas no gira hasta cierto PWM. Se suma un escalón suave `db` a la salida: `|u_out| = db + (1 − db)·|u|`, con una rampa en el primer 2 % para que no haya brinco en cero.
- **Batería:** el par es proporcional a `V·u`. Al bajar la LiPo de 12.6 a 10.5 V, la ganancia efectiva baja 17 %. El firmware multiplica `u` por `V_nom / V_bat`.
- **Saturación:** `u` se limita a ±1. Si la rueda pasa de `maxrps`, ya no puede dar par útil y el controlador suelta.

## 6. Autoajuste del punto de equilibrio

Si el offset calibrado está mal por δ grados, en estado estable el controlador mantiene la rueda girando o desplazada para compensar. El autoajuste integra lentamente esa señal:

```
trim ← trim − k_trim · (φ̇ + φ/2) · dt
```

En el estado estable solo se detiene cuando la rueda está quieta y en su posición, es decir, cuando el offset es el correcto. En la simulación corrige un error de 1° en unos 60 s.

## 7. Punto fijo en el PIC

Un PIC18 tarda decenas de microsegundos en una operación de punto flotante de 32 bits, y `atan2` tarda milisegundos. El núcleo del PIC (`control_core.c`) usa enteros de 32 bits con escalas fijas (formato Qn = valor × 2ⁿ):

| Variable | Formato | Rango usado |
|---|---|---|
| ángulo integrado | Q16 [°] | ±32768° |
| ángulo y velocidad para el control | Q8 [°], [°/s] | ±500 °/s |
| velocidad de rueda | Q12 [rps] (exacta: `cuentas × LOOP_HZ`) | ±500 rps |
| autoajuste | Q20 [°] | ±3° |
| ganancias | `K · PWM_MAX · 16` | salida `>> 12` = cuentas PWM |

- **Ángulo del acelerómetro sin `atan2`:** `atan(x) ≈ x / (1 + 0.28·x²)`, con error menor a 0.3° hasta 45°. Cerca de la vertical el error es despreciable, y la calibración del offset lo absorbe.
- **Sin desbordes:** cada producto se acotó para no pasar de 2³¹. Cada término de la ley de control se satura antes de sumarse, y los límites de los parámetros (`k2 ≤ 0.5`, `k4 ≤ 0.5`, `maxrps ≤ 150`) garantizan el rango.
- **Desplazamiento aritmético portable:** `asr()` en lugar de `>>` sobre negativos, porque su comportamiento depende del compilador.
- **Flotantes solo fuera del lazo:** los parámetros de usuario se guardan como `float`. `params_apply()` los convierte a enteros una sola vez, cuando cambian.

El mismo archivo compila en gcc. `sim/test_control_core.c` lo prueba contra el modelo físico con ruido y sesgo de sensores, así que la versión PIC se valida sin hardware.

## 8. Temporización

| | ESP32 | PIC18 |
|---|---|---|
| Disparo del lazo | `esp_timer` → notificación a tarea FreeRTOS de prioridad 20 en el núcleo 1 | Timer2 (el mismo del PWM) con postescala 10 → interrupción a 2 kHz → 1 de cada 4 |
| Comunicación | `loop()` de prioridad baja | entre ciclos de control, con buffers de RX/TX por interrupción |
| Lectura I2C (400 kHz) | ~0.6 ms | ~0.7 ms |
| Estadísticas | periodo mín/máx, tiempo de ejecución, sobrecargas | ciclos perdidos, errores I2C |
