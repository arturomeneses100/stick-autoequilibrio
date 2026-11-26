# Stick de autoequilibrio de un eje (rueda de reacción)

Un péndulo invertido que se mantiene de pie sobre una esquina acelerando y frenando una rueda de inercia con un motor DC. Este repositorio tiene dos firmwares con la misma ley de control y las mismas unidades de ganancia:

| Firmware | Microcontrolador | Lazo | Aritmética | Ajuste |
|---|---|---|---|---|
| [`firmware/esp32`](firmware/esp32/StickBalance_ESP32) | ESP32  | 500 Hz en tarea FreeRTOS | flotante | USB o Bluetooth, guarda en NVS |
| [`firmware/pic`](firmware/pic/stick_pic) | PIC18F45K22 o PIC18F4550 (CCS PIC C) | 500 Hz por Timer2 | punto fijo 32 bits | UART / HC-05, guarda en EEPROM |

Es una reimplementación de la idea de [remrc/One-Axis-Self-Balancing-Stick-DC](https://github.com/remrc/One-Axis-Self-Balancing-Stick-DC), pensada como práctica de la unidad *Desarrollo de aplicaciones de control de procesos utilizando sistemas embebidos*: lazo cerrado, realimentación de estado, filtrado de sensores, punto fijo y sintonía experimental.

## Qué mejora respecto al original

- **Lazo 5 veces más rápido** (500 Hz contra 100 Hz) y con periodo fijo marcado por un temporizador de hardware.
- **Realimentación de estado completa**: ángulo, velocidad angular, velocidad de la rueda y posición de la rueda. Las ganancias por defecto salen de un LQR y se probaron en simulación.
- **Filtro complementario con compuerta**: ignora el acelerómetro durante los golpes de la rueda.
- **Compensación de la zona muerta** del motor DC, sin brinco cerca de cero.
- **Compensación de batería**: las ganancias no cambian conforme baja la LiPo.
- **Autoajuste del punto de equilibrio** (`trim`): corrige un error de calibración de 1° en menos de un minuto.
- **Seguridad**: detección de caída con histéresis, arranque suave, corte si la rueda se satura y detección de fallas de I2C.
- **Diagnóstico integrado**: telemetría para Serial Plotter, prueba de motor, medición automática de zona muerta y estado del imán del AS5600.
- **Compatible con los comandos del original** (`p+ i+ a+ s+ c+ c-`).

## Hardware

- Microcontrolador: ESP32 DevKit **o** PIC18F45K22 / PIC18F4550
- IMU MPU6050 (GY-521), encoder magnético AS5600 con su imán diametral
- Driver TB6612FNG, motor DC con rueda de inercia
- LiPo 3S (11.1 V), módulo HC-05 opcional (PIC) o Bluetooth integrado (ESP32)
- Piezas impresas en 3D: diseño de remrc en [Cults3D](https://cults3d.com/:920970). Ver [`hardware/`](hardware/README.md).

Conexiones detalladas en [`docs/conexiones.md`](docs/conexiones.md).

> **¿Por qué no PIC16F877A?** Tiene 368 bytes de RAM, no tiene multiplicador por hardware y corre a 5 MIPS. No alcanza para leer dos sensores I2C, filtrar y calcular cuatro términos a una frecuencia útil con margen. Los PIC18 propuestos usan la misma placa de 40 pines y tienen 3 a 4 veces más velocidad y RAM.

## Inicio rápido

1. Arma y conecta según [`docs/conexiones.md`](docs/conexiones.md). Pon los capacitores anti-ruido en el motor.
2. Compila y carga el firmware:
   - **ESP32:** abre `firmware/esp32/StickBalance_ESP32/StickBalance_ESP32.ino` en Arduino IDE con la placa *ESP32 Dev Module*. Funciona con el core 2.x y 3.x.
   - **PIC:** abre `firmware/pic/stick_pic/stick_pic.c` en CCS PIC C. Elige el micro al inicio (`USE_PIC18F45K22` o `USE_PIC18F4550`) y compila. `control_core.c` se incluye solo.
3. Abre un monitor serie (ESP32: 115200 baud; PIC: 9600 baud) y escribe `help`.
4. Deja el stick quieto 2 s al encender: calibra el giroscopio.
5. Revisa los **signos** (3 pruebas) y **calibra** el punto de equilibrio con `c+` ... `c-`. Ver [`docs/sintonia.md`](docs/sintonia.md).
6. Ajusta las ganancias en caliente y guarda con `save`.

## Ley de control

```
u [duty -1..1] = K1·θ[°] + K2·ω[°/s] + K3·vel_rueda[rps] + K4·pos_rueda[rev]
```

Con la convención de signos del firmware, las cuatro ganancias son **positivas**. Valores iniciales: `K1=0.45  K2=0.045  K3=0.08  K4=0.015`. Para calcular las de tu stick: `python sim/modelo_lqr.py --m <kg> --l <m> --jw <kg·m²> --kt <N·m/A> --r <Ω>`.

La teoría, el modelo físico y el diseño en punto fijo están en [`docs/teoria.md`](docs/teoria.md).

## Estructura

```
├── firmware/
│   ├── esp32/StickBalance_ESP32/StickBalance_ESP32.ino
│   └── pic/stick_pic/
│       ├── stick_pic.c        programa principal CCS (hardware, comandos, EEPROM)
│       └── control_core.c     control en punto fijo, C portable (PIC y PC)
├── sim/
│   ├── modelo_lqr.py          modelo físico, LQR y prueba de robustez
│   └── test_control_core.c    banco de pruebas del núcleo PIC contra la planta simulada
├── docs/
│   ├── conexiones.md          diagramas de conexión ESP32 y PIC, anti-ruido
│   ├── sintonia.md            signos, calibración, sintonía y lista de comandos
│   └── teoria.md              modelo, realimentación de estado, filtros, punto fijo
└── hardware/README.md         piezas impresas, lista de material
```

## Pruebas

El núcleo de control del PIC se prueba en la PC contra un modelo físico del stick, con ruido y sesgo de sensores:

```
gcc -O2 -o test_core sim/test_control_core.c -lm
./test_core
```

Escenarios: equilibrio nominal, empujón de 40 °/s, offset mal calibrado con autoajuste, sesgo de giroscopio con ruido triple, batería baja con compensación y detección de caída.

## Licencia

MIT. Ver [`LICENSE`](LICENSE). La idea original y el diseño mecánico son de [remrc](https://github.com/remrc); este código es una reimplementación independiente.
