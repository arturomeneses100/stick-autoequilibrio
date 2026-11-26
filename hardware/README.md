# Hardware

## Piezas impresas en 3D

El diseño mecánico (cuerpo del stick, rueda de inercia y soportes) es de **remrc**:

- Modelo 3D: [Cults3D — One axis self balancing stick (DC motor)](https://cults3d.com/:920970)
- Proyecto original: [github.com/remrc/One-Axis-Self-Balancing-Stick-DC](https://github.com/remrc/One-Axis-Self-Balancing-Stick-DC)

Descarga los STL desde Cults3D: no se redistribuyen aquí para respetar la licencia del autor.

Sugerencias de impresión: PLA o PETG, 3 perímetros, 30–40 % de relleno en el cuerpo. Para la rueda, más relleno o tornillos/tuercas en el borde para aumentar la inercia: una rueda con más inercia en el aro satura menos.

## Lista de material

| Cant. | Componente | Notas |
|---|---|---|
| 1 | ESP32 DevKit **o** PIC18F45K22 / PIC18F4550 | PIC: más zócalo, regulador 5 V, cristal 20 MHz solo para el 4550 |
| 1 | MPU6050 (GY-521) | I2C 0x68 |
| 1 | AS5600 con imán diametral | I2C 0x36, en modo 3.3 V |
| 1 | TB6612FNG | 1.2 A continuos por canal |
| 1 | Motor DC 12 V con escobillas | el del diseño original |
| 1 | LiPo 3S 500 mAh | con conector y alarma de voltaje recomendada |
| 1 | HC-05 | opcional, para ajustar por Bluetooth con PIC |
| 1 | Buzzer activo 5 V / 3.3 V | opcional |
| 3 | Capacitor cerámico 100 nF | en el motor (anti-ruido) |
| 1 | Electrolítico 470–1000 µF 25 V | en VM del TB6612 |
| 1 | Resistencia 100 kΩ + 22 kΩ (ESP32) o 47 kΩ (PIC) | divisor de batería |
| 1 | Interruptor | en la línea de batería |

## Datos para calcular las ganancias

Para obtener ganancias LQR de tu stick con `sim/modelo_lqr.py`, mide o estima:

- `--m`: masa total en kg (pesa el stick armado con batería)
- `--l`: distancia del pivote (la esquina) al centro de masa, en m. Equilibra el stick acostado sobre un lápiz para encontrar el centro de masa.
- `--jw`: inercia de la rueda en kg·m². Anillo: `masa · radio²`; disco: `½ · masa · radio²`.
- `--kt`, `--r`: constante de par y resistencia del motor (hoja de datos, o `r` medida con multímetro y `kt ≈ V / (velocidad sin carga en rad/s)`).
