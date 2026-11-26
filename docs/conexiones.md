# Conexiones

## ESP32 DevKit

| Señal | ESP32 | Destino |
|---|---|---|
| SDA | GPIO21 | MPU6050 SDA, AS5600 SDA |
| SCL | GPIO22 | MPU6050 SCL, AS5600 SCL |
| PWM | GPIO25 | TB6612 PWMA |
| Dirección 1 | GPIO26 | TB6612 AIN1 |
| Dirección 2 | GPIO27 | TB6612 AIN2 |
| Standby | GPIO33 | TB6612 STBY |
| Batería | GPIO34 (ADC1) | punto medio del divisor 100k / 22k |
| Buzzer | GPIO32 | buzzer activo (opcional) |
| LED | GPIO2 | LED integrado |
| 3V3 | 3V3 | VCC de MPU6050, AS5600 y lógica del TB6612 |

- Divisor de batería: `Vbat — 100 kΩ — GPIO34 — 22 kΩ — GND`. Con 12.6 V el pin recibe 2.27 V.
- **AS5600 a 3.3 V.** Algunos módulos tienen un puente 3V3/5V: déjalo en 3V3.
- Los pines se cambian en la sección `PINES` del sketch. Si usas un ESP32-S3/C3, desactiva el Bluetooth (`USE_BLUETOOTH 0`) y elige otro pin ADC.

## PIC18F45K22 / PIC18F4550 (40 pines, a 5 V)

| Señal | PIC18F45K22 | PIC18F4550 | Destino |
|---|---|---|---|
| SDA | RC4 | RB0 | MPU6050 SDA, AS5600 SDA |
| SCL | RC3 | RB1 | MPU6050 SCL, AS5600 SCL |
| PWM (CCP1) | RC2 | RC2 | TB6612 PWMA |
| Dirección 1 | RD0 | RD0 | TB6612 AIN1 |
| Dirección 2 | RD1 | RD1 | TB6612 AIN2 |
| Standby | RD2 | RD2 | TB6612 STBY |
| LED | RD3 | RD3 | LED + resistencia 330 Ω |
| Buzzer | RD4 | RD4 | buzzer activo (opcional) |
| TX | RC6 | RC6 | RX del HC-05 (divisor 1k/2k si tu módulo no tolera 5 V) |
| RX | RC7 | RC7 | TX del HC-05 |
| Batería | RA0/AN0 | RA0/AN0 | punto medio del divisor 100k / 47k |
| Oscilador | interno 64 MHz | cristal 20 MHz + 2×22 pF | — |
| MCLR | 10 kΩ a 5 V | 10 kΩ a 5 V | — |

- Divisor de batería: `Vbat — 100 kΩ — AN0 — 47 kΩ — GND`. Con 12.6 V el pin recibe 4.03 V.
- **Bus I2C a 3.3 V con PIC a 5 V:** los módulos GY-521 y AS5600 traen pull-ups a 3.3 V. El firmware activa los niveles SMBus del MSSP (`CKE = 1`, entrada alta desde 2.1 V), así que el PIC lee bien el bus. **No agregues pull-ups a 5 V**, porque el MPU6050 no tolera 5 V. Si tus módulos no traen pull-ups, pon 4.7 kΩ a 3.3 V. Un módulo convertidor de nivel (BSS138) también funciona.
- **Mídelo antes de conectar el PIC:** con los módulos alimentados y el bus en reposo, SDA y SCL deben marcar unos 3.3 V. Según el fabricante, algunos GY-521 llevan los pull-ups a su VCC de entrada; si ves 5 V, aliméntalos con 3.3 V de un regulador (AMS1117-3.3) o usa un convertidor de nivel.
- Desacopla el PIC con 100 nF en cada par VDD/VSS.

## TB6612FNG

| Pin | Conexión |
|---|---|
| VM | batería (+) con electrolítico de 470–1000 µF a GND |
| VCC | 3.3 V (ESP32) o 5 V (PIC) |
| AO1 / AO2 | motor |
| PWMA, AIN1, AIN2, STBY | microcontrolador |
| GND | tierra común |

Se puede usar un solo canal, o los dos canales en paralelo (A con B) si el motor pide más de 1.2 A continuos.

## Anti-ruido (lo que más falla con motor DC)

Las escobillas generan chispas que se meten al bus I2C. Los síntomas son que el stick "se cae a ratos" o que aparecen errores I2C en `stat`.

1. Capacitor cerámico de **100 nF entre las terminales del motor**, y uno de cada terminal a la carcasa.
2. Cables del motor **trenzados** y lejos de SDA/SCL.
3. Cables I2C **cortos** (menos de 15 cm) y con su propia tierra.
4. Electrolítico grande en VM del TB6612.
5. Si el motor está muy gastado y saca mucha chispa, cámbialo. El README original de remrc menciona este problema.

## Montaje de sensores

- **MPU6050:** cerca del pivote, con un eje paralelo al eje de la rueda (`AX_GYRO`) y otro apuntando hacia arriba cuando el stick está vertical (`AX_UP`). Por defecto: X arriba, Y horizontal en el plano de caída, Z paralelo al eje de la rueda. El comando `imu` muestra qué eje lee +1 g.
- **AS5600:** centrado con el eje del motor, imán diametral a 0.5–3 mm. El comando `stat` indica si el imán se detecta, si está débil o si está demasiado fuerte.
