# ESP32 sample with BMI160
The ultra-tiny, low-power, low-noise, 16-bit inertial measurement unit BMI160 is a IMU for AR use cases, indoor navigation and gaming. Highly accurate.

## Wiring (SPI)

| PIN | NAME | CONNECT TO |
|-----|------|------------|
| VIN |      |            |
| 3v3 |      |   3v3      |
| GND |      |   GND      |
| SCL |  CLK |    18      |
| SDA | MOSI |    23      |
| CS  |      |    21      |
| SAO | MISO |    19      |

## Schema
![wiring diagram](/schema/wiring-diagram.png)

## Pulseview
![device config](/doc/pulseview-device-config.png)
![pulseview config](/doc/pulseview-config.png)