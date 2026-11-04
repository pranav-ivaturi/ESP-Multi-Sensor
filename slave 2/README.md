# ESP32 Slave 2 — ADS1113 + VL53L4CD CAN Telemetry

## Overview

Slave 2 is an ESP32-WROOM-32 based sensor node for measuring an analog input through an ADS1113 16-bit ADC and hosting the VL53L4CD LiDAR interface relocated from Slave 4. It transmits both data sets to the Master through CAN.

The node performs:

1. ADS1113 ADC acquisition
2. Voltage conversion and external voltage-divider compensation
3. VL53L4CD LiDAR I²C interface
4. CAN/TWAI initialization
5. Periodic CAN transmission

Slave 2 is part of the **ESP Multi Sensor** distributed Master–Slave CAN architecture.

---

## Hardware

### ESP32

- ESP32-WROOM-32 DevKit
- ESP-IDF v6.x
- Built-in TWAI/CAN controller

### ADS1113

- 16-bit ADC
- I²C interface
- I²C address: `0x48`
- Differential measurement: `AIN0 - AIN1`

### VL53L4CD LiDAR

- I²C address: `0x29`
- Shares the ADS1113 I²C bus; the devices use distinct addresses (`0x29` and `0x48`)
- Initialised with the VL53L4CD Ultra Lite Driver register sequence and ranged continuously with a 50 ms timing budget

---

## Wiring

### ADS1113 → ESP32

| ADS1113 | ESP32 |
|---|---|
| VDD | 3.3 V |
| GND | GND |
| SDA | GPIO 21 |
| SCL | GPIO 22 |
| ADDR | GND |
| AIN1 | GND |
| AIN0 | Voltage-divider output |

The current firmware uses I²C at **400 kHz**.

### VL53L4CD → ESP32

| VL53L4CD | ESP32 |
|---|---|
| VCC | 3.3 V |
| GND | GND |
| SDA | GPIO 21 |
| SCL | GPIO 22 |

The relocated LiDAR shares I²C controller 0 with the ADS1113 at 400 kHz. Their distinct addresses allow both sensors to operate on the existing GPIO 21/22 bus.

### Input Voltage Divider

```text
INPUT
  │
  │
  └── 10 kΩ ──────┬────── AIN0
                  │
                 20 kΩ
                  │
                 GND
```

The firmware compensates for the divider using:

```text
V_ADC = V_INPUT / 1.5

V_INPUT = V_ADC × 1.5
```

The maximum theoretical input corresponding to the ADS1113's ±2.048 V range is approximately 3.072 V with this divider.

---

# CAN Communication

Slave 2 communicates with the Master using the ESP32's built-in TWAI controller and an external CAN transceiver.

### CAN wiring

| ESP32 | CAN Transceiver |
|---|---|
| GPIO 5 | TXD |
| GPIO 4 | RXD |
| 3.3 V | Logic supply |
| GND | GND |

The CAN bus connects to:

```text
CANH
CANL
```

Use **120 Ω termination at both physical ends of the CAN bus**.

---

## CAN Configuration

| Parameter | Value |
|---|---|
| CAN bitrate | **500 kbit/s** |
| CAN IDs | **0x200** (ADS1113), **0x201** (LiDAR) |
| Frame type | Standard 11-bit |
| Data length | 8 bytes |
| Encoding | **Big-endian** |
| Transmission interval | ~100 ms |

The firmware uses GPIO 5 for TX, GPIO 4 for RX and a 500000 bit/s bitrate.

---

# CAN Frame — ID 0x200

Slave 2 uses two 8-byte CAN frames.

### Big-endian payload

| Byte | Data |
|---:|---|
| 0–1 | Raw ADC value |
| 2–3 | ADC voltage in mV |
| 4–5 | Original input voltage in mV |
| 6–7 | Packet counter |

All multi-byte values are transmitted **MSB first**.

Example:

```text
Value = 0x1234

Byte 0 = 0x12
Byte 1 = 0x34
```

This was changed from the original little-endian implementation so that Slave 2 now uses the same CAN encoding convention as the other slaves.

---

# CAN Frame — ID 0x201

| Bytes | Field | Type |
|---|---|---|
| 0–1 | LiDAR distance in mm; `-1` when unavailable | `int16_t` |
| 2–3 | Packet counter | `uint16_t` |
| 4 | Status; bit 0 means a valid distance | `uint8_t` |
| 5–7 | Reserved | — |

The VL53L4CD hardware interface has moved from Slave 4. Slave 2 validates the module identity during startup, runs continuous ranging, and transmits each valid millimetre measurement. It sends `-1` with status `0x00` if the module is absent, a measurement times out, or the sensor reports an invalid range result.

---

## CAN Data Interpretation

### Bytes 0–1 — Raw ADC

Signed 16-bit ADC result.

```text
raw = (byte0 << 8) | byte1
```

### Bytes 2–3 — ADC Voltage

Unsigned 16-bit value in millivolts.

```text
ADC_voltage_mV =
    (byte2 << 8) | byte3
```

### Bytes 4–5 — Original Input Voltage

Unsigned 16-bit value in millivolts.

```text
Input_voltage_mV =
    (byte4 << 8) | byte5
```

### Bytes 6–7 — Packet Counter

Unsigned 16-bit counter.

```text
counter =
    (byte6 << 8) | byte7
```

The counter allows the Master to identify missing or repeated packets.

---

## CAN Architecture

```text
              SLAVE 2
                 │
        ┌────────▼────────┐
        │ ADS1113 + LiDAR │
        │ Shared I²C0     │
        └────────┬────────┘
                 │
            ESP32-WROOM-32
                 │
          TWAI / CAN GPIO
           TX = GPIO 5
           RX = GPIO 4
                 │
          CAN Transceiver
                 │
              CANH/CANL
                 │
            Shared CAN Bus
                 │
              MASTER
```

---

## Software Structure

```text
slave 2/
│
├── CMakeLists.txt
├── sdkconfig
├── README.md
│
└── main/
    ├── CMakeLists.txt
    └── main.c
```

---

## Main Program Flow

```text
Start
  │
  ▼
Initialize I²C
  │
  ▼
Detect ADS1113
  │
  ▼
Detect VL53L4CD
  │
  ▼
Initialize CAN/TWAI
  │
  ▼
Start measurement
  │
  ▼
Read ADS1113
  │
  ▼
Convert ADC → voltage
  │
  ▼
Apply divider compensation
  │
  ▼
Pack data in BIG-ENDIAN format
  │
  ▼
Transmit CAN ID 0x200
  │
  ▼
Transmit CAN ID 0x201
  │
  ▼
Increment packet counter
  │
  ▼
Wait ~100 ms
  │
  └──────────────► Repeat
```

---

## Dependencies

`main/CMakeLists.txt` requires:

```cmake
idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES
        esp_driver_i2c
        esp_driver_gpio
        esp_driver_twai
)
```

---

## Verification

A successful startup should show:

```text
ESP32 SLAVE 2
ADS1113 + VL53L4CD + CAN
I2C initialized
ADS1113 detected at 0x48
VL53L4CD ranging at 0x29 on the ADS1113 I2C bus
CAN initialized
CAN TX GPIO : 5
CAN RX GPIO : 4
CAN bitrate : 500000 bit/s
CAN IDs     : 0x200 / 0x201
System initialized
```

Measurements are then periodically transmitted over CAN.

---

## CAN Network Convention

All ESP Multi Sensor slaves use:

```text
CAN bitrate: 500 kbit/s
Standard 11-bit identifiers
BIG-ENDIAN multi-byte data
```

Slave 2:

```text
0x200
0x201
```

This common encoding allows the Master to use one decoding method for all sensor nodes.
