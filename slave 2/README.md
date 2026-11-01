# ESP32 Slave 2 — ADS1113 CAN Telemetry

## Overview

Slave 2 is an ESP32-WROOM-32 based sensor node for measuring an analog input through an ADS1113 16-bit ADC and transmitting the measured data to the Master through the CAN bus.

The node performs:

1. ADS1113 ADC acquisition
2. Voltage conversion
3. External voltage-divider compensation
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
| CAN ID | **0x200** |
| Frame type | Standard 11-bit |
| Data length | 8 bytes |
| Encoding | **Big-endian** |
| Transmission interval | ~100 ms |

The firmware uses GPIO 5 for TX, GPIO 4 for RX and a 500000 bit/s bitrate.

---

# CAN Frame — ID 0x200

Slave 2 uses one 8-byte CAN frame.

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
        │     ADS1113     │
        │   16-bit ADC    │
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
ADS1113 + CAN
I2C initialized
ADS1113 detected at 0x48
CAN initialized
CAN TX GPIO : 5
CAN RX GPIO : 4
CAN bitrate : 500000 bit/s
CAN ID      : 0x200
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
```

This common encoding allows the Master to use one decoding method for all sensor nodes.