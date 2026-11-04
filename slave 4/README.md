# ESP32 Slave 4 — Dual PT100 + Hall CAN Node

## Overview

Slave 4 is an ESP32-WROOM-32 based sensor node containing:

1. PT100 RTD #1 through MAX31865
2. PT100 RTD #2 through MAX31865
3. Analog Hall-effect sensor

Slave 4 is part of the **ESP Multi Sensor** Master–Slave CAN architecture.

The existing sensor acquisition behavior has been retained. CAN communication was added around the existing sensor data.

---

## Hardware

### ESP32

- ESP32-WROOM-32 DevKit
- ESP-IDF v6.x
- Built-in TWAI/CAN controller

### Sensors

- 2 × PT100 RTD
- 2 × MAX31865 RTD interface boards
- Analog Hall-effect sensor

### Sensor power

Power both MAX31865 boards and the Hall sensor from the external regulated sensor supply in the system architecture, not from the ESP32 `3V3` pin or a GPIO. Connect the external-supply ground to ESP32 GND. The firmware only reads the sensor interfaces and has no sensor-power control output.

If the external sensor rail can remain on while the ESP32 is off, isolate or sequence the SPI/ADC signal lines to prevent back-powering through the ESP32 input pins.

---

# PT100 / MAX31865 Wiring

Each MAX31865 uses an independent SPI bus.

| MAX31865 | RTD #1 | RTD #2 |
|---|---:|---:|
| VIN | External regulated 3.3 V sensor supply | External regulated 3.3 V sensor supply |
| GND | Common ground | Common ground |
| SCK | GPIO 18 | GPIO 14 |
| MOSI / SDI | GPIO 23 | GPIO 13 |
| MISO / SDO | GPIO 19 | GPIO 27 |
| CS | GPIO 25 | GPIO 26 |
| RDY | Not connected | Not connected |

These are the existing Slave 4 SPI assignments.

### PT100 Configuration

```text
RTD type       : PT100
RTD resistance : 100 Ω nominal
Reference      : 430 Ω
Wiring         : 4-wire
Filter         : 50 Hz
SPI speed      : 1 MHz
```

The MAX31865 devices are configured independently on SPI2 and SPI3.

---

# Hall Sensor

| Hall Sensor | ESP32 |
|---|---|
| OUT | GPIO 34 |
| GND | Common ground |
| VCC | External supply required by the sensor |

GPIO 34 is used as ADC1 channel 6.

The firmware averages **16 ADC readings** for each measurement.

### Important

The ESP32 ADC input must not exceed 3.3 V.

If the Hall sensor is powered from 5 V and its output can exceed the ESP32 ADC limit, use an appropriate voltage divider.

---

# CAN Communication

Slave 4 uses the ESP32's built-in TWAI controller with an external CAN transceiver.

### CAN wiring

| ESP32 | CAN Transceiver |
|---|---|
| GPIO 5 | TXD |
| GPIO 4 | RXD |
| 3.3 V | Logic supply |
| GND | GND |

The transceiver connects to the shared:

```text
CANH
CANL
```

Use **120 Ω termination at both physical ends of the CAN bus**.

---

# CAN Configuration

| Parameter | Value |
|---|---|
| CAN bitrate | **500 kbit/s** |
| Frame type | Standard 11-bit |
| Encoding | **Big-endian** |
| TX | GPIO 5 |
| RX | GPIO 4 |

Slave 4 uses the following CAN identifiers:

```text
0x400 → PT100 data
0x401 → Hall sensor data
```

---

# CAN Frame 0x400 — PT100 Data

Eight-byte payload:

| Byte | Data |
|---:|---|
| 0–1 | RTD1 temperature × 100 |
| 2–3 | RTD2 temperature × 100 |
| 4 | RTD validity status |
| 5–6 | Packet counter |
| 7 | Reserved |

Temperature values are signed 16-bit integers.

Example:

```text
25.34 °C

25.34 × 100 = 2534

0x09E6

CAN:
09 E6
```

### Status byte

```text
Bit 0 → RTD1 valid
Bit 1 → RTD2 valid
```

An invalid RTD is represented using the signed minimum value.

---

# CAN Frame 0x401 — Hall Sensor

Eight-byte payload:

| Byte | Data |
|---:|---|
| 0–1 | Hall raw ADC |
| 2–3 | Hall voltage in mV |
| 4 | Status |
| 5–6 | Packet counter |
| 7 | Reserved |

### Status

```text
Bit 0 → Hall sensor valid
```

All multi-byte values use **big-endian encoding**.

---

# Big-Endian Convention

Slave 4 follows the common CAN encoding standard used throughout the ESP Multi Sensor architecture.

For a 16-bit value:

```text
0x1234
```

the CAN payload is:

```text
Byte 0 = 0x12
Byte 1 = 0x34
```

The Master can therefore decode all slaves using the same method:

```c
uint16_t value =
    ((uint16_t)data[0] << 8) |
    data[1];
```

No special byte-order handling is required for Slave 4.

---

# CAN ID Allocation

The ESP Multi Sensor network currently uses:

| Node | CAN IDs |
|---|---|
| Slave 1 | `0x100–0x102` |
| Slave 2 | `0x200–0x201` |
| Slave 3 | `0x300–0x302` |
| Slave 4 | `0x400–0x401` |
| Master | Receives / processes |

This gives each slave its own ID range and makes CAN debugging easier.

---

# Data Flow

```text
PT100 #1 ──┐
PT100 #2 ──┤
           │
Hall ──────┼──► Slave 4 ESP32
           │          │
                      ▼
                 Local processing
                      │
                      ▼
                  CAN/TWAI
                      │
                      ▼
                 CAN Transceiver
                      │
                      ▼
                 Shared CAN Bus
                      │
                      ▼
                    MASTER
```

---

# Main Program Flow

```text
Start
  │
  ▼
Initialize RTD #1
  │
  ▼
Initialize RTD #2
  │
  ▼
Initialize Hall ADC
  │
  ▼
Initialize CAN
  │
  ▼
Read RTD #1
  │
  ├──────────────┐
  ▼              │
Read RTD #2      │
  │              │
  ▼              │
Read Hall        │
  │              │
  └──────┬───────┘
         ▼
   Transmit CAN
   0x400 / 0x401
         │
         ▼
    Wait 500 ms
         │
         └──────► Repeat
```

The existing sensor loop runs every 500 ms.

---

# Software Structure

```text
slave 4/
│
├── CMakeLists.txt
├── sdkconfig
├── README.md
│
└── main/
    ├── CMakeLists.txt
    ├── main.c
    └── idf_component.yml
```

The MAX31865 library is provided through the ESP-IDF component dependency.

---

# CAN Network Standard

All ESP Multi Sensor nodes use the same CAN communication convention:

```text
CAN bitrate        : 500 kbit/s
CAN frame type     : Standard 11-bit
CAN physical bus   : CANH / CANL
CAN data encoding  : BIG-ENDIAN
CAN TX              : GPIO 5
CAN RX              : GPIO 4
```

The purpose of using one encoding convention is to make the Master firmware simpler and eliminate slave-specific byte-order decoding.

---

The VL53L4CD interface and CAN frame have moved to Slave 2. Slave 4 now operates independently as the PT100 and Hall node.
