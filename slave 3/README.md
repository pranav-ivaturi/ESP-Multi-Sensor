# ESP32 Slave 3: Dual IR Temperature + LiDAR + Pressure, over CAN

An ESP32 node that reads **two GY-906 (MLX90614) infrared temperature sensors, one VL53L0X laser distance sensor, and one hydraulic pressure sensor**, then sends the readings as three CAN frames at 10 Hz. The same values are printed on the serial monitor.

## Features

- Two non-contact temperature sensors (ambient and object temperature each)
- Laser distance in millimetres
- Pressure in pascals (0 to 1 MPa)
- CAN (TWAI) output at 500 kbit/s, with a status frame that reports which sensors are working
- Both GY-906 sensors keep their **default address (0x5A)**, since each one sits on its own I²C bus
- Startup self-check that reports which sensors were found

## Hardware

| Part | Qty | Interface |
|---|---:|---|
| ESP32-WROOM-32 DevKit | 1 | n/a |
| GY-906 (MLX90614) | 2 | I²C |
| VL53L0X module | 1 | I²C |
| Hydraulic pressure sensor, 0.5–4.5 V output, 0–1 MPa | 1 | Analog |
| 10 kΩ and 20 kΩ resistors | 1 each | Voltage divider |
| External 5 V supply (for the pressure sensor) | 1 | n/a |
| 3.3 V CAN transceiver (for example SN65HVD230) | 1 | CAN |
| 120 Ω resistors | 2 | CAN termination, one at each end of the bus |

## Pin Map

| Function | GPIO |
|---|---|
| I²C bus 0 SDA (GY-906 #1, VL53L0X) | 21 |
| I²C bus 0 SCL (GY-906 #1, VL53L0X) | 22 |
| I²C bus 1 SDA (GY-906 #2) | 25 |
| I²C bus 1 SCL (GY-906 #2) | 26 |
| Pressure ADC (ADC1_CH6) | 34 |
| CAN TX | 5 |
| CAN RX | 4 |

Use the **GPIO labels** printed on your board, not physical header positions.

## I²C Addresses

| Device | Bus | Address |
|---|---|---|
| GY-906 #1 | 0 | `0x5A` |
| VL53L0X | 0 | `0x29` |
| GY-906 #2 | 1 | `0x5A` |

The two GY-906 modules share an address, which is fine because they are on separate buses.

## Wiring

### Sensor power

Power both GY-906 modules, the VL53L0X, and the pressure sensor from their required external regulated rails in the system architecture. Do not use the ESP32 `3V3` pin or a GPIO as a sensor supply. All external-supply grounds must join ESP32 GND, and every ESP32 input must remain within the 0–3.3 V range.

If an external sensor rail can remain on while the ESP32 is off, isolate or sequence the sensor signal lines to prevent back-powering through the ESP32 input pins.

### Bus 0: GY-906 #1 and VL53L0X

| Module pin | ESP32 |
|---|---|
| VCC / VIN | External regulated sensor supply compatible with the module |
| GND | Common ground |
| SDA | GPIO 21 |
| SCL | GPIO 22 |

Both modules connect to the same GPIO 21 and GPIO 22 lines. On the VL53L0X, leave GPIO1 unconnected and keep XSHUT at its module default (or tie it to 3V3).

### Bus 1: GY-906 #2

| Module pin | ESP32 |
|---|---|
| VCC / VIN | External regulated sensor supply compatible with the module |
| GND | Common ground |
| SDA | GPIO 25 |
| SCL | GPIO 26 |

### Pressure sensor

| Sensor pin | Connection |
|---|---|
| VCC | External 5 V |
| GND | ESP32 GND (common ground) |
| OUT | Voltage divider, then GPIO 34 |

**Voltage divider** (the sensor outputs up to 4.5 V, but the ESP32 ADC must stay at or below 3.3 V):

```text
Pressure OUT ──[ 10 kΩ ]──┬── GPIO 34
                          │
                       [ 20 kΩ ]
                          │
                         GND
```

```text
V_ADC = V_SENSOR × 20k / (10k + 20k) = V_SENSOR / 1.5
4.5 V → 3.0 V
```

**Common ground is required.** The external 5 V supply's ground, the pressure sensor ground, and the ESP32 ground must all be connected together.

### CAN

| ESP32 | CAN transceiver |
|---|---|
| GPIO 5 | TXD |
| GPIO 4 | RXD |
| 3V3 | VCC (for a 3.3 V transceiver) |
| GND | GND |

Connect CANH and CANL to the bus. Put a 120 Ω resistor at **each end of the bus**, not at every node.

Use a **3.3 V** transceiver. A 5 V transceiver such as the TJA1050 outputs 5 V on RXD, which can damage the ESP32 pin unless you level-shift it.

## CAN Frames

All frames use standard 11-bit IDs, 8 bytes, 500 kbit/s, sent every 100 ms. Multi-byte values are **big-endian**.

| ID | Bytes | Content |
|---|---|---|
| `0x300` | 0–1 | GY-906 #1 ambient temperature × 100 (int16) |
| | 2–3 | GY-906 #1 object temperature × 100 (int16) |
| | 4–5 | GY-906 #2 ambient temperature × 100 (int16) |
| | 6–7 | GY-906 #2 object temperature × 100 (int16) |
| `0x301` | 0–1 | LiDAR distance in mm (uint16) |
| | 2–5 | Pressure in Pa (uint32) |
| | 6–7 | Sequence number (uint16, wraps around) |
| `0x302` | 0 | Status bits (see below) |
| | 1–7 | Reserved (0) |

**Status byte (frame `0x302`, byte 0):**

| Bit | Meaning when set |
|---|---|
| 0 | GY-906 #1 reading OK |
| 1 | GY-906 #2 reading OK |
| 2 | LiDAR reading OK |
| 3 | Pressure ADC reading OK |

**Invalid values:** a sensor that fails is marked in the data as well as in the status byte.

| Field | Invalid value |
|---|---|
| Any temperature | `-32768` (`0x8000`) |
| LiDAR distance | `65535` (`0xFFFF`) |
| Pressure | `4294967295` (`0xFFFFFFFF`) |

### Decoding values

| Value | Conversion |
|---|---|
| Temperature | `raw / 100.0` gives °C (for example `2741` = 27.41 °C) |
| Distance | Already in mm |
| Pressure | Already in Pa (`raw / 1000.0` gives kPa) |

Temperatures are clamped to ±327.67 °C so they fit in 16 bits. The GY-906 object sensor can measure higher than that, so very hot targets will read 327.67 °C.

### Choosing CAN IDs across slaves

This slave uses the `0x300` range. Slave 1 uses `0x100`–`0x102`. Keep each slave in its own range so several can share one bus.

## Serial Output

One line every 100 ms:

```text
PACKET | T1_AMB=27.41 C | T1_OBJ=31.72 C | T2_AMB=27.38 C | T2_OBJ=29.64 C | LIDAR=183 mm | PRESSURE=245000 Pa | CAN=ok
```

| Field | Meaning | Unit | Error value |
|---|---|---|---|
| `T1_AMB`, `T2_AMB` | Ambient temperature | °C | -999.00 |
| `T1_OBJ`, `T2_OBJ` | Object temperature | °C | -999.00 |
| `LIDAR` | Distance | mm | -1 |
| `PRESSURE` | Pressure | Pa | -1 |
| `CAN` | `ok` if all three frames were queued, otherwise `ERR` | n/a | `ERR` |

## Software Requirements

- ESP-IDF v5.5 or newer (the project builds on v6.0.2)
- No extra libraries

## Project Structure

```text
slave 3/
├── CMakeLists.txt
├── README.md
└── main/
    ├── CMakeLists.txt
    └── main.c
```

**Top-level `CMakeLists.txt`**

```cmake
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp_slave3)
```

**`main/CMakeLists.txt`** (note the added `esp_driver_twai` for CAN)

```cmake
idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES esp_driver_i2c esp_driver_gpio
                                     esp_driver_twai esp_adc)
```

## Build and Flash

```bash
idf.py set-target esp32
idf.py build
idf.py -p <PORT> flash monitor
```

Replace `<PORT>` with your serial port (for example `COM5`). Press `Ctrl+]` to exit the monitor.

## Expected Output

At startup:

```text
ADC calibration enabled
GY-906 #1 (bus 0) detected at 0x5A
GY-906 #2 (bus 1) detected at 0x5A
VL53L0X   (bus 0) detected at 0x29
VL53L0X initialised
CAN started at 500 kbit/s
```

## Troubleshooting

| Problem | Likely cause |
|---|---|
| Build error: `esp_twai.h` not found | `esp_driver_twai` is missing from `PRIV_REQUIRES` in `main/CMakeLists.txt` |
| A sensor says "NOT detected" | Check VCC, GND, SDA and SCL wires, and that SDA/SCL are not swapped |
| Temperature shows -999.00 | Sensor not responding on its bus, or loose wiring |
| `LIDAR=-1` | VL53L0X not initialised, or nothing within range (about 2 m max) |
| Pressure stuck at 0 | Missing common ground, sensor not powered at 5 V, or divider wired wrongly |
| Pressure stuck at max | Divider missing or wired wrongly, so the ADC is saturated |
| `CAN=ERR` on every line | No CAN transceiver, or no second node on the bus. Frames are only acknowledged when another node is connected |
| Board does not boot with the CAN wires connected | GPIO 5 is a boot-strapping pin. Disconnect the CAN TX wire, power up, then reconnect |

## Notes

- With nothing else on the CAN bus, the controller will eventually stop sending (bus-off), but the serial output keeps working. To test CAN, connect a second CAN node or a USB-CAN adapter.
- The GY-906 sensors update only a few times per second by default, so temperature values repeat across some of the 100 ms cycles.
- The pressure reading is least accurate near the top of the range (about 3 V at the ADC pin). Compare it with a reference gauge if you can.
- The VL53L0X code is a **simplified driver** that skips ST's full calibration sequence. Distances may be rough. Check them against a ruler.
- The loop runs at 10 Hz. Change `LOOP_PERIOD_MS` in `main.c` to send faster or slower.
