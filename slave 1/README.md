# ESP32 Slave 1: IMU + Hall Sensor + 2× PT100, over CAN

An ESP32 node that reads an **ICM-42688 IMU**, an **HW-484 Hall-effect sensor** and **two PT100 temperature probes (via MAX31865 boards)**, then sends everything as three CAN frames at 20 Hz. The same values are printed on the serial monitor.

## Features

- IMU accelerometer and gyroscope, 6 axes
- Hall sensor as an analog value (A0) and a digital threshold (D0)
- Two PT100 temperatures on one shared SPI bus
- CAN (TWAI) output at 500 kbit/s
- Each sensor fails separately: a missing IMU or Hall sensor no longer stops the program, and the CAN frames carry fault and status bytes

## Hardware

| Part | Qty | Interface |
|---|---:|---|
| ESP32-WROOM-32 DevKit | 1 | n/a |
| ICM-42688 IMU module | 1 | I²C |
| HW-484 Hall sensor module | 1 | Analog + digital |
| MAX31865 RTD board | 2 | SPI |
| PT100 probe | 2 | Connected to MAX31865 |
| 3.3 V CAN transceiver (for example SN65HVD230) | 1 | CAN |
| 120 Ω resistors | 2 | CAN termination, one at each end of the bus |

## Pin Map

| Function | GPIO |
|---|---|
| I²C SDA (IMU) | 21 |
| I²C SCL (IMU) | 22 |
| SPI CLK (both MAX31865) | 18 |
| SPI MOSI (both MAX31865) | 23 |
| SPI MISO (both MAX31865) | 19 |
| MAX31865 #1 CS | 15 |
| MAX31865 #2 CS | 13 |
| Hall A0 (ADC1_CH6) | 34 |
| Hall D0 | 27 |
| CAN TX | 5 |
| CAN RX | 4 |

Use the **GPIO labels** printed on your board, not physical header positions.

## Wiring

### Sensor power

Power the IMU, Hall module, and both MAX31865 boards from the external regulated sensor supply in the system architecture — **not** from the ESP32 `3V3` pin or a GPIO. Connect the external-supply ground to ESP32 GND. The firmware only reads sensor signals; it does not control sensor power.

If the external sensor rail can remain on while the ESP32 is off, use level shifters, bus isolation, or suitable power sequencing so I²C/SPI/ADC signal pins cannot back-power the ESP32.

### ICM-42688 IMU

| IMU pin | ESP32 |
|---|---|
| VCC | External regulated 3.3 V sensor supply |
| GND | Common ground |
| SDA | GPIO 21 |
| SCL | GPIO 22 |
| AD0 | GND (address `0x68`) |

If you tie AD0 to 3V3 the address becomes `0x69`, and you must change `ICM_ADDR` in `main.c`.

### HW-484 Hall sensor

| Hall pin | ESP32 |
|---|---|
| + | External sensor supply (3.3 V if no level shifting) |
| G | Common ground |
| A0 | GPIO 34 |
| D0 | GPIO 27 |

Use a 3.3 V external supply (or level-shift/divide its output) to keep the analog output within the ESP32's ADC range.

### MAX31865 boards (shared SPI)

| MAX31865 pin | Board 1 → ESP32 | Board 2 → ESP32 |
|---|---|---|
| VIN | External regulated 3.3 V sensor supply | External regulated 3.3 V sensor supply |
| GND | Common ground | Common ground |
| CLK | GPIO 18 | GPIO 18 |
| SDI (MOSI) | GPIO 23 | GPIO 23 |
| SDO (MISO) | GPIO 19 | GPIO 19 |
| CS | GPIO 15 | GPIO 13 |

Only the CS wire is different. The default code assumes **3-wire** PT100 probes and a **430 Ω** reference resistor. If your probes are 2-wire or 4-wire, change `RTD_WIRING` at the top of `main.c`, and make sure the board's solder jumpers match.

### CAN

| ESP32 | CAN transceiver |
|---|---|
| GPIO 5 | TXD |
| GPIO 4 | RXD |
| 3V3 | VCC (for a 3.3 V transceiver) |
| GND | GND |

Connect CANH and CANL to the bus, with a 120 Ω resistor at each end. Use a **3.3 V** transceiver. A 5 V transceiver such as the TJA1050 outputs 5 V on RXD, which can damage the ESP32 pin unless you level-shift it.

## CAN Frames

All frames use standard 11-bit IDs, 8 bytes, 500 kbit/s. Multi-byte values are **big-endian**.

| ID | Bytes 0–1 | Bytes 2–3 | Bytes 4–5 | Bytes 6–7 |
|---|---|---|---|---|
| `0x100` | Accel X (int16) | Accel Y (int16) | Accel Z (int16) | Sequence number (uint16) |
| `0x101` | Gyro X (int16) | Gyro Y (int16) | Gyro Z (int16) | Hall A0 raw (uint16) |
| `0x102` | Temp 1 × 100 (int16) | Temp 2 × 100 (int16) | Byte 4: Hall D0, byte 5: RTD 1 fault | Byte 6: RTD 2 fault, byte 7: status |

**Status byte (frame `0x102`, byte 7):** bit 0 = IMU reading OK, bit 1 = Hall ADC reading OK.

**Fault bytes:** `0` means no fault. A non-zero value is the MAX31865 fault register (for example an open or shorted probe wire). It is cleared after each report so it can recover once the wiring is fixed.

### Decoding values

| Value | Conversion |
|---|---|
| Temperature | `raw / 100.0` gives °C (for example `2543` = 25.43 °C) |
| Acceleration | `raw / 2048.0` gives g (default ±16 g range) |
| Gyro | `raw / 16.4` gives °/s (default ±2000 °/s range) |
| Hall A0 | Raw 12-bit ADC count. It is about 1900 with no magnet near the sensor |

## Software Requirements

- ESP-IDF v5.5 or newer (tested layout for v6.0.2)
- No extra libraries. The MAX31865 driver is built into `main.c`.

> **Single-file project:** `main/main.c` is the only source file. **Do not** add the `esp-idf-lib/max31865` component or any separate `max31865` files.

## Project Structure

```text
slave 1/
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
project(esp_slave1)
```

**`main/CMakeLists.txt`**

```cmake
idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES esp_driver_i2c esp_driver_spi
                                     esp_driver_gpio esp_driver_twai
                                     esp_adc)
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
ICM-42688 ready
Dual MAX31865 initialised on shared SPI bus
Hall sensor ready (A0 on GPIO34, D0 on GPIO27)
CAN started at 500 kbit/s
```

Then one line every 50 ms:

```text
#42 | A:12,-8,2041 | G:3,-1,0 | T1:27.35 C (F:0x00) | T2:27.41 C (F:0x00) | Hall:1902 (+2) D0:1 | IMU:ok CAN:ok
```

## Changes Made While Merging

| Area | Change |
|---|---|
| IMU | The data read started at register `0x1F`, but the code expected temperature first. It now starts at `0x1D`, so accel and gyro values land on the right axes |
| IMU | A missing IMU no longer ends the program. The other sensors keep working and the status byte shows the problem |
| I²C and CAN | Switched to the current ESP-IDF drivers (`i2c_master` and the TWAI node driver), since the older ones are deprecated or removed in ESP-IDF 6 |
| RTD driver | The below-zero temperature branch was never used because the wrong value was tested. It is fixed, and the polynomial now has all its terms |
| RTD driver | Faults were being cleared before they could be read. They are now read first, reported, and then cleared |
| RTD driver | Added the 50 Hz noise filter (set `MAX31865_MAINS_50HZ` to 0 for 60 Hz mains) |
| CAN frame 3 | Temperatures are clamped to the int16 range so a bad reading cannot wrap around. The reserved byte is now a status byte |
| Hall | The separate Hall test program is merged into `main.c`, so there is only one `app_main` |
| Structure | The MAX31865 driver is now built into `main.c`, so the whole project is one source file |

## Troubleshooting

| Problem | Likely cause |
|---|---|
| Build error about `esp_twai.h` or a TWAI type | Your ESP-IDF version has a different TWAI API. Paste the error and it can be adapted |
| `ICM-42688 check failed` | Wrong wiring, wrong address (AD0 pin), or the module is not powered at 3.3 V |
| Accel Z about 2048 when flat | Normal. That is 1 g at the default ±16 g range |
| `T1` or `T2` shows odd values with fault `0x..` | Check the probe wires, the wiring mode (`RTD_WIRING`), the reference resistor, and the board jumpers |
| `CAN:ERR` on every line | No CAN transceiver, or no second node on the bus. Frames are only acknowledged when another node is connected |
| Board does not boot with CAN or CS1 connected | GPIO 5 and GPIO 15 are boot-strapping pins. Disconnect those wires, power up, then reconnect |

## Notes

- With nothing else on the CAN bus, the controller will eventually stop sending (bus-off), but the serial output keeps working. To test CAN, connect a second CAN node or a USB-CAN adapter.
- The MAX31865 updates about every 60 ms, so temperature values repeat for a cycle or two at the 50 ms loop rate.
