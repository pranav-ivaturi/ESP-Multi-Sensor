# Pilot — ESP Multi Sensor

## Overview

The `pilot` project is the pilot-side telemetry receiver for the ESP Multi Sensor system. It receives wireless telemetry transmitted by the master-side system through a LoRa module connected to the ESP32.

The firmware processes incoming LoRa packets, validates their integrity using CRC16, extracts telemetry information, and outputs the received sensor data and communication statistics in JSON format.

## Hardware Requirements

- ESP32-WROOM-32 development board
- LoRa transceiver module with UART interface
- USB cable for programming and serial monitoring
- Appropriate power supply for the ESP32 and LoRa module

## Communication Configuration

| Parameter | Configuration |
|---|---|
| Microcontroller | ESP32 |
| LoRa UART | UART2 |
| TX GPIO | GPIO 17 |
| RX GPIO | GPIO 16 |
| Baud rate | 9600 |
| Pilot address | 2 |
| Telemetry CAN ID range | 0x500–0x50A |
| Maximum radio payload | 240 bytes |
| Radio response timeout | 150 ms |
| Telemetry batch size | 4 |
| Batch wait interval | 50 ms |

## Software Requirements

- ESP-IDF v6.0.2
- ESP-IDF-supported Python environment
- CMake and Ninja build system
- ESP32 target configuration

## Project Structure

```text
pilot/
├── main/
│   ├── main.c
│   └── CMakeLists.txt
├── CMakeLists.txt
├── sdkconfig
└── README.md
```

## Firmware Functionality

### 1. LoRa UART Communication

Initializes UART2 with the configured TX and RX GPIO pins and baud rate to communicate with the LoRa module.

### 2. Telemetry Reception

Receives incoming radio data and processes telemetry packets within the configured payload size and response timeout.

### 3. CRC16 Validation

Uses CRC16 validation to verify received packet integrity and identify corrupted or invalid packets before processing.

### 4. Telemetry Parsing

Extracts telemetry messages associated with CAN IDs ranging from `0x500` to `0x50A` and processes the received data, including Slave 2 relay status.

### 5. JSON Output

Formats processed telemetry and communication information into JSON output for serial monitoring and debugging.

### 6. Communication Statistics

Reports link and reception statistics to help monitor the pilot-side wireless communication system.

### 7. Relay GUI bridge

The USB-serial GUI sends `RELAY,1` / `RELAY,ON` or `RELAY,0` / `RELAY,OFF`. The Pilot sends CRC-protected LoRa keepalives (`LC1,SEQ,STATE*CRC`) every 250 ms, but an ON command expires after 1 second unless the GUI continues to refresh it. It prints `relay_status` JSON from CAN ID `0x50A`; treat a requested ON state as successful only when `output_on` and `feedback_closed` are both true and `faults` is zero.

## Building the Project

Open the ESP-IDF PowerShell terminal and navigate to the project directory:

```powershell
cd "C:\Users\Prana\Documents\LOOP\ESP Multi Sensor\pilot"
```

Build the firmware:

```powershell
idf.py build
```

A successful build generates:

```text
build/pilot.bin
```

## Flashing the ESP32

Identify the ESP32's COM port and flash the firmware:

```powershell
idf.py -p COM3 flash
```

Replace `COM3` with the actual port assigned to your ESP32.

## Serial Monitoring

To view received telemetry and diagnostic output:

```powershell
idf.py -p COM3 monitor
```

To exit the serial monitor, press:

```text
Ctrl + ]
```

## Troubleshooting

| Issue | Solution |
|---|---|
| `LINE_MAX` redefinition error | Use `LORA_LINE_MAX` instead of the system-defined `LINE_MAX`. |
| UART header or driver errors | Ensure `esp_driver_uart` and `esp_driver_gpio` are included in `PRIV_REQUIRES` in `main/CMakeLists.txt`. |
| `No SOURCES given to target: __idf_main` | Verify `main.c` exists and is referenced correctly in CMake. |
| Flashing fails | Check the COM port, USB connection, and ESP32 boot mode. |
| No LoRa data received | Verify UART TX/RX wiring, baud rate, power supply, and transmitter configuration. |
| CRC validation failures | Verify packet formatting and ensure transmitter and receiver use the same CRC16 implementation. |

## Notes

- The main source file must remain named `main.c`.
- UART2 is dedicated to LoRa communication.
- The pilot address is configured as `2`.
- The firmware is built for the ESP32 target using ESP-IDF.
- Successful compilation does not guarantee wireless communication; the LoRa transmitter and receiver must share compatible configurations.

## Project Context

This project belongs to the **ESP Multi Sensor** repository and serves as the pilot-side receiver component of the larger ESP32-based multi-sensor telemetry architecture.
