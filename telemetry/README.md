# ESP Multi Sensor — Telemetry

## Overview

The Telemetry module is a dedicated CAN-to-LoRa communication bridge within the ESP Multi Sensor project.

It receives sensor data frames retransmitted by the Master over the CAN bus, packages them into structured telemetry batches, and forwards them wirelessly through the RYLR993 LoRa module to the Pilot receiver. It also accepts one strictly validated relay-control keepalive path in the reverse direction.

The module operates independently from the Master and Slave firmware, providing a dedicated communication interface for external sensor data monitoring.

## Hardware Platform

| Parameter | Specification |
|---|---|
| Microcontroller | ESP32-WROOM-32 DevKit |
| Framework | ESP-IDF |
| ESP-IDF Version | v6.0.2 |
| Target | ESP32 |
| Language | C |
| Build System | CMake and Ninja |
| CAN Interface | On-chip TWAI |
| CAN Bitrate | 500 kbps |
| LoRa Module | RYLR993 |
| LoRa Frequency | 865 MHz |

## Pin Configuration

### CAN / TWAI

| Signal | ESP32 GPIO |
|---|---|
| CAN TX | GPIO 5 |
| CAN RX | GPIO 4 |

The CAN interface publishes validated relay commands (`0x600`) to Master; all other command IDs are rejected.

### RYLR993 — UART2

| Signal | ESP32 GPIO |
|---|---|
| UART TX | GPIO 17 |
| UART RX | GPIO 16 |
| Baud Rate | 9600 |
| Pilot Address | 2 |
| Pod Address | 1 |

The RYLR993 is connected through UART2 for wireless telemetry transmission.

## System Architecture

The Telemetry module operates alongside the Master and four Slave nodes.

- **Master:** Receives sensor frames from the Slaves and republishes them using dedicated telemetry CAN IDs.
- **Slaves 1–4:** Acquire sensor measurements and transmit structured CAN data frames.
- **Telemetry:** Receives the Master telemetry frames and forwards them through the RYLR993 LoRa module.
- **Pilot:** Receives the wireless telemetry packets for external monitoring and processing.

### Communication Flow

```text
Slave 1 ──┐
Slave 2 ──┤
Slave 3 ──┼── CAN Bus ── Master
Slave 4 ──┘                  │
                             │ IDs 0x500–0x50A
                             ▼
                      CAN Bus (Listen-Only)
                             │
                             ▼
                        Telemetry ESP32
                             │
                      Frame Batching
                             │
                       CRC Generation
                             │
                             ▼
                        UART2 (9600)
                             │
                             ▼
                         RYLR993
                             │
                        LoRa (865 MHz)
                             │
                             ▼
                           Pilot
```

## CAN Configuration

The Telemetry ESP32 accepts only the following Master-generated telemetry frames:

| CAN ID | Source frame |
|---|---|
| `0x500` | Slave 1 — IMU |
| `0x501` | Slave 1 — Gyroscope / Hall |
| `0x502` | Slave 1 — Temperature / Status |
| `0x503` | Slave 2 — ADC |
| `0x509` | Slave 2 — LiDAR |
| `0x50A` | Slave 2 — relay status |
| `0x504` | Slave 3 — Temperature |
| `0x505` | Slave 3 — Distance / Pressure |
| `0x506` | Slave 3 — Status |
| `0x507` | Slave 4 — RTD |
| `0x508` | Slave 4 — Hall |

All accepted frames use standard 11-bit Classical CAN format with an 8-byte payload.

## Telemetry Packet Format

The module batches up to four CAN frames into a single LoRa transmission.

### LM1 packet structure

```text
LM1,SEQ,COUNT,CAN_ID,LENGTH,DATA_HEX[,CAN_ID,LENGTH,DATA_HEX...]*CRC
```

| Field | Description |
|---|---|
| `LM1` | Telemetry protocol identifier and version |
| `SEQ` | Incrementing radio packet sequence number |
| `COUNT` | Number of CAN frames in the batch |
| `CAN_ID` | Three-digit hexadecimal CAN identifier |
| `LENGTH` | CAN payload length (8 bytes) |
| `DATA_HEX` | CAN payload encoded as hexadecimal |
| `CRC` | CRC-16/CCITT-FALSE checksum |

### Example

```text
LM1,1,2,500,8,0010002000300040,501,8,0040005000600070*CRC
```

The CRC is calculated over the packet contents preceding the `*` separator.

## Relay command return protocol

Telemetry accepts only LoRa packets from Pilot address `2` in this format: `LC1,SEQ,STATE*CRC`, where `STATE` is `0` or `1` and `CRC` is CRC-16/CCITT-FALSE over the text before `*`. It rejects bad CRCs, malformed packets, and stale/duplicate sequences. A validated command is forwarded on CAN as `0x600`; Master independently requires continuing keepalives and sends `0x610` to Slave 2. Loss of this path opens the relay.

### Batching Configuration

| Parameter | Value |
|---|---|
| Maximum frames per batch | 4 |
| Batch collection window | 50 ms |
| Radio response timeout | 300 ms |
| Maximum payload buffer | 240 bytes |

## Software Requirements

- Visual Studio Code
- ESP-IDF extension
- ESP-IDF v6.0.2
- ESP-IDF configured PowerShell environment
- CMake
- Ninja
- ESP32 toolchain

## Build Instructions

Open an ESP-IDF PowerShell terminal and navigate to the Telemetry directory.

### Configure target

```powershell
idf.py set-target esp32
```

### Build

```powershell
idf.py build
```

### Clean build

```powershell
idf.py fullclean
```

If the build directory cannot be cleaned:

```powershell
Remove-Item -Recurse -Force .\build
idf.py build
```

## Main Application

| File | Purpose |
|---|---|
| `main/main.c` | CAN reception, filtering, batching, CRC generation, UART transmission |
| `main/CMakeLists.txt` | ESP-IDF component registration |
| `CMakeLists.txt` | Root project configuration |

## Current Status

- [x] Telemetry project initialized
- [x] ESP32 target configured
- [x] ESP-IDF project structure established
- [x] CAN/TWAI initialization implemented
- [x] CAN telemetry bridge and validated relay-command forwarding configured
- [x] Master telemetry ID filtering implemented
- [x] CAN receive queue implemented
- [x] CAN frame batching implemented
- [x] LM1 packet formatting implemented
- [x] CRC-16/CCITT-FALSE implemented
- [x] UART2 communication implemented
- [x] RYLR993 command transmission implemented
- [x] LoRa response handling implemented
- [x] Error and transmission statistics implemented
- [x] ESP-IDF build completed successfully
- [ ] Hardware validation
- [ ] CAN-to-LoRa end-to-end verification
- [ ] Pilot-side packet reception and decoding verification

**Build Status:** Successfully compiled using ESP-IDF v6.0.2. Hardware communication and end-to-end telemetry validation remain pending.

## Repository

Part of the **ESP Multi Sensor** project.

The Telemetry module is maintained independently to support modular development and provide a dedicated wireless communication bridge between the CAN-based sensor network and the Pilot receiver.
