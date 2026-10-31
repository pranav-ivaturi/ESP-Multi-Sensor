# ESP Multi Sensor — Telemetry

## Overview

The Telemetry module is a dedicated component of the ESP Multi Sensor project, designed to handle the collection, processing, and monitoring of sensor data across the distributed ESP32-based sensor network.

It complements the Master and Slave modules by providing a dedicated foundation for system-wide telemetry, enabling structured sensor data handling and facilitating communication, monitoring, and future data analysis.

## Project Structure

```text
telemetry/
├── CMakeLists.txt
├── README.md
├── sdkconfig
├── main/
│   ├── CMakeLists.txt
│   └── main.c
└── build/
```

## Hardware Platform

- **Microcontroller:** ESP32-WROOM-32 DevKit
- **Framework:** ESP-IDF
- **ESP-IDF Version:** v6.0.2
- **Target:** ESP32
- **Programming Language:** C
- **Build System:** CMake and Ninja

## Architecture

The Telemetry module is part of the broader ESP Multi Sensor architecture, consisting of:

- **Master:** Central coordination, control, and system-level communication.
- **Slave 1–4:** Distributed sensor acquisition and CAN-based data transmission.
- **Telemetry:** Dedicated subsystem for sensor data monitoring, processing, and system-wide telemetry integration.

The system uses the Controller Area Network (CAN) protocol for communication between the Master and distributed Slave nodes.

## Software Requirements

- Visual Studio Code
- ESP-IDF extension
- ESP-IDF v6.0.2
- ESP-IDF configured PowerShell environment
- CMake
- Ninja build system
- ESP32 toolchain

## Build Instructions

Open an ESP-IDF PowerShell terminal and navigate to the Telemetry directory.

### Configure the ESP32 target

```powershell
idf.py set-target esp32
```

### Build the project

```powershell
idf.py build
```

### Clean the build

```powershell
idf.py fullclean
```

If ESP-IDF cannot clean an incomplete build directory, remove it manually:

```powershell
Remove-Item -Recurse -Force .\build
```

Then run `idf.py build` again.

## Main Application

The application entry point is:

```text
main/main.c
```

The ESP-IDF component is registered through:

```text
main/CMakeLists.txt
```

The root project configuration is defined in:

```text
CMakeLists.txt
```

## Current Status

- [x] Telemetry project initialized
- [x] ESP32 target configured
- [x] ESP-IDF project structure established
- [x] `main.c` established as application entry point
- [x] CMake component configuration established
- [x] Initial ESP-IDF build completed successfully
- [ ] Telemetry functionality implementation
- [ ] Integration with the Master–Slave CAN network
- [ ] Sensor data reception and processing
- [ ] System-wide telemetry monitoring

## Repository

Part of the **ESP Multi Sensor** project.

The Telemetry module is maintained independently within the repository to support modular development, testing, and integration with the existing Master and Slave firmware.
