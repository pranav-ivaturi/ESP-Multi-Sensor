# ESP Multi Sensor — Master

## 1. Overview

The Master is the central CAN communication node of the ESP Multi Sensor system. It receives sensor data from four ESP32 slave nodes, decodes the incoming CAN frames, prints sensor readings to the serial monitor, monitors slave communication status, and republishes the received data using Master-assigned CAN identifiers for the Telemetry ESP.

The firmware runs on an ESP32-WROOM-32 DevKit using ESP-IDF and FreeRTOS.

### Implemented functionality

- Initializes the ESP32 TWAI controller.
- Receives standard 11-bit CAN frames at 500 kbit/s.
- Processes received messages using a FreeRTOS queue.
- Decodes sensor frames from all four slave nodes.
- Prints decoded sensor data to the serial monitor.
- Tracks slave communication using receive timestamps.
- Detects ONLINE and OFFLINE communication states.
- Republishes recognized slave frames using Master telemetry IDs `0x500–0x509`.
- Tracks successful and failed Master telemetry transmissions.

**Hardware control and command transmission to slaves are not implemented.**

---

## 2. Hardware Specifications

| Parameter | Specification |
|---|---|
| Microcontroller | ESP32-WROOM-32 DevKit |
| Framework | ESP-IDF v6.x |
| Programming language | C |
| RTOS | FreeRTOS |
| CAN controller | ESP32 TWAI |
| CAN transceiver | External CAN transceiver |
| CAN bitrate | 500 kbit/s |
| CAN identifier | Standard 11-bit |
| CAN frame type | Classic CAN |
| Data length | 8 bytes |
| Byte order | Big-endian |

### 2.1 CAN Wiring

| ESP32 GPIO | Connection |
|---|---|
| GPIO 5 | CAN transceiver TXD |
| GPIO 4 | CAN transceiver RXD |
| 3.3 V | Transceiver supply, if supported |
| GND | Transceiver ground |

The ESP32 contains an integrated TWAI controller but requires an external physical CAN transceiver to connect to CANH and CANL.

### 2.2 CAN Bus Requirements

- All nodes must operate at 500 kbit/s.
- CANH and CANL are shared across all five CAN nodes.
- All nodes must share a common ground.
- Install 120 Ω termination resistors at the two physical ends of the bus.
- With power disconnected, the resistance between CANH and CANL should be approximately 60 Ω when both terminators are present.
- Keep CAN stubs short and maintain proper wiring.

---

## 3. System Architecture

```text
                 SHARED CAN BUS (500 kbit/s)
                            |
       +----------+---------+---------+----------+
       |          |         |         |          |
   +--------+ +--------+ +--------+ +--------+
   |Slave 1 | |Slave 2 | |Slave 3 | |Slave 4 |
   |        | |        | |        | |        |
   |IMU     | |ADC     | |IR Temp | |PT100   |
   |Gyro    | |ADS1115 | |LiDAR   | |Hall    |
   |Hall    | |        | |Pressure| |LiDAR   |
   |PT100x2 | |        | |        | |        |
   +--------+ +--------+ +--------+ +--------+
       |          |         |         |
       +----------+---------+---------+
                            |
                     +-------------+
                     |   MASTER    |
                     |             |
                     | TWAI RX     |
                     | FreeRTOS   |
                     | Queue      |
                     | Decoding   |
                     | Serial Log |
                     | Status     |
                     | TWAI TX    |
                     +-------------+
                            |
                 Master CAN IDs
                    0x500–0x509
                            |
                     +-------------+
                     | TELEMETRY  |
                     | ESP32       |
                     |             |
                     | CAN RX      |
                     | UART2      |
                     +-------------+
                            |
                         RYLR993
                       LoRa Module
                            |
                            v
                     Pilot Receiver
```

The Master performs two CAN operations:

1. **Receive:** Reads sensor frames from the four slaves using their original CAN identifiers.
2. **Republish:** Transmits the same eight-byte data payload using corresponding Master telemetry identifiers.

The Telemetry ESP receives the Master-republished frames and forwards them over LoRa.

---

## 4. CAN Protocol

All currently supported slave frames use:

- Standard 11-bit CAN identifiers.
- Classic CAN frames.
- Eight-byte payloads.
- Big-endian encoding for multibyte values.

### 4.1 Slave CAN ID Mapping

| Slave | CAN ID | Data |
|---|---|---|
| Slave 1 | `0x100` | Accelerometer X, Y, Z and sequence |
| Slave 1 | `0x101` | Gyroscope X, Y, Z and Hall ADC |
| Slave 1 | `0x102` | Temperatures, Hall digital output, RTD faults, status |
| Slave 2 | `0x200` | ADC raw value, voltages, packet counter |
| Slave 3 | `0x300` | Two IR sensors' ambient and object temperatures |
| Slave 3 | `0x301` | LiDAR distance, pressure, sequence |
| Slave 3 | `0x302` | Sensor status flags |
| Slave 4 | `0x400` | PT100 temperatures, status, sequence |
| Slave 4 | `0x401` | Hall raw value, voltage, status, sequence |
| Slave 4 | `0x402` | LiDAR distance, sequence, status |

### 4.2 Master Telemetry CAN ID Mapping

The Master maps the original slave CAN identifiers to its own telemetry identifiers.

| Incoming Slave ID | Transmitted Master ID |
|---|---|
| `0x100` | `0x500` |
| `0x101` | `0x501` |
| `0x102` | `0x502` |
| `0x200` | `0x503` |
| `0x300` | `0x504` |
| `0x301` | `0x505` |
| `0x302` | `0x506` |
| `0x400` | `0x507` |
| `0x401` | `0x508` |
| `0x402` | `0x509` |

The Master republishes the original eight-byte data payload without modifying its contents.

Only recognized identifiers with an eight-byte payload are eligible for telemetry retransmission.

---

## 5. CAN Data Frame Layouts

All multibyte values use big-endian encoding. Signed values use `int16_t` where specified.

### 5.1 Slave 1

**ID `0x100` — Accelerometer**

| Bytes | Field | Type |
|---|---|---|
| 0–1 | AX | `int16_t` |
| 2–3 | AY | `int16_t` |
| 4–5 | AZ | `int16_t` |
| 6–7 | Sequence number | `uint16_t` |

**ID `0x101` — Gyroscope and Hall ADC**

| Bytes | Field | Type |
|---|---|---|
| 0–1 | GX | `int16_t` |
| 2–3 | GY | `int16_t` |
| 4–5 | GZ | `int16_t` |
| 6–7 | Hall ADC raw value | `uint16_t` |

**ID `0x102` — Temperature and Status**

| Bytes | Field |
|---|---|
| 0–1 | Temperature 1 × 100 (`int16_t`) |
| 2–3 | Temperature 2 × 100 (`int16_t`) |
| 4 | Hall digital output |
| 5 | RTD 1 fault |
| 6 | RTD 2 fault |
| 7 | Status flags |

Status byte:
- Bit 0: IMU OK
- Bit 1: Hall ADC OK

### 5.2 Slave 2

**ID `0x200` — ADC Measurement**

| Bytes | Field | Type |
|---|---|---|
| 0–1 | Raw ADC value | `int16_t` |
| 2–3 | ADC voltage (mV) | `uint16_t` |
| 4–5 | Input voltage (mV) | `uint16_t` |
| 6–7 | Packet counter | `uint16_t` |

The Master currently interprets these fields as big-endian.

### 5.3 Slave 3

**ID `0x300` — IR Temperature Sensors**

| Bytes | Field | Type |
|---|---|---|
| 0–1 | Sensor 1 ambient × 100 | `int16_t` |
| 2–3 | Sensor 1 object × 100 | `int16_t` |
| 4–5 | Sensor 2 ambient × 100 | `int16_t` |
| 6–7 | Sensor 2 object × 100 | `int16_t` |

**ID `0x301` — LiDAR and Pressure**

| Bytes | Field | Type |
|---|---|---|
| 0–1 | LiDAR distance (mm) | `uint16_t` |
| 2–5 | Pressure (Pa) | `uint32_t` |
| 6–7 | Sequence number | `uint16_t` |

**ID `0x302` — Status**

| Byte | Field |
|---|---|
| 0 | Status flags |
| 1–7 | Reserved |

Status flags:
- Bit 0: IR sensor 1 OK
- Bit 1: IR sensor 2 OK
- Bit 2: LiDAR OK
- Bit 3: Pressure sensor OK

Invalid-value markers defined by the Slave 3 protocol:
- Temperature: `0x8000`
- LiDAR: `0xFFFF`
- Pressure: `0xFFFFFFFF`

### 5.4 Slave 4

**ID `0x400` — PT100 Temperatures**

| Bytes | Field | Type |
|---|---|---|
| 0–1 | PT100 1 temperature × 100 | `int16_t` |
| 2–3 | PT100 2 temperature × 100 | `int16_t` |
| 4 | Status | `uint8_t` |
| 5–6 | Sequence number | `uint16_t` |
| 7 | Reserved | — |

**ID `0x401` — Hall Sensor**

| Bytes | Field | Type |
|---|---|---|
| 0–1 | Hall raw value | `uint16_t` |
| 2–3 | Hall voltage (mV) | `uint16_t` |
| 4 | Status | `uint8_t` |
| 5–6 | Sequence number | `uint16_t` |
| 7 | Reserved | — |

**ID `0x402` — LiDAR**

| Bytes | Field | Type |
|---|---|---|
| 0–1 | LiDAR distance (mm) | `int16_t` |
| 2–3 | Sequence number | `uint16_t` |
| 4 | Status | `uint8_t` |
| 5–7 | Reserved | — |

Negative LiDAR values are displayed as unavailable.

---

## 6. Software Implementation

### 6.1 CAN Initialization

The Master initializes an ESP-IDF on-chip TWAI node with:

- TX GPIO: 5
- RX GPIO: 4
- Bitrate: 500000 bit/s
- TX queue depth: 8
- RX FreeRTOS queue length: 32 messages

The receive callback is registered before the CAN node is enabled.

### 6.2 Receive Callback

The callback runs in the TWAI receive callback context.

It:
1. Receives the CAN frame.
2. Copies the identifier and eight data bytes.
3. Places the copied message in a FreeRTOS queue.
4. Returns without performing sensor decoding or serial logging.

This minimizes processing inside the callback.

### 6.3 CAN Processing Task

The `can_processing_task` runs at priority 10.

It blocks until a message becomes available in the receive queue, then:

1. Identifies the slave based on the incoming CAN ID.
2. Updates the corresponding last-received timestamp.
3. Calls the appropriate slave decoder.
4. Prints the decoded sensor values.
5. Determines the corresponding Master telemetry ID.
6. Republishes the frame if the identifier is recognized.

### 6.4 Master Status Task

The `master_status_task` runs at priority 5 and prints communication status once per second.

A slave is considered:
- **ONLINE:** a frame was received within the last 2000 ms.
- **OFFLINE:** no frame was received within the last 2000 ms.

The task also prints the Master telemetry transmission counters:

- `OK` — successful telemetry transmissions
- `FAIL` — failed telemetry transmissions

### 6.5 Telemetry CAN Transmission

The Master creates a standard CAN frame using the mapped Master ID and the original eight-byte payload.

It:
1. Enqueues the frame for transmission.
2. Waits for the transmission to complete.
3. Increments the success counter if transmission is confirmed.
4. Increments the failure counter and logs an error if transmission fails.

---

## 7. Project Structure

```text
ESP Multi Sensor/
└── master/
    ├── CMakeLists.txt
    ├── sdkconfig
    ├── README.md
    └── main/
        ├── CMakeLists.txt
        └── main.c
```

### Component CMake Configuration

`main/CMakeLists.txt`

```cmake
idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_driver_twai esp_driver_gpio
)
```

---

## 8. Build Instructions

Open an ESP-IDF PowerShell terminal.

```powershell
cd "C:\Users\Prana\Documents\LOOP\ESP Multi Sensor\master"
idf.py set-target esp32
idf.py build
```

### Flash and Monitor

When hardware is available:

```powershell
idf.py -p COMx flash monitor
```

Replace `COMx` with the actual ESP32 serial port.

Exit the monitor using:

```text
Ctrl + ]
```

---

## 9. Expected Serial Output

The following is an illustrative example of the output format, not data collected from connected hardware.

```text
I (xxx) MASTER: CAN initialized
I (xxx) MASTER: CAN bitrate : 500000 bit/s
I (xxx) MASTER: CAN TX      : GPIO 5
I (xxx) MASTER: CAN RX      : GPIO 4
I (xxx) MASTER: Master initialized.
I (xxx) MASTER: Waiting for Slave 1, Slave 2, Slave 3 and Slave 4...

I (xxx) MASTER: S1 | IMU | AX=12 AY=-3 AZ=2046 SEQ=118
I (xxx) MASTER: S1 | GYRO/HALL | GX=1 GY=2 GZ=3 HALL=1240
I (xxx) MASTER: S1 | TEMP | T1=24.51 C T2=24.60 C HALL_D0=1 RTD1F=0 RTD2F=0 STATUS=0x03

I (xxx) MASTER: S2 | ADC | RAW=16384 ADC=1.024 V INPUT=1.536 V SEQ=42

I (xxx) MASTER: S3 | TEMP | T1A=25.00 C T1O=26.50 C T2A=24.80 C T2O=27.10 C
I (xxx) MASTER: S3 | DIST/PRESS | LIDAR=450 mm PRESSURE=12000 Pa SEQ=20

I (xxx) MASTER: STATUS | S1=ONLINE S2=ONLINE S3=ONLINE S4=OFFLINE
I (xxx) MASTER: MASTER TELEMETRY TX | OK=100 FAIL=0
```

---

## 10. Troubleshooting

| Symptom | Possible cause |
|---|---|
| No CAN frames received | Transceiver wiring, TX/RX swapped, no common ground, bitrate mismatch, termination issue |
| Slaves report CAN transmission errors | Master is disconnected or unpowered, bus wiring fault, missing acknowledgement |
| Intermittent CAN frames | Incorrect termination, long stubs, wiring noise |
| `UNKNOWN CAN ID` warning | Frame ID is not handled by the current Master firmware |
| Slave appears OFFLINE | No recognized frame from that slave within 2 seconds |
| Slave 2 measurements appear incorrect | Byte-order mismatch between Slave 2 and Master |
| Master telemetry TX failures | CAN bus fault, transmission timeout, bus load |
| High serial output volume | Sensor transmission rate produces excessive logging |

---

## 11. Known Limitations

1. **Slave 2 byte order:** The Master decodes CAN ID `0x200` as big-endian. Slave 2 must use the same encoding.
2. **Slave 4 communication:** The Master supports IDs `0x400–0x402`, but the Slave 4 firmware must transmit those frames for it to appear ONLINE.
3. **Startup status:** Last-received timestamps start at zero. The status task may briefly report ONLINE before any frame is received.
4. **Invalid sensor values:** Some invalid-value markers are currently displayed numerically rather than as `N/A`.
5. **Sequence numbers:** Sequence fields are decoded but are not used for packet-loss detection.
6. **Bus-off recovery:** Automatic TWAI bus-off recovery or node restart is not implemented.
7. **Acceptance filtering:** The Master does not currently configure a CAN acceptance filter. Unknown IDs are handled in software.
8. **Serial logging load:** High-rate frames can produce substantial logging and may contribute to receive-queue overflow.
9. **Task creation checks:** FreeRTOS task-creation return values are not currently checked.
10. **Hardware control:** No commands are transmitted from the Master to operate slave hardware.
11. **Hardware validation:** Build success does not verify physical CAN reception, sensor accuracy, or end-to-end telemetry operation.

---

## 12. Verification Status

| Function | Status |
|---|---|
| ESP-IDF compilation | Build verified |
| TWAI initialization | Implemented |
| CAN reception | Implemented |
| Slave 1–4 frame decoding | Implemented |
| Serial sensor logging | Implemented |
| ONLINE/OFFLINE monitoring | Implemented |
| Master telemetry ID mapping | Implemented |
| Telemetry CAN retransmission | Implemented |
| Telemetry ESP firmware interface | Implemented |
| Physical CAN communication | Not yet hardware-verified |
| LoRa end-to-end transmission | Not yet hardware-verified |
| Master-to-slave hardware control | Not implemented |

---

**ESP Multi Sensor — Master**  
ESP32-WROOM-32 · ESP-IDF v6.x · FreeRTOS · TWAI/CAN
