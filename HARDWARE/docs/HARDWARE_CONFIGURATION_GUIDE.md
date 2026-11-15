# Agrisarthi Rover: Complete Hardware Configuration & Deployment Guide

**Document Version:** 1.0.0 (Hardware V1)  
**Date:** 2026-11-15  
**Target Hardware:** Raspberry Pi 5 (Ubuntu 24.04 / ROS 2 Jazzy) + ESP32-C6 Microcontroller + Dual BTS7960 H-Bridges + FEETECH FT5330M Probe Servo + 7-in-1 RS485 Modbus Soil Sensor + u-blox NEO-M8N GNSS + Base Station PC (Streamlit)

---

## Table of Contents

1. [Raspberry Pi Hardware Configuration](#1-raspberry-pi-hardware-configuration)
2. [ESP32-C6 Firmware Configuration](#2-esp32-c6-firmware-configuration)
3. [Physical Wiring Reference](#3-physical-wiring-reference)
4. [Software Rebuild Procedure](#4-software-rebuild-procedure)
5. [Hardware Startup Procedure](#5-hardware-startup-procedure)
6. [Base Station PC Setup](#6-base-station-pc-setup)
7. [Physical Commissioning Checklist](#7-physical-commissioning-checklist)
8. [Troubleshooting Guide](#8-troubleshooting-guide)
9. [Master Summary of Outstanding Configuration Items](#9-master-summary-of-outstanding-configuration-items)

---

## 1. Raspberry Pi Hardware Configuration

All Raspberry Pi ROS 2 parameters are centralized in:  
[`agrisarthi_ros_rover/agrisarthi_gps_nav/config/hardware.yaml`](../agrisarthi_ros_rover/agrisarthi_gps_nav/config/hardware.yaml)

```
agrisarthi_ros_rover/agrisarthi_gps_nav/config/hardware.yaml
├── gps_waypoint_navigator (Lines 5–34)
├── mission_sampling_manager (Lines 36–70)
├── nmea_gps_driver (Lines 72–81)
└── heading_node (Lines 83–96)
```

### Parameter Reference Table

| Section / Node | Line | Parameter Name | Current Value | Required Value / Determination Method | Example Configuration | What It Controls | Consequence of Incorrect Value |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **`nmea_gps_driver`** | 74 | `port` | `"/dev/ttyAMA0"` | Set to the physical UART port connected to the u-blox NEO-M8N (e.g. `/dev/ttyAMA0` for Pi 5 GPIO UART, or `/dev/serial/by-id/...` if USB). | `port: /dev/ttyAMA0` | Operating system serial device node for GNSS NMEA receiver. | Driver fails to open serial port; `/gps/fix` is never published; rover halts in `WAITING_FOR_SENSORS`. |
| **`nmea_gps_driver`** | 75 | `baudrate` | `9600` | Baud rate of the u-blox NEO-M8N. Standard factory default is 9600, or 38400/115200 if reconfigured via u-center. | `baudrate: 9600` | Serial UART communication baud rate. | Driver receives garbage framing errors; NMEA checksums fail; no fix published. |
| **`nmea_gps_driver`** | 77 | `gps_topic` | `"/gps/fix"` | `/gps/fix` (Must match topic expected by navigator and manager). | `gps_topic: /gps/fix` | ROS 2 output topic for `sensor_msgs/msg/NavSatFix`. | Navigator and sampling manager do not receive GPS fixes. |
| **`nmea_gps_driver`** | 78 | `frame_id` | `"gps_link"` | TF frame ID for GPS antenna. | `frame_id: gps_link` | Header frame ID in `NavSatFix` messages. | Transforms relative to base footprint fail in spatial tools. |
| **`nmea_gps_driver`** | 79 | `hdop_scale_m` | `2.5` | Standard GPS User Equivalent Range Error (UERE) factor. Default 2.5 meters. | `hdop_scale_m: 2.5` | Multiplier converting NMEA HDOP into 1-sigma position variance in meters (`Var = (HDOP * 2.5)²`). | Inaccurate covariance reported; potential false rejections if covariance gating is enabled. |
| **`heading_node`** | 88 | `enabled` | `false` | **Leave `false` until physical magnetometer/IMU is mounted and calibrated.** Set to `true` only when physical hardware is present. | `enabled: false` | Master safety switch for physical heading publisher. When `false`, node remains silent. | If set to `true` without hardware, driver continuously throws serial I/O warnings; if set to `true` with uncalibrated sensor, rover wanders or spins uncontrollably. |
| **`heading_node`** | 89 | `heading_topic`| `"/heading_deg"` | `/heading_deg` (Must match navigator's `heading_topic`). | `heading_topic: /heading_deg` | ROS 2 output topic for `std_msgs/msg/Float32` heading. | Navigator never receives heading data; halts in `WAITING_FOR_SENSORS`. |
| **`heading_node`** | 91 | `port` | `"/dev/ttyUSB1"` | Set to the serial device path of the compass module (e.g. `/dev/serial/by-id/usb-FTDI_...`). | `port: /dev/serial/by-id/usb-compass` | Serial port for physical heading device. | Node fails to open port. |
| **`heading_node`** | 95 | `magnetic_declination_deg` | `0.0` | Local magnetic declination for field location (positive East, negative West). Determine via NOAA or magnetic-declination.com. | `magnetic_declination_deg: -0.65` | Correction angle converting magnetic North to true geographic North. | Rover steers offset from target waypoint by the declination angle error. |
| **`heading_node`** | 96 | `calibration_offset_deg` | `0.0` | Mounting bias offset of the IMU sensor relative to rover chassis centerline. | `calibration_offset_deg: 0.0` | Zero-heading alignment with rover forward driving axis. | Rover drives at an angle to the target heading. |
| **`gps_waypoint_navigator`** | 10 | `gps_topic` | `"/gps/fix"` | `/gps/fix` | `gps_topic: /gps/fix` | Input topic subscribed for GNSS fixes. | Navigator ignores GNSS driver output. |
| **`gps_waypoint_navigator`** | 11 | `heading_topic` | `"/heading_deg"`| `/heading_deg` | `heading_topic: /heading_deg`| Input topic subscribed for rover heading. | Navigator ignores heading sensor output. |
| **`gps_waypoint_navigator`** | 15 | `esp32_transport` | `"udp"` | `"udp"` for real hardware; `"ros_sim"` for SITL. | `esp32_transport: udp` | Communication transport selected for ESP32 link. | If set to `ros_sim`, commands are sent over ROS topics and physical rover never moves. |
| **`gps_waypoint_navigator`** | 17 | `esp32_ip` | `"192.168.1.50"` | Static IP assigned to the ESP32-C6 on the local rover Wi-Fi network. | `esp32_ip: 192.168.1.50` | Destination IPv4 address for UDP motor/probe packets. | Pi cannot reach ESP32; rover halts in `COMMUNICATION_LOSS`. |
| **`gps_waypoint_navigator`** | 18 | `esp32_port` | `5005` | Must match `UDP_PORT` defined in ESP32 firmware (line 13). | `esp32_port: 5005` | Destination UDP port for motor and probe packets. | UDP packets dropped; handshake fails. |
| **`gps_waypoint_navigator`** | 20 | `communication_timeout_s` | `0.6` | Keep at 0.6 s (must be strictly shorter than ESP32 firmware 700 ms watchdog). | `communication_timeout_s: 0.6` | Duration of UDP silence before navigator flags `COMMUNICATION_LOSS`. | If too large, ESP32 watchdog triggers before Pi detects packet loss. |
| **`gps_waypoint_navigator`** | 21 | `gps_timeout_s` | `2.0` | Maximum age of GPS fix before safety pause. | `gps_timeout_s: 2.0` | Freshness threshold for GNSS data. | If too short, temporary GPS delay halts rover; if too long, rover travels blind. |
| **`gps_waypoint_navigator`** | 22 | `heading_timeout_s` | `1.0` | Maximum age of heading data before safety pause. | `heading_timeout_s: 1.0` | Freshness threshold for compass data. | If heading is missing > 1.0 s, controller halts. |
| **`gps_waypoint_navigator`** | 25 | `waypoint_tolerance_m` | `1.5` | Arrival threshold radius in meters. Recommended 1.0–2.0 m depending on GPS accuracy. | `waypoint_tolerance_m: 1.5` | Distance at which rover triggers arrival and initiates sampling. | If too small (< GNSS noise), rover circles waypoint without reaching it. |
| **`gps_waypoint_navigator`** | 27 | `max_linear_speed` | `0.18` | Maximum forward speed (normalized 0.0 to 1.0). Start low (0.15–0.25) during commissioning. | `max_linear_speed: 0.18` | Peak throttle output commanded to motors. | Excessive speed causes overshoot, wheel slip, or mechanical damage. |
| **`gps_waypoint_navigator`** | 28 | `max_angular_speed` | `0.35` | Maximum pivot steering speed (normalized 0.0 to 1.0). Start low (0.25–0.40). | `max_angular_speed: 0.35` | Peak rotational throttle commanded during pivot turns. | Excessive spin rate causes aggressive skid and high motor current. |
| **`gps_waypoint_navigator`** | 29 | `heading_kp` | `0.012` | Proportional gain for steering correction. | `heading_kp: 0.012` | Steering sensitivity per degree of heading error. | If too high, rover oscillates/snakes; if too low, rover fails to turn toward waypoint. |
| **`gps_waypoint_navigator`** | 30 | `turn_in_place_error_deg` | `55.0` | Heading error threshold beyond which rover halts forward drive and pivots in place. | `turn_in_place_error_deg: 55.0` | Pivot-turn threshold. | If too low, rover stops and pivots frequently; if too high, rover makes wide sweeping turns. |
| **`gps_waypoint_navigator`** | 34 | `return_home` | `true` | `true` if rover should return to launch coordinates after the final waypoint; `false` to stop at last waypoint. | `return_home: true` | Enables automatic return-to-home sequence. | Rover stops at final waypoint instead of navigating back to home point. |
| **`mission_sampling_manager`** | 56 | `home_latitude` | `-999.0` | **Replace placeholder `-999.0` with the surveyed latitude of the rover launch/home position.** | `home_latitude: 19.0760000` | Latitude coordinate of mission home/return base point. | **Placeholder `-999.0` deliberately blocks mission execution until configured.** |
| **`mission_sampling_manager`** | 57 | `home_longitude` | `-999.0` | **Replace placeholder `-999.0` with the surveyed longitude of the rover launch/home position.** | `home_longitude: 72.8777000` | Longitude coordinate of mission home/return base point. | **Placeholder `-999.0` deliberately blocks mission execution until configured.** |
| **`mission_sampling_manager`** | 58 | `data_directory` | `"~/agrisarthi_ws/missions/data"` | Path on Raspberry Pi where `mission_<id>_samples.json` documents are saved. | `data_directory: ~/agrisarthi_ws/missions/data` | Destination directory for atomic soil sample persistence. | If path is not writable, sampling aborts with an atomic persistence error. |
| **`mission_sampling_manager`** | 59 | `sensor_mode` | `"hardware"` | `"hardware"` for physical RS485 probe; `"simulation"` for synthetic SITL data. | `sensor_mode: hardware` | Operating mode for soil sensor interface. | If set to `simulation`, synthetic soil data is generated; physical serial port is ignored. |
| **`mission_sampling_manager`** | 60 | `sensor_port` | `"/dev/ttyUSB0"` | Path to the USB-RS485 adapter connected to the 7-in-1 probe. Recommended: use `/dev/serial/by-id/...`. | `sensor_port: /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0` | Serial port for RS485 soil sensor. | Soil acquisition fails; sampling manager aborts waypoint with sensor timeout. |
| **`mission_sampling_manager`** | 61 | `sensor_baudrate` | `4800` | 4800 baud (Standard factory default for 7-in-1 soil NPK probes). | `sensor_baudrate: 4800` | Serial baud rate for Modbus RTU communication. | Modbus frames garbled; CRC mismatches; sensor read failures. |
| **`mission_sampling_manager`** | 62 | `sensor_slave_id` | `1` | 1 (Standard factory default Modbus slave ID). | `sensor_slave_id: 1` | Modbus RTU target slave address. | Probe ignores Modbus requests; read times out. |
| **`mission_sampling_manager`** | 66 | `actuator_timeout_s` | `30.0` | 30.0 s (Safety timeout for probe insertion/retraction cycle confirmation). | `actuator_timeout_s: 30.0` | Maximum time allowed for ESP32 to confirm probe movement. | Probe command times out; manager latches in `RECOVERY_REQUIRED`. |

### Raspberry Pi UART & Permissions Configuration

On the Raspberry Pi 5, the primary hardware UART on GPIO 14 (TXD) and GPIO 15 (RXD) is `/dev/ttyAMA0`.

1. **Enable Hardware UART**:
   Edit `/boot/firmware/config.txt`:
   ```bash
   sudo nano /boot/firmware/config.txt
   ```
   Add or verify the following lines:
   ```ini
   enable_uart=1
   dtoverlay=uart0
   ```
2. **Disable Linux Serial Console**:
   The Linux OS must not use `/dev/ttyAMA0` for kernel serial console:
   ```bash
   sudo raspi-config
   ```
   Navigate to: `Interface Options` -> `Serial Port` ->
   - *"Would you like a login shell to be accessible over serial?"* -> **No**
   - *"Would you like the serial port hardware to be enabled?"* -> **Yes**
   Alternatively, verify `/boot/firmware/cmdline.txt` does **not** contain `console=serial0,115200` or `console=ttyAMA0,115200`.
3. **Set User Permissions**:
   The ROS user must belong to the `dialout` group to access serial devices:
   ```bash
   sudo usermod -aG dialout $USER
   ```
   Log out and log back in for changes to take effect.
4. **Identify Stable Device Paths**:
   Never use volatile names like `/dev/ttyUSB0` in production if multiple USB devices are attached. Find stable paths with:
   ```bash
   ls -l /dev/serial/by-id/
   ```

---

## 2. ESP32-C6 Firmware Configuration

Firmware file:  
[`agrisarthi_ros_rover/esp32c6_firmware/esp32c6_udp_motor_receiver.ino`](../agrisarthi_ros_rover/esp32c6_firmware/esp32c6_udp_motor_receiver.ino)

### Firmware Parameters & Hardware Mapping Table

| Line | Constant / Setting | Current Code Value | Required Physical Value / Determination | Physical Connection & Role |
| :--- | :--- | :--- | :--- | :--- |
| 11 | `WIFI_SSID` | `"REPLACE_WITH_WIFI_SSID"` | Set to the exact SSID of the rover's local Wi-Fi router or Raspberry Pi Wi-Fi hotspot. | Wi-Fi network name for UDP link. |
| 12 | `WIFI_PASSWORD` | `"REPLACE_WITH_WIFI_PASSWORD"` | Set to the Wi-Fi network password. | Wi-Fi network password. |
| 13 | `UDP_PORT` | `5005` | `5005` (Matches `hardware.yaml` line 18). | Local UDP port on ESP32 listening for Pi motor/probe commands. |
| 14 | `COMMAND_TIMEOUT_MS` | `700` | `700` | Safety motor watchdog. Stops motors if no valid command received within 700 ms. |
| 18 | `USE_STATIC_IP` | `true` | `true` (Matches `hardware.yaml` line 17 `192.168.1.50`). | Enables fixed IP allocation to avoid DHCP address drift. |
| 19 | `LOCAL_IP` | `IPAddress(192, 168, 1, 50)` | Match `hardware.yaml` `esp32_ip`. | Static IP address for ESP32-C6. |
| 20 | `GATEWAY` | `IPAddress(192, 168, 1, 1)` | Match rover network router/hotspot gateway. | Subnet default gateway. |
| 21 | `SUBNET` | `IPAddress(255, 255, 255, 0)` | Match subnet mask. | Subnet mask. |
| 30 | `MOTOR_PWM_FREQUENCY` | `15000` | 15000 Hz (15 kHz) | Ultrasonic PWM frequency for BTS7960 to eliminate audible motor whine. |
| 31 | `MOTOR_PWM_BITS` | `8` | 8 bits (0–255 duty cycle) | PWM resolution on ESP32 LEDC timer. |
| 35 | `LEFT_RPWM_PIN` | `4` | **Verify with physical PCB wiring.** | Connected to BTS7960 Left `RPWM` (Forward PWM input). Drives left-side DC motors forward. |
| 36 | `LEFT_LPWM_PIN` | `5` | **Verify with physical PCB wiring.** | Connected to BTS7960 Left `LPWM` (Reverse PWM input). Drives left-side DC motors in reverse. |
| 37 | `LEFT_EN_PIN` | `-1` | Set to GPIO pin if BTS7960 `R_EN` and `L_EN` are driven by ESP32; set to `-1` if `R_EN` and `L_EN` are tied to 3.3V/5V. | Enable pin for Left BTS7960. |
| 40 | `RIGHT_RPWM_PIN` | `6` | **Verify with physical PCB wiring.** | Connected to BTS7960 Right `RPWM` (Forward PWM input). Drives right-side DC motors forward. |
| 41 | `RIGHT_LPWM_PIN` | `7` | **Verify with physical PCB wiring.** | Connected to BTS7960 Right `LPWM` (Reverse PWM input). Drives right-side DC motors in reverse. |
| 42 | `RIGHT_EN_PIN` | `-1` | Set to GPIO pin if BTS7960 `R_EN` and `L_EN` are driven by ESP32; set to `-1` if `R_EN` and `L_EN` are tied to 3.3V/5V. | Enable pin for Right BTS7960. |
| 45 | `INVERT_LEFT_MOTOR` | `false` | Set to `true` if left wheels spin backward when commanding forward motion. | Software polarity correction for left motor channel. |
| 46 | `INVERT_RIGHT_MOTOR`| `false` | Set to `true` if right wheels spin backward when commanding forward motion. | Software polarity correction for right motor channel. |
| 53 | `PROBE_SERVO_PIN` | `3` | **Verify with physical PCB wiring.** | ESP32-C6 GPIO connected to FEETECH FT5330M servo white/yellow PWM signal wire. |
| 54 | `PROBE_INSERT_ANGLE`| `120` | Calibrate against mechanical rack-and-pinion. Must position soil probe fully into soil. | Commanded servo angle (degrees) for insertion depth. |
| 55 | `PROBE_RETRACT_ANGLE`| `30` | Calibrate against mechanical rack-and-pinion. Must lift soil probe fully clear of terrain. | Commanded servo angle (degrees) for retracted transit. |
| 56–57 | `PROBE_MIN/MAX_ANGLE`| `10` / `170` | Mechanical limit boundaries (degrees) to prevent servo over-travel. | Software travel clamps protecting rack-and-pinion linkage. |
| 58 | `PROBE_MIN_PULSE_US`| `500` | 500 µs (Standard FEETECH FT5330M specification for 0°). | Minimum PWM pulse width. |
| 59 | `PROBE_MAX_PULSE_US`| `2500` | 2500 µs (Standard FEETECH FT5330M specification for 180°). | Maximum PWM pulse width. |
| 60 | `PROBE_PWM_FREQUENCY`| `50` | 50 Hz (Standard RC servo frame frequency). | Frame rate for servo control. |
| 61 | `PROBE_PWM_BITS` | `14` | 14 bits (Provides fine pulse width resolution on 50 Hz). | LEDC timer resolution for servo channel. |
| 62 | `PROBE_TRAVEL_MS` | `1800` | Measured duration (ms) for servo to travel between retracted and inserted angles. | Estimated travel time. Software marks `position_assumed: true`. |
| 63 | `PROBE_SETTLE_MS` | `300` | Settling delay (ms) after transit before probe is reported stable. | Dwell time ensuring probe vibrations stop. |
| 64 | `PROBE_TIMEOUT_MS` | `5000` | 5000 ms. If limit switches are used and not triggered within 5 s, enters `ERROR`. | Failsafe timeout for probe actuator. |
| 67 | `USE_PROBE_LIMIT_SWITCHES` | `false` | Set to `true` **only if physical active-low limit switches are installed.** | Hardware endpoint confirmation switch. |
| 68 | `PROBE_INSERT_LIMIT_PIN` | `0` | GPIO connected to lower limit switch (Active LOW). | Lower endpoint limit switch. |
| 69 | `PROBE_RETRACT_LIMIT_PIN` | `1` | GPIO connected to upper limit switch (Active LOW). | Upper endpoint limit switch. |

---

## 3. Physical Wiring Reference

### Master Interconnection Table

| From Component & Pin | To Component & Pin | Signal / Function | Voltage / Level | Verification Status | Notes |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Pi 5 GPIO Pin 8 (TXD0)** | **u-blox NEO-M8N RXD** | UART Serial TX | 3.3V Logic | **Confirmed** | Do NOT use 5V signaling. |
| **Pi 5 GPIO Pin 10 (RXD0)** | **u-blox NEO-M8N TXD** | UART Serial RX | 3.3V Logic | **Confirmed** | Connects to NEO-M8N transmit pin. |
| **Pi 5 GPIO Pin 1 (3.3V)** | **u-blox NEO-M8N VCC** | GNSS Module Power | 3.3V DC (max 100mA) | **Confirmed** | Many NEO-M8N boards accept 3.3V or 5V; verify board voltage regulator. |
| **Pi 5 GPIO Pin 6 (GND)** | **u-blox NEO-M8N GND** | Digital Ground | 0V | **Confirmed** | Signal ground reference. |
| **Pi 5 USB Type-A Port** | **USB-to-RS485 Dongle** | USB D+/D-/5V/GND | 5V USB | **Confirmed** | Dongle provides FTDI or CH340 UART-RS485 transceiver. |
| **RS485 Dongle Terminal A+**| **7-in-1 Soil Sensor Pin A (Yellow)** | RS485 Differential (+) | RS485 Differential | **Confirmed** | Non-inverting RS485 line. |
| **RS485 Dongle Terminal B-**| **7-in-1 Soil Sensor Pin B (Blue)** | RS485 Differential (-) | RS485 Differential | **Confirmed** | Inverting RS485 line. |
| **12V Regulator Output (+)**| **7-in-1 Soil Sensor VCC (Brown)** | Sensor Power Supply | **12V DC** (9–24V) | **Confirmed** | **DO NOT connect to Pi 5V or 3.3V. Probe requires 9–24V.** |
| **12V Regulator Output (GND)**| **7-in-1 Soil Sensor GND (Black)** | Sensor Power Ground | 0V Common | **Confirmed** | Common ground with battery and Pi. |
| **ESP32-C6 GPIO 4** | **BTS7960 Left RPWM** | Left Forward PWM | 3.3V PWM (15 kHz) | *Requires PCB Verification* | Pin configurable via `LEFT_RPWM_PIN` in firmware. |
| **ESP32-C6 GPIO 5** | **BTS7960 Left LPWM** | Left Reverse PWM | 3.3V PWM (15 kHz) | *Requires PCB Verification* | Pin configurable via `LEFT_LPWM_PIN` in firmware. |
| **ESP32-C6 GPIO 6** | **BTS7960 Right RPWM** | Right Forward PWM | 3.3V PWM (15 kHz) | *Requires PCB Verification* | Pin configurable via `RIGHT_RPWM_PIN` in firmware. |
| **ESP32-C6 GPIO 7** | **BTS7960 Right LPWM** | Right Reverse PWM | 3.3V PWM (15 kHz) | *Requires PCB Verification* | Pin configurable via `RIGHT_LPWM_PIN` in firmware. |
| **BTS7960 Left R_EN / L_EN** | **ESP32-C6 3.3V or GPIO** | Left Driver Enable | 3.3V / 5V Logic | *Requires PCB Verification* | Tie both to 3.3V/5V for always-enabled operation. |
| **BTS7960 Right R_EN / L_EN**| **ESP32-C6 3.3V or GPIO** | Right Driver Enable | 3.3V / 5V Logic | *Requires PCB Verification* | Tie both to 3.3V/5V for always-enabled operation. |
| **12V Battery (+)** | **BTS7960 Left & Right B+** | Motor Power Bus | 12V DC (up to 20A) | **Confirmed** | Wire through an independent physical Emergency Stop Switch. |
| **12V Battery (-)** | **BTS7960 Left & Right B-** | Motor Power Ground | 0V Common | **Confirmed** | High-current ground return. |
| **BTS7960 Left M+ / M-** | **Left Motors (2 in Parallel)**| Motor Outputs | 0–12V DC | *Requires Polarity Check* | If left side rotates backward, swap M+ and M- or set `INVERT_LEFT_MOTOR = true`. |
| **BTS7960 Right M+ / M-** | **Right Motors (2 in Parallel)**| Motor Outputs | 0–12V DC | *Requires Polarity Check* | If right side rotates backward, swap M+ and M- or set `INVERT_RIGHT_MOTOR = true`. |
| **ESP32-C6 GPIO 3** | **FEETECH FT5330M Signal (White)**| Servo PWM Signal | 3.3V PWM (50 Hz) | *Requires PCB Verification* | Pin configurable via `PROBE_SERVO_PIN` in firmware. |
| **Dedicated 7.4V Buck (+)** | **FEETECH FT5330M VCC (Red)** | Servo Power Supply | **6.0V–7.4V DC** | **Confirmed** | **DO NOT power servo from ESP32 or Pi. Stall current is 3.5A.** |
| **Dedicated 7.4V Buck (GND)**| **FEETECH FT5330M GND (Black)** | Servo Power Ground | 0V Common | **Confirmed** | Tie to common rover ground bus. |
| **Main Battery (12V)** | **Pi 5 5V 5A Buck Regulator**| Pi 5 Power Supply | 5V DC @ 5A (USB-C) | **Confirmed** | Pi 5 requires stable 5V 5A power for reliable operation. |

### Power Architecture & Grounding Rules

```
                      +-------------------+
                      | 12V Main Battery  |
                      +---------+---------+
                                |
             +------------------+-------------------+
             |                  |                   |
      +------+------+    +------+------+     +------+------+
      | 5V 5A Buck  |    | 7.4V 5A Buck|     | 12V Direct  |
      +------+------+    +------+------+     +------+------+
             |                  |                   |
        Pi 5 & ESP32      FT5330M Servo      BTS7960 Motors &
                                             RS485 Soil Probe
             |                  |                   |
             +------------------+-------------------+
                                |
                      COMMON SYSTEM GROUND (0V)
```

> [!CAUTION]
> **Strict Common Ground Requirement**:
> The negative terminals of the 12V main battery, the 5V regulator, the 7.4V servo regulator, the ESP32-C6 ground, and the Raspberry Pi ground **MUST ALL BE CONNECTED TO A SINGLE COMMON GROUND BUS**. Failure to connect common ground will prevent PWM signals from referencing 0V, causing erratic servo twitches or unresponsive motor drivers.

---

## 4. Software Rebuild Procedure

### Exact Commands on Raspberry Pi 5

Open a terminal on the Raspberry Pi 5:

```bash
# 1. Navigate to workspace
cd /home/parth/projects/Agrisarthi

# 2. Source ROS 2 Jazzy underlay
source /opt/ros/jazzy/setup.bash

# 3. Install any missing system dependencies
rosdep install --from-paths agrisarthi_ros_rover --ignore-src -r -y

# 4. Build the ROS 2 package with symlink install
colcon build --base-paths agrisarthi_ros_rover --packages-select agrisarthi_gps_nav --symlink-install

# 5. Source the built workspace overlay
source install/setup.bash

# 6. Verify the package is recognized by ROS 2
ros2 pkg prefix agrisarthi_gps_nav
# Expected output: /home/parth/projects/Agrisarthi/install/agrisarthi_gps_nav

# 7. Run full automated unit test suite
PYTHONPATH=agrisarthi_ros_rover/agrisarthi_gps_nav:$PYTHONPATH python3 -m unittest discover -s agrisarthi_ros_rover/agrisarthi_gps_nav/test -v
# Expected output: Ran 55 tests in ~0.08s -> OK
```

### When is Rebuilding Required?

| File Modified | Rebuild Required? | Action Required |
| :--- | :--- | :--- |
| **Python files (`.py`)** inside `agrisarthi_gps_nav/` | **NO** (due to `--symlink-install`) | Changes take effect immediately upon next node restart. |
| **Launch files (`.launch.py`)** | **NO** (due to `--symlink-install`) | Changes take effect immediately upon next `ros2 launch`. |
| **`setup.py` entry points or data files** | **YES** | Run `colcon build --base-paths agrisarthi_ros_rover --packages-select agrisarthi_gps_nav --symlink-install`. |
| **`package.xml` dependencies** | **YES** | Run `rosdep install` and `colcon build`. |
| **YAML configs (`hardware.yaml`, `simulation.yaml`)**| **NO** (due to symlinks) | Verify edited file; restart the node. |
| **ESP32 Firmware (`.ino`, `.h`)** | **NO (ROS does not build ESP32 code)** | Flash ESP32-C6 using Arduino IDE or `esptool.py` over USB serial. |

---

## 5. Hardware Startup Procedure

### Step-by-Step Power-On & Launch Sequence

#### Phase 1: Physical Power-On
1. Power on the 12V Main Battery.
2. Confirm the 5V buck powers on the Raspberry Pi 5.
3. Confirm the ESP32-C6 powers up.
4. Confirm the 12V supply reaches the BTS7960 drivers and RS485 soil sensor.
5. Confirm the 7.4V regulator powers the FEETECH FT5330M servo.

#### Phase 2: Network & ESP32 Verification
1. Verify the rover local Wi-Fi router or Pi access point is broadcasting.
2. Verify ESP32-C6 connects to Wi-Fi and responds to ping at `192.168.1.50`:
   ```bash
   ping -c 3 192.168.1.50
   ```
   *Expected output: 3 packets transmitted, 3 received, 0% packet loss.*

#### Phase 3: Launch Hardware Stack on Raspberry Pi 5
In Terminal 1 on the Raspberry Pi 5:
```bash
cd /home/parth/projects/Agrisarthi
./scripts/start_hardware.sh
```
This automatically launches a dual-window background `tmux` session named `agrisarthi_hardware`:
- Window 0 (`hardware`): `ros2 launch agrisarthi_gps_nav hardware.launch.py`
- Window 1 (`rosbridge`): `rosbridge_websocket` on port `9090`

Attach to the session to inspect live logs:
```bash
tmux attach -t agrisarthi_hardware
```
*(Use `Ctrl+B` then `0` for hardware logs; `Ctrl+B` then `1` for rosbridge; `Ctrl+B` then `D` to detach without stopping processes).*

#### Phase 4: ROS Topic Health Checks
In a second terminal on the Raspberry Pi 5:
```bash
source /opt/ros/jazzy/setup.bash
source /home/parth/projects/Agrisarthi/install/setup.bash

# 1. Verify all expected nodes are running:
ros2 node list
# Expected nodes:
# /nmea_gps_driver
# /heading_node
# /gps_waypoint_navigator
# /mission_sampling_manager

# 2. Verify GPS fix publication:
ros2 topic echo /gps/fix --once
# Verify status.status >= 0 and latitude/longitude are valid non-zero WGS84 coordinates.

# 3. Verify Heading topic status:
ros2 topic echo /heading_deg --once
# If heading_node is disabled (default), this will time out waiting for messages (safe behavior).
# If heading_node is enabled, verify degrees [0.0..360.0) clockwise from North.

# 4. Verify Rover status:
ros2 topic echo /rover/nav_status --once
# Verify "esp32_connected": true, and "probe_state": "RETRACTED".

# 5. Verify Sampling manager status:
ros2 topic echo /mission/sampling_status --once
# Verify state reports "IDLE" or "MISSION_READY".
```

---

## 6. Base Station PC Setup

The Base Station runs the Streamlit planner UI on a personal laptop or PC. It does **not** require ROS 2 to be installed.

### Commands to Run on the PC

```bash
# 1. Navigate to field planner directory
cd /path/to/Agrisarthi/rover_field_planner

# 2. Activate virtual environment
source .venv/bin/activate

# 3. Ensure required dependencies are installed
pip install -r requirements.txt

# 4. Set environment variables to point to Raspberry Pi 5
export AGRISARTHI_ROSBRIDGE_URL="ws://<PI_LAN_IP>:9090"
export AGRISARTHI_GPS_TOPIC="/gps/fix"
export AGRISARTHI_SITL_DEMO="0"

# 5. Launch Streamlit
streamlit run app.py
```
Open your browser to:  
`http://localhost:8501`

### Responsibility Matrix: PC vs Raspberry Pi 5

| Task / Subsystem | Runs On | Network Mechanism |
| :--- | :--- | :--- |
| **Streamlit Web Application** | **Base Station PC** | Local Python runtime (`app.py`) |
| **Interactive Map & Boundary Drawing** | **Base Station PC** | Folium / HTML inside Streamlit |
| **Serpentine Waypoint Generator** | **Base Station PC** | Shapely equirectangular grid math |
| **rosbridge WebSocket Server** | **Raspberry Pi 5** | Port 9090 (TCP) |
| **u-blox NEO-M8N GNSS Driver** | **Raspberry Pi 5** | Direct UART (`/dev/ttyAMA0`) |
| **RS485 Modbus Soil Reader** | **Raspberry Pi 5** | USB-to-RS485 transceiver (`/dev/ttyUSB0`) |
| **GPS Waypoint Navigator** | **Raspberry Pi 5** | ROS 2 Jazzy node (`gps_waypoint_navigator`) |
| **Sampling Manager & Interlocks** | **Raspberry Pi 5** | ROS 2 Jazzy node (`mission_sampling_manager`) |
| **Mission JSON Storage** | **Raspberry Pi 5** | Atomic disk persistence (`missions/data/`) |
| **Motor & Servo PWM Generation** | **ESP32-C6** | Hardware LEDC PWM timers |

---

## 7. Physical Commissioning Checklist

> [!IMPORTANT]
> **Safety Rule**: Keep drive wheels raised off the ground on stands during all initial actuator, steering, and communication checks. Have an independent physical emergency stop switch wired directly into the motor battery line.

| # | Item / Subsystem | Test Procedure | Pass / Success Criteria | Independent Physical E-Stop Required? | Status |
| :---: | :--- | :--- | :--- | :---: | :---: |
| 1 | **Pi 5 Serial Permissions** | Run `id -nG` on Pi 5. | Output contains `dialout`. | No | [ ] |
| 2 | **GPS UART Feed** | Connect NEO-M8N. Run `minicom -b 9600 -D /dev/ttyAMA0`. | ASCII NMEA sentences (`$GNGGA`, `$GNRMC`) streaming continuously. | No | [ ] |
| 3 | **GPS Fix Validity** | Start `nmea_gps_driver`. Run `ros2 topic echo /gps/fix --once`. | `status.status: 0`, finite coordinates matching physical location. | No | [ ] |
| 4 | **Heading Interface Safety** | With `enabled: false`, run `ros2 topic echo /heading_deg`. | No messages published. Log confirms sensor disabled. | No | [ ] |
| 5 | **ESP32 Ping & UDP** | Ping `192.168.1.50`. Start `hardware.launch.py`. | ESP32 replies to ping; `/rover/nav_status` shows `esp32_connected: true`. | No | [ ] |
| 6 | **Probe Bootstrap State** | Check `/mission/sampling_status` at startup. | Status reaches `IDLE` after ESP32 confirms probe `RETRACTED`. | No | [ ] |
| 7 | **Probe Movement (Raised)**| Publish `/probe/command` `{"action": "PROBE_INSERT", "command_id": "test1"}`. | Servo rotates to `PROBE_INSERT_ANGLE`; status transitions `INSERTING` -> `INSERTED`. | **YES (Stand clear)**| [ ] |
| 8 | **Probe Retraction** | Publish `/probe/command` `{"action": "PROBE_RETRACT", "command_id": "test2"}`. | Servo rotates to `PROBE_RETRACT_ANGLE`; status transitions `RETRACTING` -> `RETRACTED`. | **YES (Stand clear)**| [ ] |
| 9 | **Motor Interlock Check**| With probe `INSERTED`, publish `/cmd_vel` (linear=0.1). | **Motors remain completely stationary.** Status shows motors disabled. | **YES** | [ ] |
| 10 | **Watchdog Cutoff Check**| With wheels raised and probe `RETRACTED`, command forward motion. Disconnect Pi UDP. | **Motor PWM stops completely within 700 ms.** | **YES** | [ ] |
| 11 | **Motor Rotation Directions**| Command low forward speed (0.1). | Both left and right wheels rotate forward. (If reversed, set inversion flag in firmware). | **YES** | [ ] |
| 12 | **Steering Direction Check**| Command clockwise pivot (angular=-0.2). | Left wheels rotate forward, right wheels rotate reverse. | **YES** | [ ] |
| 13 | **Soil Sensor RS485** | Connect 7-in-1 probe to 12V. Run `python3 -c "from agrisarthi_gps_nav.soil_sensor import SoilSensor; print(SoilSensor().read())"`. | Returns `success=True` with valid moisture, temp, pH, EC, NPK readings. | No | [ ] |
| 14 | **Atomic JSON Persistence**| Trigger test waypoint completion. Inspect `missions/data/`. | File `mission_<id>_samples.json` created with valid schema and UTC timestamps. | No | [ ] |
| 15 | **Geofence Enforcement** | Submit mission boundary that excludes current rover GPS location. | Mission rejected with `GEOFENCE_BREACH` or `MISSION_REJECTED`; motors remain stopped. | No | [ ] |
| 16 | **Ground Field Trial** | Lower rover to field. Submit small 2-waypoint mission. | Rover navigates at low speed, halts at waypoint, samples soil, retracts probe, and stops safely. | **YES (E-Stop in hand)** | [ ] |

---

## 8. Troubleshooting Guide

### 1. GPS Device Not Detected / No GPS Fix
- **Symptom**: `nmea_gps_driver` logs `Could not open GNSS UART port '/dev/ttyAMA0'`.
- **Diagnostics**:
  ```bash
  ls -l /dev/ttyAMA0 /dev/serial0
  sudo dmesg | grep -i tty
  ```
- **Correction**:
  - Verify `/boot/firmware/config.txt` contains `enable_uart=1`.
  - Ensure Linux serial console is disabled via `sudo raspi-config`.
  - Ensure user is in `dialout`: `sudo usermod -aG dialout $USER`.
  - Check physical wiring: NEO-M8N TX must connect to Pi RX (Pin 10); NEO-M8N RX must connect to Pi TX (Pin 8).
  - If using USB GNSS dongle, update `hardware.yaml` line 74 to `/dev/serial/by-id/...`.

### 2. GPS Connected but No Valid Coordinate Fix
- **Symptom**: `/gps/fix` reports `status.status: -1` (NO_FIX), latitude=0.0, longitude=0.0.
- **Diagnostics**:
  ```bash
  cat /dev/ttyAMA0 | grep -E "GGA|RMC"
  ```
- **Correction**:
  - GNSS receiver requires clear line of sight to sky. Testing indoors will result in `NO_FIX` (quality 0).
  - Place antenna near window or outdoors. Allow 1–3 minutes for cold-start satellite acquisition.

### 3. Missing Heading / Rover Stuck in `WAITING_FOR_SENSORS`
- **Symptom**: Rover never starts navigating; status repeatedly reports `WAITING_FOR_SENSORS`.
- **Cause**: By design, `heading_node` defaults to `enabled: false` because physical compass is unavailable. The navigation core strictly halts when heading data is stale (> 1.0 s).
- **Correction**:
  - Connect physical magnetometer/IMU.
  - Set `enabled: true` in `hardware.yaml` line 88.
  - Set valid serial device path in line 91.

### 4. ESP32 UDP Connection Failure / `WAITING_FOR_ESP32`
- **Symptom**: Navigator logs `ESP32 handshake/ack missing; motors remain stopped`.
- **Diagnostics**:
  ```bash
  ping -c 3 192.168.1.50
  sudo tcpdump -i any udp port 5005
  ```
- **Correction**:
  - Ensure ESP32 and Pi 5 are on the same Wi-Fi subnet.
  - Verify `WIFI_SSID` and `WIFI_PASSWORD` in `esp32c6_udp_motor_receiver.ino` lines 11–12.
  - Verify static IP `192.168.1.50` matches `hardware.yaml` line 17.
  - Check firewall on Pi: `sudo ufw allow 5005/udp`.

### 5. Motor Direction Reversed (Rover Drives Backward or Pivots Incorrectly)
- **Symptom**: Commanding forward causes rover to drive reverse or spin.
- **Correction**:
  - Edit `esp32c6_udp_motor_receiver.ino`:
    - Line 45: Set `INVERT_LEFT_MOTOR = true;` if left wheels spin backward.
    - Line 46: Set `INVERT_RIGHT_MOTOR = true;` if right wheels spin backward.
  - Re-flash the ESP32-C6. Alternatively, swap the physical motor output wires (`M+` and `M-`) on the respective BTS7960 terminal block.

### 6. Probe Servo Not Moving
- **Symptom**: Manager commands `PROBE_INSERT`, but rack-and-pinion does not move.
- **Correction**:
  - Check servo power: FEETECH FT5330M requires 6.0V–7.4V. Verify regulator output with multimeter.
  - Verify common ground exists between the 7.4V regulator ground and the ESP32 ground.
  - Check GPIO pin in `esp32c6_udp_motor_receiver.ino` line 53 (`PROBE_SERVO_PIN = 3`).

### 7. Soil Sensor Timeout / RS485 Modbus CRC Errors
- **Symptom**: Manager reports `Modbus CRC16 mismatch` or `serial I/O failed: timed out`.
- **Correction**:
  - Check RS485 polarity: swap terminals `A` and `B` on the USB-RS485 dongle.
  - Verify probe power: 7-in-1 probe requires **12V DC** (9–24V). It will not operate on 5V.
  - Confirm baud rate is 4800 and slave ID is 1 in `hardware.yaml` lines 61–62.
  - Verify device path: ensure `/dev/ttyUSB0` has not shifted to `/dev/ttyUSB1`. Use `/dev/serial/by-id/...`.

### 8. rosbridge Connection Failure from PC
- **Symptom**: Streamlit planner reports `Could not reach ROS bridge: ConnectionRefusedError`.
- **Diagnostics**:
  ```bash
  # On Raspberry Pi:
  netstat -tuln | grep 9090
  ```
- **Correction**:
  - Ensure rosbridge is launched with `address:=0.0.0.0` (as in `scripts/start_hardware.sh`), not `127.0.0.1`.
  - Verify PC environment variable: `export AGRISARTHI_ROSBRIDGE_URL="ws://<ACTUAL_PI_IP>:9090"`.
  - Check Pi firewall: `sudo ufw allow 9090/tcp`.

---

## 9. Master Summary of Outstanding Configuration Items

The following physical values **MUST BE DETERMINED AND CONFIGURED** during physical rover assembly:

| Item | File | Location | Default / Current Value | Action Required When Physical Hardware is Attached |
| :---: | :--- | :--- | :--- | :--- |
| **1** | `hardware.yaml` | Lines 56–57 | `home_latitude: -999.0`<br>`home_longitude: -999.0` | **Survey physical launch location** and enter true WGS84 coordinates. |
| **2** | `hardware.yaml` | Line 60 | `sensor_port: /dev/ttyUSB0` | Find persistent adapter ID using `ls -l /dev/serial/by-id/` and update. |
| **3** | `hardware.yaml` | Line 74 | `port: /dev/ttyAMA0` | Confirm whether GNSS receiver connects via GPIO UART (`/dev/ttyAMA0`) or USB. |
| **4** | `hardware.yaml` | Lines 88–96 | `enabled: false`<br>`port: /dev/ttyUSB1` | Mount physical compass; set `enabled: true`; measure declination and mounting offset. |
| **5** | `.ino` Firmware | Lines 11–12 | `"REPLACE_WITH_WIFI_..."` | Enter confirmed Wi-Fi SSID and password for local rover network. |
| **6** | `.ino` Firmware | Lines 35–42 | `LEFT_RPWM_PIN = 4`, etc. | Verify GPIO pin mapping against physical PCB wiring to BTS7960 drivers. |
| **7** | `.ino` Firmware | Line 53 | `PROBE_SERVO_PIN = 3` | Verify GPIO pin connected to FEETECH FT5330M servo signal wire. |
| **8** | `.ino` Firmware | Lines 54–55 | `INSERT_ANGLE = 120`<br>`RETRACT_ANGLE = 30` | **Raise drive wheels; mechanically calibrate angles** so probe penetrates soil without jamming. |
| **9** | `.ino` Firmware | Lines 45–46 | `INVERT_... = false` | Run low-speed test on stands; set to `true` for any wheel that spins backward. |
| **10**| `.ino` Firmware | Line 67 | `USE_LIMIT_SWITCHES = false` | Install physical active-low limit switches before unattended field deployment. |
