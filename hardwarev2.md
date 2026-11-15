# Agrisarthi Rover: Complete Hardware Implementation Report

The physical hardware integration for the Agrisarthi autonomous soil-analysis rover is complete, verified, and ready for physical commissioning. The existing Software-In-The-Loop (SITL) implementation remains 100% intact and operational.

---

## 1. Files Created

| File | Purpose / Role |
| :--- | :--- |
| [`agrisarthi_ros_rover/agrisarthi_gps_nav/agrisarthi_gps_nav/nmea_gps_driver.py`](agrisarthi_ros_rover/agrisarthi_gps_nav/agrisarthi_gps_nav/nmea_gps_driver.py) | ROS 2 Jazzy node parsing UART NMEA-0183 streams (`$GNGGA`, `$GPGGA`, `$GNRMC`) from the u-blox NEO-M8N receiver. Validates checksums, formats decimal WGS84 coordinates, computes HDOP-scaled horizontal/vertical position covariance, and publishes `sensor_msgs/msg/NavSatFix` on `/gps/fix`. |
| [`agrisarthi_ros_rover/agrisarthi_gps_nav/agrisarthi_gps_nav/heading_node.py`](agrisarthi_ros_rover/agrisarthi_gps_nav/agrisarthi_gps_nav/heading_node.py) | Physical compass/IMU interface node. Configured as `enabled: false` by default (since physical sensor is unavailable). Adheres strictly to fail-closed safety: **does not publish constant or simulated data**, safely allowing `NavigationController` to hold the rover in `WAITING_FOR_SENSORS`. Extensible for physical serial/I2C magnetometers once attached. |
| [`agrisarthi_ros_rover/agrisarthi_gps_nav/test/test_nmea_gps_driver.py`](agrisarthi_ros_rover/agrisarthi_gps_nav/test/test_nmea_gps_driver.py) | Unit tests verifying NMEA XOR checksum verification, coordinate parsing (N/S/E/W), valid/void fix handling, HDOP covariance calculation, and fragmented/chunked serial stream accumulation. |
| [`agrisarthi_ros_rover/agrisarthi_gps_nav/test/test_heading_node.py`](agrisarthi_ros_rover/agrisarthi_gps_nav/test/test_heading_node.py) | Unit tests verifying heading angle normalization and wrapping [0..360°), declination/offset compensation, disabled-mode silence, and live parsing of heading strings (`$HCHDT`, `H:deg`, raw float). |
| [`scripts/start_hardware.sh`](scripts/start_hardware.sh) | Executable bash runner that starts the entire physical hardware ROS 2 stack and `rosbridge_server` (bound to `0.0.0.0:9090`) in a dual-window background `tmux` session on Raspberry Pi 5. |

---

## 2. Files Modified

| File | Changes Made |
| :--- | :--- |
| [`agrisarthi_ros_rover/agrisarthi_gps_nav/config/hardware.yaml`](agrisarthi_ros_rover/agrisarthi_gps_nav/config/hardware.yaml) | Added explicit `esp32_transport: udp`, `return_home: true`, and `sensor_mode: hardware`. Added complete parameter blocks for `nmea_gps_driver` (`port: /dev/ttyAMA0`, `baudrate: 9600`) and `heading_node` (`enabled: false`). Documented placeholder parameters. |
| [`agrisarthi_ros_rover/agrisarthi_gps_nav/launch/hardware.launch.py`](agrisarthi_ros_rover/agrisarthi_gps_nav/launch/hardware.launch.py) | Added `nmea_gps_driver` and `heading_node` to the production launch description alongside `gps_waypoint_navigator` and `mission_sampling_manager`. |
| [`agrisarthi_ros_rover/agrisarthi_gps_nav/setup.py`](agrisarthi_ros_rover/agrisarthi_gps_nav/setup.py) | Registered console script entry points for `nmea_gps_driver` and `heading_node`. |
| [`agrisarthi_ros_rover/esp32c6_firmware/esp32c6_udp_motor_receiver.ino`](agrisarthi_ros_rover/esp32c6_firmware/esp32c6_udp_motor_receiver.ino) | Replaced simple H-bridge code with dual BTS7960 module driver (`setBts7960Channel` driving RPWM, LPWM, and optional EN pins). Added static IP configuration (`192.168.1.50`). Configured FEETECH FT5330M servo timing (500–2500 µs pulse at 50 Hz). Maintained watchdog (700 ms) and probe interlock. |
| [`scripts/README.md`](scripts/README.md) | Documented physical hardware startup, base station PC connection instructions, and tmux operational commands. |

---

## 3. Final Software Architecture

```mermaid
flowchart TD
    subgraph BaseStation["Personal PC / Base Station"]
        Planner["Streamlit Field Planner (app.py)"]
        LocalFiles["Local Mission JSON Storage"]
    end

    subgraph Pi5["Raspberry Pi 5 (ROS 2 Jazzy)"]
        Bridge["rosbridge_server (WebSocket :9090)"]
        GPSNode["nmea_gps_driver (u-blox NEO-M8N)"]
        HeadingNode["heading_node (Compass/IMU - disabled)"]
        NavNode["gps_waypoint_navigator (NavigationController)"]
        MgrNode["mission_sampling_manager"]
        SoilMod["SoilSensor (RS485 Modbus RTU /dev/ttyUSB0)"]
        DataStore["Atomic Sample Records (mission_<id>_samples.json)"]
    end

    subgraph ESP32["ESP32-C6 Microcontroller (:5005)"]
        UDPComm["WiFiUDP Server (Session & Seq Auth)"]
        Watchdog["700ms Safety Motor Watchdog"]
        ProbeFSM["Probe State Machine (probe_fsm.h)"]
        BTSLeft["BTS7960 Left (2 Motors Parallel)"]
        BTSRight["BTS7960 Right (2 Motors Parallel)"]
        Servo["FEETECH FT5330M Servo (50Hz PWM)"]
    end

    %% Network links
    Planner <-->|WebSocket ws://:9090| Bridge
    Bridge <-->|/mission/waypoints_json\n/mission/start\n/mission/query| MgrNode
    Bridge <-->|/gps/fix (map centering)| Planner

    %% Internal ROS 2 topics
    GPSNode -->|/gps/fix| NavNode
    GPSNode -->|/gps/fix| MgrNode
    HeadingNode -.->|/heading_deg (when enabled)| NavNode
    MgrNode -->|/mission/navigation_start| NavNode
    NavNode -->|/rover/nav_status| MgrNode
    NavNode -->|/probe/status| MgrNode
    MgrNode -->|/probe/command| NavNode
    MgrNode -->|/mission/waypoint_complete| NavNode
    MgrNode --> SoilMod
    SoilMod --> DataStore

    %% Hardware links
    NavNode <-->|Acknowledged UDP JSON :5005| UDPComm
    UDPComm --> Watchdog
    Watchdog --> BTSLeft
    Watchdog --> BTSRight
    UDPComm --> ProbeFSM
    ProbeFSM --> Servo
    ProbeFSM -->|Hardware Interlock: Disable Motors| Watchdog
```

---

## 4. Completed Hardware Drivers

1. **u-blox NEO-M8N GNSS Driver (`nmea_gps_driver.py`)**:
   - Reads UART NMEA sentences (`$GNGGA`, `$GPGGA`, `$GNRMC`).
   - Validates NMEA XOR checksums on every frame.
   - Converts `DDMM.MMMM` strings into signed decimal WGS84 coordinates.
   - Calculates horizontal accuracy covariance: `Var = (HDOP * 2.5m)²`.
   - Publishes `sensor_msgs/msg/NavSatFix` to `/gps/fix`.

2. **Heading Sensor Interface (`heading_node.py`)**:
   - Manages compass/IMU integration.
   - Supports configurable port, baud rate, declination, and calibration offset.
   - Normalizes heading degrees strictly clockwise from North in `[0.0, 360.0)`.
   - **Safety Enforcement**: Defaults to `enabled: false`. Does not inject fake or constant numbers; ensures `gps_waypoint_navigator` latches in `WAITING_FOR_SENSORS` until a physical sensor is online.

3. **7-in-1 Soil NPK Modbus Driver (`soil_sensor.py`)**:
   - Preserved and verified. Modbus RTU function 0x03 over RS485 at 4800 baud, 8N1, slave 1.
   - Registers: 0x0000 (moisture/10), 0x0001 (temperature/10), 0x0002 (raw EC), 0x0003 (pH/10).
   - NPK: primary query at `0x001E`, fallback to `0x0004` if zero/missing. Rejects all-zero NPK readings as failed acquisition.

---

## 5. ESP32 Firmware Changes

In [`esp32c6_firmware/esp32c6_udp_motor_receiver.ino`](agrisarthi_ros_rover/esp32c6_firmware/esp32c6_udp_motor_receiver.ino):

1. **BTS7960 Dual H-Bridge Driver**:
   - Implemented `setBts7960Channel()` driving forward PWM (`RPWM`) and reverse PWM (`LPWM`) with optional enable (`EN`).
   - Independent channels for Left (two motors parallel) and Right (two motors parallel).
   - Differential / skid-steering logic supports: forward, reverse, stop, clockwise rotation (pivot right), counter-clockwise rotation (pivot left), and proportional steering.
   - Configurable inversion flags: `INVERT_LEFT_MOTOR`, `INVERT_RIGHT_MOTOR`.
2. **FEETECH FT5330M Servo Control**:
   - Configured 50 Hz PWM with 14-bit resolution on LEDC timer.
   - Pulse widths: `PROBE_MIN_PULSE_US = 500` (0°), `PROBE_MAX_PULSE_US = 2500` (180°).
   - Transit timing: 1800 ms travel + 300 ms settle (5000 ms timeout).
   - Reports `position_assumed: true` in status payloads when physical limit switches are absent.
3. **Static IP Configuration**:
   - Added `USE_STATIC_IP = true` with `LOCAL_IP(192, 168, 1, 50)`, matching `hardware.yaml`.
4. **Safety Watchdog & Interlock**:
   - 700 ms silence watchdog cuts motor output immediately.
   - Probe state machine interlock prohibits motor power unless probe state is `RETRACTED`.

---

## 6. Hardware Configuration Requirements

Prior to real field deployment, edit [`agrisarthi_ros_rover/agrisarthi_gps_nav/config/hardware.yaml`](agrisarthi_ros_rover/agrisarthi_gps_nav/config/hardware.yaml):

1. **Surveyed Home Point**: Replace placeholder `-999.0`:
   ```yaml
   home_latitude: <SURVEYED_LATITUDE>
   home_longitude: <SURVEYED_LONGITUDE>
   ```
2. **Soil Sensor Port**: Confirm stable serial path:
   ```yaml
   sensor_port: /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
   ```
3. **GNSS Serial Port**: Confirm UART port for u-blox NEO-M8N:
   ```yaml
   port: /dev/ttyAMA0  # or /dev/serial0
   baudrate: 9600
   ```
4. **Compass / Heading**: When physical magnetometer is mounted:
   ```yaml
   heading_node:
     ros__parameters:
       enabled: true
       port: /dev/serial/by-id/<COMPASS_DEVICE>
       magnetic_declination_deg: <LOCAL_DECLINATION>
   ```
5. **ESP32 Firmware**: In `esp32c6_udp_motor_receiver.ino`:
   - Set `WIFI_SSID` and `WIFI_PASSWORD`.
   - Confirm pin numbers for BTS7960 and FEETECH FT5330M servo.

---

## 7. Exact Raspberry Pi Build Commands

From the repository root on the Raspberry Pi 5:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths agrisarthi_ros_rover --packages-select agrisarthi_gps_nav --symlink-install
source install/setup.bash
```

---

## 8. Exact Raspberry Pi Hardware Launch Command

```bash
# Automated launch (launches hardware nodes + rosbridge in tmux):
./scripts/start_hardware.sh

# Or manual terminal launch:
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch agrisarthi_gps_nav hardware.launch.py
```

---

## 9. Exact PC Streamlit Launch Command

On the laptop / base station PC:

```bash
cd rover_field_planner
source .venv/bin/activate
export AGRISARTHI_ROSBRIDGE_URL="ws://<PI_LAN_IP>:9090"
export AGRISARTHI_GPS_TOPIC="/gps/fix"
export AGRISARTHI_SITL_DEMO="0"
streamlit run app.py
```

---

## 10. Tests Executed & Results

All 56 unit and integration tests passed cleanly:

| Test Suite | File | Tests | Result |
| :--- | :--- | :--- | :--- |
| **NMEA GNSS Driver** | `test_nmea_gps_driver.py` | 12 | **PASS** (12/12) |
| **Heading Node Interface** | `test_heading_node.py` | 7 | **PASS** (7/7) |
| **ESP32 Link & Protocol** | `test_esp32_link.py` | 3 | **PASS** (3/3) |
| **Navigation Safety Core** | `test_navigation_core.py` | 18 | **PASS** (18/18) |
| **Mission Data & Schema** | `test_mission_data.py` | 6 | **PASS** (6/6) |
| **Simulated Hardware (SITL)**| `test_simulated_hardware.py` | 3 | **PASS** (3/3) |
| **Modbus Soil Sensor** | `test_soil_sensor.py` | 6 | **PASS** (6/6) |
| **Planner Fixture Tests** | `test_sitl_mission.py` | 1 | **PASS** (1/1) |
| **Firmware Protocol C++** | `esp32_protocol_test.cpp` | 1 | **PASS** |
| **Probe State Machine C++** | `probe_fsm_test.cpp` | 1 | **PASS** |
| **Launch Description Check**| `hardware.launch.py` / `simulation.launch.py` | 2 | **PASS** (4 entities each) |

---

## 11. Tests Not Executed & Technical Reasons

1. **Physical UART Serial I/O on `/dev/ttyAMA0`**: The u-blox NEO-M8N hardware is currently physically disconnected. Verified via recorded NMEA test fixtures and mock byte streams.
2. **Physical RS485 Modbus Transmission**: Physical 7-in-1 sensor was not connected during this turn. Verified via simulated serial Modbus frames, CRC16 validation, and mock exception responses.
3. **Physical DC Motor Activation**: As mandated by safety conventions, motors were not energized during software validation. BTS7960 logic was validated via host-side unit tests and simulated link packets.

---

## 12. Remaining Physical Hardware Dependencies

Before field operation can begin, the following physical tasks must be completed:
1. **Compass / Magnetometer Module**: Physical IMU/magnetometer must be connected to the Pi and calibrated (hard/soft iron).
2. **UART GNSS Connection**: Connect u-blox NEO-M8N TX/RX lines to Pi 5 UART GPIO pins (`/dev/ttyAMA0` or `/dev/serial0`) with 3.3 V logic levels.
3. **12 V Sensor Power**: Ensure 7-in-1 probe is powered with 9–24 V DC (do not power from the Pi’s 5 V rail).
4. **Mechanical Calibration**: Calibrate rack-and-pinion travel angles against FEETECH FT5330M servo horn.

---

## 13. Step-by-Step Physical Commissioning Procedure

```text
[Bench with Wheels Raised]
  1. Flash ESP32-C6 firmware with confirmed Wi-Fi SSID/password.
  2. Verify ESP32 connects to local Wi-Fi at 192.168.1.50.
  3. Start hardware stack on Pi: ./scripts/start_hardware.sh.
  4. Verify ESP32 handshake is accepted (probe reports RETRACTED).
  5. Test probe insertion/retraction manually via /probe/command topic.
  6. Confirm probe interlock stops motor output during probe movement.
  7. Confirm 700 ms watchdog cuts motor PWM when Pi traffic ceases.

[Sensor Checks]
  8. Connect GNSS module; verify fresh NavSatFix messages on /gps/fix.
  9. Connect compass module, set enabled: true in hardware.yaml, verify /heading_deg (0°=N, 90°=E).
  10. Connect RS485 probe to 12V; test single-shot read using soil_sensor.py.

[Ground Field Test]
  11. Survey field boundary corners and home location.
  12. Set surveyed home coordinates in hardware.yaml.
  13. Connect Streamlit planner on PC; draw field boundary and send mission.
  14. Verify MISSION_READY and probe RETRACTED.
  15. Issue /mission/start true with independent physical E-stop held in hand.
```

---

## 14. Known Limitations & Unresolved Risks

1. **Compass Tilt & Magnetic Interference**: Near-field DC motor currents produce magnetic distortion. The magnetometer must be mounted on a mast elevated away from the motor driver power cables.
2. **Open-Loop Probe Insertion**: Without limit switches (`USE_PROBE_LIMIT_SWITCHES = false`), probe position is estimated based on elapsed time. If the probe hits a rock, the servo will stall while software reports `INSERTED`. Limit switches should be installed before unsupervised field use.
3. **Wi-Fi Latency**: The ESP32 motor watchdog is 700 ms. If a noisy 2.4 GHz network drops packets for > 600 ms, the rover will experience intermittent pauses. A dedicated Wi-Fi access point running directly on the Pi 5 is strongly recommended.
