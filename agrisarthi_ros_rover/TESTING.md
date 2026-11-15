# Tests and bring-up

## Automated software tests

From the repository root, run the ROS-independent test suite:

```bash
PYTHONPATH=agrisarthi_ros_rover/agrisarthi_gps_nav \
  python3 -m unittest discover -s agrisarthi_ros_rover/agrisarthi_gps_nav/test -v
```

The tests cover mission/geofence validation, arrival hold and explicit sampling completion, steering sign, speed scaling, GPS and heading loss, invalid mission clearing, UDP handshake/ack timeout, new Pi sessions, Modbus CRC and register mapping, failed NPK behavior, strict result JSON validation, duplicate records, and atomic persistence. UDP tests use a local Python peer; they do not exercise hardware.

The ESP32 sequence wrap helper can be checked on a development machine with a C++ compiler:

```bash
g++ -std=c++17 agrisarthi_ros_rover/esp32c6_firmware/test/esp32_protocol_test.cpp \
  -o /tmp/agrisarthi-esp32-protocol-test
/tmp/agrisarthi-esp32-protocol-test
```

The probe state machine is separately host-testable:

```bash
g++ -std=c++17 agrisarthi_ros_rover/esp32c6_firmware/test/probe_fsm_test.cpp \
  -o /tmp/agrisarthi-probe-fsm-test
/tmp/agrisarthi-probe-fsm-test
```

These check protocol helpers and interlock transitions, including wrap from `UINT32_MAX` to zero. They are not an ESP32 Arduino build and do not verify the servo or physical endpoints.

## Laptop software-in-the-loop

Build and launch the integrated simulated stack using the three-terminal sequence in `README.md`. With the dashboard set to `ws://127.0.0.1:9090` and GPS topic `/sim/gps/fix`, send the four-waypoint SITL mission, verify `MISSION_READY` and `probe_state: RETRACTED`, then explicitly start it. Expect four records in `/tmp/agrisarthi_sitl/data/mission_<id>_samples.json`, then `MISSION_COMPLETE` after return home. Confirm records contain only `waypoint_id`, `timestamp`, `gps`, and `soil`.

Inspect the complete path through `/sim/esp32/command` and `/sim/esp32/feedback`. The simulated world accepts movement only when feedback reports enabled motors and a retracted probe. Inject `FAIL_NEXT_READS`, `FAIL_NEXT_RETRACTION`, and `GPS_LOSS` on their `/sim/.../faults` topics to check retry, stationary failure, and latched stop behavior. Duplicate ACKs are enabled by default and the ROS protocol unit test verifies de-duplication. Synthetic soil values and planar GPS are not hardware readings or a Gazebo dynamics model.

## Pi and sensor checks

- [ ] Set Wi-Fi credentials, board-specific GPIOs, surveyed home coordinates, and `/dev/serial/by-id/...` sensor path in `hardware.yaml`.
- [ ] Build and launch with `hardware.launch.py`; confirm both navigator and sampling manager start.
- [ ] Connect the GNSS UART at the receiver's documented voltage and baud; confirm fresh valid `/gps/fix` data.
- [ ] Confirm `/heading_deg` is calibrated degrees clockwise from north (0 north, 90 east).
- [ ] Verify planner and navigator topics match; mission boundary and waypoints must be WGS84 and inside the field boundary.
- [ ] Inspect `/rover/nav_status` for mission acceptance and `esp32_connected: true` before starting.
- [ ] Confirm `/mission/sampling_status` reaches `IDLE` after ESP32 handshake reports the probe retracted. `IDLE` at power-up is not treated as retracted; the manager requests a safe retract first.
- [ ] Test with driver disconnected or wheels raised; confirm forward/clockwise signs at low speed.
- [ ] Stop GPS, heading, and ESP32 traffic separately; verify the controller stops and needs a new `/mission/start true` after recovery.
- [ ] Send malformed and out-of-bound missions; confirm status is `MISSION_REJECTED`, the previous route is cleared, and no movement occurs.
- [ ] Restart the Pi process while the ESP32 remains on; verify a new handshake session is accepted.
- [ ] Verify the ESP32 watchdog by removing Pi/UDP traffic; expect motor PWM to stop within 700 ms.
- [ ] With wheels raised, validate probe angles and timeout in short increments. Confirm motors remain disabled in INSERTING, INSERTED, RETRACTING, and ERROR states.
- [ ] Confirm RS485 settings are slave 1, 4800 baud, 8N1 and test all registers before using a field mission. Inspect the mission result JSON after each saved sample; each record may contain only `waypoint_id`, `timestamp`, `gps`, and `soil`.
- [ ] Confirm failed sensor reads never add records and never advance the waypoint; check safe retraction and explicit recovery behavior.
- [ ] Confirm a physical E-stop works independently before ground operation.

Manual start/pause (the mission manager gates the navigator):

```bash
ros2 topic pub --once /mission/start std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /mission/start std_msgs/msg/Bool "{data: false}"
```

Recovery actions are operator decisions, not automatic retries after a fault:

```bash
ros2 topic pub --once /mission/sampling_recovery std_msgs/msg/String "{data: RETRY_SAMPLE}"
ros2 topic echo /mission/sampling_status
```

The JSON result schema is `agrisarthi_ros_rover/agrisarthi_gps_nav/config/mission_data.schema.json`. Sample files intentionally contain no battery, speed, status, or `rover` object. `ec` is the raw conductivity register; its physical unit remains unverified.

`simulation.launch.py` starts the deterministic laptop SITL world and simulated ESP32 interface. Keep it separate from `hardware.launch.py`; no physical UART, soil sensor, or UDP motor command is opened by the SITL configuration. This repository has no Gazebo integration; no simulation run is represented as hardware verification.
