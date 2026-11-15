# Agrisarthi autonomous soil-analysis rover — ROS 2 Jazzy

The Raspberry Pi 5 runs ROS 2 Jazzy navigation and the mission sampling manager. The Streamlit planner sends boundary/waypoint missions through rosbridge. The Pi communicates with the ESP32-C6 over acknowledged UDP; the ESP32 drives the motors and the rack-and-pinion probe servo. Soil measurements use the Pi's RS485 Modbus interface.

```text
Personal PC / base station                 Raspberry Pi 5 (ROS 2 Jazzy)                 ESP32-C6
Streamlit + WebSocket client  <─ rosbridge ─> mission manager + navigator  <─ UDP ─> motors + probe servo
local planner mission files                 GPS, RS485, JSON storage
```

These are separate deployments. The PC app has no ROS 2 dependency and does not access Pi hardware. The Pi package has no Streamlit dependency. Only rosbridge crosses the PC/Pi network boundary; only the existing UDP protocol crosses the Pi/ESP32 boundary.

## Topics and behavior

The planner publishes `/mission/waypoints_json` (`std_msgs/String`). The navigator reads `/gps/fix` (`sensor_msgs/NavSatFix`) and `/heading_deg` (`std_msgs/Float32`, degrees clockwise from north), and publishes `/cmd_vel` plus `/rover/nav_status`. Publish `/mission/start true` for an explicit mission start; the mission manager gates this and sends internal starts on `/mission/navigation_start`. `/cmd/estop` remains the software emergency stop.

From the PC, rosbridge also carries a `GET_SNAPSHOT` request on `/mission/query` (`std_msgs/String`). The Pi mission manager answers with status on `/mission/status_response` and the current persisted result document on `/mission/data_response`; the Streamlit app uses this for its status view and JSON download. The manager workflow status is also published on `/mission/sampling_status`.

Every route must contain a non-self-intersecting WGS84 `boundary` and non-empty `waypoints`, all inside the boundary. Navigation holds at each arrival. The mission manager confirms stopped motors, inserts the probe, reads and validates the sensor, atomically saves the result, retracts the probe, then acknowledges waypoint completion. The navigator cannot advance before this acknowledgement. Sensor or actuator failures do not complete the waypoint; publish `RETRY_SAMPLE`, `RETRACT`, `COMPLETE_SAVED_WAYPOINT`, or `RESUME` to `/mission/sampling_recovery` only after operator assessment. `/mission/sampling_status` reports workflow state. Geofence, sensor freshness, E-stop and ESP32 communication checks stop or hold navigation; automatic safety recovery is disabled.

The protocol uses a new session token after each Pi process start, monotonically increasing 32-bit sequence numbers, idempotency-keyed probe commands, acknowledgements, ESP32 actuator feedback and a 700 ms motor watchdog. ESP32 firmware disables motor output unless the probe state is RETRACTED. UDP remains unauthenticated: keep it on a trusted local network.

## Soil result JSON

One file per mission is written under the configured `data_directory` (default `~/agrisarthi_ws/missions/data`). The ROS parameter `rover_id` is configurable. Supply a surveyed `home_point` in the mission or configure `home_latitude` and `home_longitude` in `hardware.yaml`; placeholder values intentionally prevent sampling until configured. The published schema is `agrisarthi_gps_nav/config/mission_data.schema.json` and validation is strict. Each record has only `waypoint_id`, UTC `timestamp`, actual sample-time `gps`, and `soil`. Top-level keys are `rover_id`, `mission_id`, `home_point`, and `records`. No rover telemetry is collected or written.

`soil.ec` preserves the integer conductivity register without conversion. The provided sensor script prints a µS/cm label, but no sensor-specific register specification was available to verify the unit. Nitrogen/phosphorus/potassium use register `0x001E`, falling back to `0x0004` when the primary response is missing or all zero. Both NPK responses all-zero is a failed sample, never a successful zero reading.

Example record (the values are illustrative; firmware and software do not fabricate readings):

```json
{
  "rover_id": "ROVER_01",
  "mission_id": "MISSION_001",
  "home_point": {"latitude": 19.0760, "longitude": 72.8777},
  "records": [
    {
      "waypoint_id": 1,
      "timestamp": "2026-09-26T20:15:30Z",
      "gps": {"latitude": 19.0760, "longitude": 72.8777},
      "soil": {"nitrogen": 72, "phosphorus": 38, "potassium": 165, "ph": 6.4,
               "moisture": 58.3, "temperature": 27.5, "ec": 1450}
    }
  ]
}
```

## Build and launch on Raspberry Pi 5

Copy `agrisarthi_gps_nav` into `~/agrisarthi_ws/src/`, then:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/agrisarthi_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch agrisarthi_gps_nav hardware.launch.py
```

Edit `agrisarthi_gps_nav/config/hardware.yaml` for ESP32 IP, surveyed home coordinate, sensor serial port, and speed limits. Connect the RS485 adapter to the sensor's A/B and reference terminals; verify adapter direction control and voltage compatibility. The required serial settings default to slave 1, 4800 baud, 8N1. Add the ROS user to `dialout` and confirm the stable path (for example `/dev/serial/by-id/...`). Install and run rosbridge on the Pi in a second terminal:

```bash
sudo apt install ros-jazzy-rosbridge-server
source /opt/ros/jazzy/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml address:=0.0.0.0 port:=9090
```

Replace `0.0.0.0` and `9090` with the intended bind address and port as required by your network. In the PC Streamlit settings, set `ws://<PI_LAN_IP>:9090` (or an authenticated `wss://` proxy). Allow TCP access only from the base station on a trusted LAN/firewall. Never expose unauthenticated rosbridge or motor UDP to the public internet.

## UART GPS on the Pi

Use a 3.3 V UART-compatible GNSS receiver. Enable the Pi UART, identify the stable device path with `ls -l /dev/serial/by-id/` (USB UART) or use `/dev/serial0` for the configured Pi UART, and add the ROS user to the `dialout` group (`sudo usermod -aG dialout "$USER"`, then log in again). Confirm the receiver baud rate from its manual. Install a Jazzy-compatible ROS 2 `nmea_navsat_driver` build (upstream ROS 2 source: `ros-drivers/nmea_navsat_driver`, branch `ros2`), then run it in a second terminal, adjusting the device and baud:

```bash
ros2 run nmea_navsat_driver nmea_serial_driver --ros-args \
  -p port:=/dev/serial/by-id/REPLACE_WITH_GNSS \
  -p baud:=9600 -p frame_id:=gps_link -r fix:=/gps/fix
ros2 topic echo /gps/fix
```

The UART driver supplies `NavSatFix`; the navigator checks fix status, finite WGS84 coordinates, optional covariance, and receive-time freshness. Do not connect 5 V or RS-232 signaling directly to Pi GPIO. The heading interface stays separate: publish calibrated `Float32` degrees clockwise from north on `/heading_deg`; raw IMU yaw requires a dedicated conversion/calibration node.

## ESP32-C6 motor and probe firmware

In `esp32c6_firmware/esp32c6_udp_motor_receiver.ino`, set Wi-Fi credentials and verify GPIO pins and H-bridge truth table for the exact board. Configure probe servo pin, insertion/retraction angles, travel, settling and timeout constants. The example angles are not mechanically certified: calibrate travel limits against the physical rack-and-pinion while the drive wheels are raised. Optional active-low limit switches provide endpoint feedback. Without switches, `INSERTED`/`RETRACTED` mean the servo command elapsed, not that physical position was verified; add limit switches before relying on position confirmation. Keep a physical E-stop wired independently of software. The firmware reports commanded state and motor PWM, not wheel rotation or soil contact.

## Simulation and limits

`simulation.launch.py` runs the production navigator and mission sampling manager with ROS-only simulated hardware. The deterministic planar world integrates movement from acknowledged simulated ESP32 motor output, publishes `/sim/gps/fix` and `/sim/heading_deg`, and stops motion whenever the simulated probe is not retracted. The simulated ESP32 uses the production JSON command/ack contract but opens no UDP socket. The simulated soil source returns deterministic synthetic data and opens no serial port. This is software-only validation; no values represent physical sensor readings. There is no Gazebo world in this repository.

### Laptop four-waypoint SITL demonstration

Build once from the repository root, then use three terminals. Terminal 1 starts all simulated ROS packages:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths agrisarthi_ros_rover --packages-select agrisarthi_gps_nav --symlink-install
source install/setup.bash
export ROS_HOME=/tmp/agrisarthi-sitl-ros-home
ros2 launch agrisarthi_gps_nav simulation.launch.py
```

Terminal 2 starts a laptop-local WebSocket bridge (no external network required):

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_HOME=/tmp/agrisarthi-sitl-ros-home
ros2 launch rosbridge_server rosbridge_websocket_launch.xml address:=127.0.0.1 port:=9090
```

Terminal 3 runs Streamlit on the PC. Install `rover_field_planner/requirements.txt` in a Python virtual environment first:

```bash
cd rover_field_planner
source .venv/bin/activate
export AGRISARTHI_ROSBRIDGE_URL=ws://127.0.0.1:9090
export AGRISARTHI_GPS_TOPIC=/sim/gps/fix
export AGRISARTHI_SITL_DEMO=1
streamlit run app.py
```

Open `http://localhost:8501`, click **Send four-waypoint SITL mission**, confirm the mission snapshot reports `MISSION_READY` and probe `RETRACTED`, then click **Start / Resume** once. The demo moves between GPS targets; it does not teleport. The manager saves results incrementally to `/tmp/agrisarthi_sitl/data/`. Inspect `/rover/nav_status`, `/mission/sampling_status`, `/sim/esp32/command`, `/sim/esp32/feedback`, `/sim/gps/fix`, `/sim/heading_deg`, and `/cmd_vel` with `ros2 topic echo <topic>`.

Fault injection topics for controlled SITL checks are `/sim/soil/faults` (`{"action":"FAIL_NEXT_READS","count":1}`), `/sim/esp32/faults` (`{"action":"FAIL_NEXT_RETRACTION"}`), and `/sim/world/faults` (`{"action":"GPS_LOSS"}` or `{"action":"GPS_RESTORE"}`), each as `std_msgs/msg/String`. Duplicate acknowledgements can be enabled in `simulation.yaml`. Restart or reupload a mission after latched failures; recovery is explicit.

Obstacle avoidance is intentionally out of scope. Geofence checking constrains the current GPS point and waypoint list; it cannot prevent a path between waypoints from cutting across a concave boundary. Software tests do not verify the Pi UART/RS485 adapter, sensor calibration, servo mechanics, limit switches, radio link, motor wiring, or field behavior. SITL uses synthetic deterministic soil values; it does not access physical ESP32 or sensor hardware and does not validate physical rover behavior.
