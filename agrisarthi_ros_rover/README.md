# Agrisarthi GPS waypoint rover — ROS 2 Jazzy starter

Receives the Streamlit field planner JSON through ROS 2, navigates GPS waypoints, and sends left/right motor commands to an ESP32-C6 over UDP/Wi-Fi.

## Data flow

`Streamlit -> rosbridge -> /mission/waypoints_json (std_msgs/String) -> gps_waypoint_navigator -> UDP JSON -> ESP32-C6 -> H-bridge -> motors`

Inputs: `/gps/fix` (`sensor_msgs/NavSatFix`, WGS84), `/heading_deg` (`std_msgs/Float32`, degrees clockwise from north), `/cmd/estop` (`std_msgs/Bool`). Output: `/rover/nav_status` (`std_msgs/String`, JSON status).

The node accepts a `waypoints` array containing `{id, latitude, longitude}`. Extra fields such as `mission_id`, `boundary`, `spacing_m`, and `created_at` are ignored. It uses haversine distance and bearing, a simple proportional heading controller, and the order of waypoints supplied by the planner. It stops on missing/stale GPS or heading, software E-stop, no mission, and mission completion. The ESP32 sample independently stops when command packets become stale.

## Build on ROS 2 Jazzy

Extract this ZIP, then copy the `agrisarthi_gps_nav` folder into `~/agrisarthi_ws/src/` so the path is `~/agrisarthi_ws/src/agrisarthi_gps_nav`.

```bash
source /opt/ros/jazzy/setup.bash
mkdir -p ~/agrisarthi_ws/src
cd ~/agrisarthi_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Run

Find the ESP32 IP from its serial output and use that address:

```bash
ros2 run agrisarthi_gps_nav gps_waypoint_navigator --ros-args \
  -p esp32_ip:=192.168.1.50 -p esp32_port:=5005 \
  -p mission_topic:=/mission/waypoints_json \
  -p gps_topic:=/gps/fix -p heading_topic:=/heading_deg
```

Replace `192.168.1.50` with the real ESP32 IP. Pi and ESP32 must be on the same reachable Wi-Fi network. Configure a DHCP reservation/static IP. For Streamlit-to-ROS, install and launch rosbridge:

```bash
sudo apt install ros-jazzy-rosbridge-server
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

Set the planner's WebSocket URL to `ws://<ROS_MACHINE_IP>:9090`, not `localhost` unless Streamlit runs on that same machine. Do not expose unauthenticated rosbridge or UDP motor commands to the public internet.

## Heading input requirement

`/heading_deg` must be `std_msgs/Float32` in degrees clockwise from north (0=N, 90=E). Do not wire raw IMU yaw into this topic until its frame convention, orientation, and calibration are verified. Raw yaw is often referenced to a different axis/convention. GPS course-over-ground is only useful while moving and should not be treated as a reliable stationary heading.

## ESP32-C6 firmware

Open `esp32c6_firmware/esp32c6_udp_motor_receiver.ino` in Arduino IDE. Install ArduinoJson v7, select the correct ESP32-C6 board, set Wi-Fi credentials and verify every example GPIO pin against your exact board and motor driver. This example assumes a generic dual H-bridge with one direction and one PWM pin per motor; modify `setMotor()` to match the driver's truth table. Never connect motors directly to GPIO.

UDP command example:

```json
{"type":"motor_cmd","mission_id":"001","seq":12,"enable":true,"left":0.18,"right":0.12,"linear":0.15,"angular":0.12}
```

`left` and `right` are normalized from -1 to +1. Positive means forward in this example. `enable:false` stops motors. Firmware watchdog timeout is 700 ms.

## Safety and limitations

This is an initial integration scaffold, not safety-certified autonomy. It has no obstacle avoidance, encoders/odometry, geofence enforcement, terrain handling, GPS filtering, route smoothing, or physical E-stop integration. GPS-only control can be unreliable near trees/buildings and at low speed. Test with the drive wheels raised and motor power disconnected initially; verify heading and steering signs at very low speed. Add a physical E-stop, obstacle sensing, watchdog testing, and a controlled field-test procedure before outdoor use. UDP is unauthenticated and must stay on a trusted network.

## Test commands

```bash
ros2 topic echo /gps/fix
ros2 topic echo /heading_deg
ros2 topic echo /rover/nav_status
ros2 topic pub --once /cmd/estop std_msgs/msg/Bool "{data: true}"
```

A minimal test mission can be published using:

```bash
ros2 topic pub --once /mission/waypoints_json std_msgs/msg/String \
  "{data: '{\"mission_id\":\"TEST001\",\"waypoints\":[{\"id\":1,\"latitude\":19.1234,\"longitude\":72.1234}]}' }"
```

## GPS connection recommendation

For this ROS-centric architecture, connect the GPS receiver directly to the Raspberry Pi 5 (USB GPS or UART receiver with correct voltage levels) so the ROS navigation node receives `sensor_msgs/msg/NavSatFix` directly. Keep the ESP32-C6 focused on motor actuation and its command watchdog. A GPS attached to the ESP32-C6 is also possible, but then you must implement reliable GPS forwarding to ROS and carefully handle timestamps/validity; it does not remove the need for a heading source.
