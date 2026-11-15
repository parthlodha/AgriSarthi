# Repository Guidelines

## Project Structure

`agrisarthi_ros_rover/agrisarthi_gps_nav/` contains the ROS 2 Jazzy Python package. ROS adapters are in `agrisarthi_gps_nav/`; navigation calculations, UDP handling, soil acquisition, mission persistence, and sampling orchestration are separate modules there. Hardware and simulation parameters and launch files live in `config/` and `launch/`. ROS-independent Python tests are in `test/`. ESP32-C6 motor and probe firmware, protocol helpers, and host-side C++ checks are in `agrisarthi_ros_rover/esp32c6_firmware/`. The Streamlit mission planner is in `rover_field_planner/`.

## Build, Test, and Development

From the repository root, build the ROS package with:

```bash
source /opt/ros/jazzy/setup.bash
cd agrisarthi_ros_rover
colcon build --symlink-install
```

Run Python unit tests with `PYTHONPATH=agrisarthi_ros_rover/agrisarthi_gps_nav python3 -m unittest discover -s agrisarthi_ros_rover/agrisarthi_gps_nav/test -v`. Compile host-side firmware checks using `g++ -std=c++17` on the corresponding source under `esp32c6_firmware/test/`. Hardware launch is `ros2 launch agrisarthi_gps_nav hardware.launch.py`; configure `config/hardware.yaml` first. No Gazebo world is included.

## Coding and Safety Conventions

Use four spaces in Python, descriptive `snake_case` names, and type hints for reusable logic. Keep calculations and protocol behavior testable outside ROS nodes. Firmware uses the existing Arduino C++ style and explicit motor-safe states. Do not add obstacle avoidance or alter GPS/heading hardware configuration without a project decision. Never treat commanded servo angle as physical endpoint feedback when limit switches are absent. Do not fabricate sensor data or acknowledgements.

## Tests and Mission Data

Name Python tests `test_*.py` and use `unittest`. Test navigation holds, safety stops, Modbus response validation, duplicate commands, and atomic JSON persistence. Sample output must follow `config/mission_data.schema.json`: root keys are `rover_id`, `mission_id`, `home_point`, and `records`; each record contains only `waypoint_id`, `timestamp`, `gps`, and `soil`. Do not add battery, speed, status, or a per-record `rover` object.

## Changes and Reviews

The existing Git history uses a conventional prefix (for example, `chore:`). Use focused messages such as `feat: gate waypoint progress on soil sampling`. Pull requests should describe behavior and safety effects, list test commands/results, identify hardware-only validation still needed, and include screenshots only for planner UI changes. Keep credentials out of commits and do not commit unless requested.
