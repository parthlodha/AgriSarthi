# Agrisarthi

This repository keeps the two deployment workspaces side by side:

- **SITL/** — laptop software-in-the-loop project, including the simulated world, simulated ESP32 and soil sensor, planner, ROS 2 packages, tests, and firmware sources.
- **HARDWARE/** — Raspberry Pi rover project, including physical GPS and heading drivers, hardware configuration and launch, RS485 sensor access, planner, ROS 2 packages, tests, and ESP32 firmware.

The folders are copies of the tracked project at `sitl` (`3831c14`) and `hardware` (`7db9e03`), respectively. The original root-level files remain in place. Compatibility modules needed by imports or registered entry points are retained in HARDWARE. Its `hardware.launch.py` starts hardware drivers and shared mission nodes only; simulation nodes are not started by that launch.

## Build SITL

From `SITL/agrisarthi_ros_rover` with ROS 2 Jazzy and `colcon` installed:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch agrisarthi_gps_nav simulation.launch.py
```

## Build hardware

From `HARDWARE/agrisarthi_ros_rover` on the Raspberry Pi 5:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch agrisarthi_gps_nav hardware.launch.py
```

Configure surveyed coordinates, serial devices, and network addresses using the hardware guide before field use. Run the Streamlit planner from the corresponding folder's `rover_field_planner` directory with its listed requirements. Do not run hardware motors or probes based only on simulation results.
