# Bring-up checklist

- [ ] ROS 2 Jazzy is sourced and package builds.
- [ ] `/mission/waypoints_json` is `std_msgs/msg/String`.
- [ ] `/gps/fix` is valid WGS84 `sensor_msgs/msg/NavSatFix`.
- [ ] `/heading_deg` is `std_msgs/msg/Float32`, degrees clockwise from north.
- [ ] ESP32 IP/port are correct and both devices are on the same network.
- [ ] Keep motor driver disconnected or wheels raised for first tests.
- [ ] Stale GPS stops motors; stale heading stops motors.
- [ ] `/cmd/estop` true stops motors; clearing it does not resume until conditions are valid.
- [ ] Mission completion stops motors.
- [ ] Check forward and turn direction signs at very low speed.
- [ ] Add a physical E-stop and obstacle sensing before ground operation.

Mission start/pause:
```bash
ros2 topic pub --once /mission/start std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /mission/start std_msgs/msg/Bool "{data: false}"
```
Confirm a received mission creates `~/agrisarthi_ws/missions/current_mission.json` and that restarting the node loads it as `MISSION_READY` without moving.
