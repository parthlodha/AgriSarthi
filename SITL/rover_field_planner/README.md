# Rover Field Planner

## PC / base-station deployment

Run this app on the personal PC or base station. It uses Streamlit and rosbridge WebSocket only; it does not import `rclpy`, access Pi serial devices, or require ROS 2 installed locally. The Pi must be running ROS 2 and rosbridge on a reachable network address.

### Windows

```bat
cd path\to\Agrisarthi\rover_field_planner
py -m venv .venv
.venv\Scripts\activate
python -m pip install -r requirements.txt
streamlit run app.py
```

### Linux

```bash
cd /path/to/Agrisarthi/rover_field_planner
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
AGRISARTHI_ROSBRIDGE_URL=ws://192.168.1.12:9090 streamlit run app.py
```

Replace the example Pi address with the Raspberry Pi's LAN address. Set the **ROS bridge WebSocket URL** in Project settings to that address; the port is part of this editable URL. The app default can also be set with `AGRISARTHI_ROSBRIDGE_URL`. `localhost` means the PC itself and is incorrect unless rosbridge runs there.

For the laptop-only SITL, set `AGRISARTHI_ROSBRIDGE_URL=ws://127.0.0.1:9090`, `AGRISARTHI_GPS_TOPIC=/sim/gps/fix`, and `AGRISARTHI_SITL_DEMO=1` before `streamlit run app.py`. The SITL demo button submits a deterministic four-waypoint mission around the simulated home point; synthetic soil data is clearly simulation-only.

## Workflow
1. Optionally use Find my location to center the map.
2. Set distance between parallel lines and distance between sample points.
3. Tap at least 3 corners around the field boundary, in order.
4. Press Finish boundary.
5. Inspect the blue route preview; adjust spacing to update it.
6. Press Send mission to rover. The app writes `missions/mission_001.json`, then `mission_002.json`, etc., and publishes the mission JSON to the Pi through rosbridge.
7. Use Refresh rover status and data to query the Pi mission manager, and download the persisted sample JSON from the PC UI.

## Mission format and caveats
The JSON contains mission_id, boundary, spacing, and generated GPS waypoints. The route generator uses a local equirectangular projection and parallel north-south passes; it is intended for small fields. This is a preview/planning implementation, not a safety-certified navigation planner. It does not currently model rover width, turn radius, obstacles, headlands, slope, or GPS uncertainty.

## Cloud persistence and ROS
The `missions/` directory is local to the app runtime. Streamlit cloud filesystems may be ephemeral, so use persistent storage/database/object storage for durable planner history. rosbridge carries mission and start/pause commands to `/mission/waypoints_json` and `/mission/start`; the app requests rover status and saved samples over `/mission/query` and receives `/mission/status_response` and `/mission/data_response`. GPS map centering reads `/gps/fix`. Keep topic names configurable in Project settings and Pi parameters. Do not expose an unauthenticated rosbridge endpoint to the public internet; use a trusted LAN or an authenticated TLS reverse proxy.

## Raspberry Pi deployment boundary

Install ROS 2 Jazzy, the `agrisarthi_gps_nav` package, GPS/RS485 drivers, and rosbridge on the Pi only. Follow the separate Pi instructions in `../agrisarthi_ros_rover/README.md`. Do not install Streamlit on the Pi. The Pi owns navigation, sampling, hardware access, and mission JSON storage; the PC owns the map UI and local planner mission files. Either application can start or stop independently, but network features require the Pi bridge to be online.
