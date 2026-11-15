# Rover Field Planner

## Run on Windows
```bat
py -m venv .venv
.venv\Scripts\activate
python -m pip install -r requirements.txt
streamlit run app.py
```

## Workflow
1. Optionally use Find my location to center the map.
2. Set distance between parallel lines and distance between sample points.
3. Tap at least 3 corners around the field boundary, in order.
4. Press Finish boundary.
5. Inspect the blue route preview; adjust spacing to update it.
6. Press Send mission to rover. The app writes `missions/mission_001.json`, then `mission_002.json`, etc., and attempts to publish the mission JSON to ROS.

## Mission format and caveats
The JSON contains mission_id, boundary, spacing, and generated GPS waypoints. The route generator uses a local equirectangular projection and parallel north-south passes; it is intended for small fields. This is a preview/planning implementation, not a safety-certified navigation planner. It does not currently model rover width, turn radius, obstacles, headlands, slope, or GPS uncertainty.

## Cloud persistence and ROS
The `missions/` directory is local to the app runtime. Streamlit cloud filesystems may be ephemeral, so use persistent storage/database/object storage for durable mission history. `ws://localhost:9090` points to the app's own host, not your rover. Configure a secured reachable `wss://` bridge or backend relay. Do not expose rosbridge to the public internet without authentication and network controls.
