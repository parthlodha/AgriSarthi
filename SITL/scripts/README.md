# Agrisarthi SITL startup scripts

These scripts use the paths and launch commands from the reported SITL run.

## One-time requirements

- ROS 2 Jazzy installed at `/opt/ros/jazzy`
- Built workspace at `/home/parth/projects/Agrisarthi/install`
- `rosbridge_server` available in the sourced ROS environment
- `tmux` installed (`sudo apt install tmux`)
- Streamlit virtual environment at `rover_field_planner/.venv`
- Planner requirements installed in that virtual environment

If the repository is not at `/home/parth/projects/Agrisarthi`, edit `REPO_ROOT` in both scripts.

## Start

Terminal 1:

```bash
cd /home/parth/projects/Agrisarthi
./scripts/start_ros.sh
```

This creates a tmux session with:
- `simulation`: `agrisarthi_gps_nav simulation.launch.py`
- `rosbridge`: rosbridge WebSocket on `127.0.0.1:9090`

Terminal 2:

```bash
cd /home/parth/projects/Agrisarthi
./scripts/start_pc.sh
```

Open `http://localhost:8501`.

## Stop

- Stop Streamlit with `Ctrl+C` in its terminal.
- Stop the ROS simulation and rosbridge together with:

```bash
tmux kill-session -t agrisarthi_sitl
```

## Reattach / inspect

```bash
tmux attach -t agrisarthi_sitl
```

Inside tmux, use `Ctrl+B` then `N` / `P` to switch windows. Use `Ctrl+B` then `D` to detach without stopping processes.

## Notes

- This starts the SITL launch only; it never launches `hardware.launch.py`.
- It does not configure or contact a physical ESP32.
- It does not rebuild the ROS workspace automatically. Build separately when source changes.
- The Streamlit script expects the existing `.venv`; it does not create or install dependencies automatically.
