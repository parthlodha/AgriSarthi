# Agrisarthi startup scripts

Startup scripts for running both the SITL simulation and physical Raspberry Pi 5 hardware deployments.

## One-time requirements

- ROS 2 Jazzy installed at `/opt/ros/jazzy`
- Built workspace at `/home/parth/projects/Agrisarthi/install`
- `rosbridge_server` available in the sourced ROS environment (`sudo apt install ros-jazzy-rosbridge-server`)
- `tmux` installed (`sudo apt install tmux`)
- Streamlit virtual environment at `rover_field_planner/.venv`
- Planner requirements installed in that virtual environment

If the repository is not at `/home/parth/projects/Agrisarthi`, edit `REPO_ROOT` in the scripts.

---

## 1. Physical Hardware Deployment (Raspberry Pi 5)

### Step 1: Start Hardware ROS Stack and rosbridge on Raspberry Pi

In a terminal on the Pi 5:

```bash
cd /home/parth/projects/Agrisarthi
./scripts/start_hardware.sh
```

This creates a tmux session (`agrisarthi_hardware`) running:
- `hardware`: `ros2 launch agrisarthi_gps_nav hardware.launch.py` (GPS, heading, navigator, sampling manager)
- `rosbridge`: rosbridge WebSocket bound to `0.0.0.0:9090`

### Step 2: Start Streamlit Planner on PC / Base Station

In a terminal on the PC (connected to the same LAN / Wi-Fi as the Pi):

```bash
cd /path/to/Agrisarthi/rover_field_planner
source .venv/bin/activate
export AGRISARTHI_ROSBRIDGE_URL="ws://<PI_LAN_IP>:9090"
export AGRISARTHI_GPS_TOPIC="/gps/fix"
export AGRISARTHI_SITL_DEMO="0"
streamlit run app.py
```

Open `http://localhost:8501`.

### Stop Hardware:

```bash
tmux kill-session -t agrisarthi_hardware
```

---

## 2. Laptop-Only SITL Simulation (No Hardware)

### Terminal 1 (ROS SITL + rosbridge):

```bash
cd /home/parth/projects/Agrisarthi
./scripts/start_ros.sh
```

This creates a tmux session (`agrisarthi_sitl`) with:
- `simulation`: `agrisarthi_gps_nav simulation.launch.py`
- `rosbridge`: rosbridge WebSocket on `127.0.0.1:9090`

### Terminal 2 (PC Streamlit):

```bash
cd /home/parth/projects/Agrisarthi
./scripts/start_pc.sh
```

Open `http://localhost:8501`.

### Stop SITL:

```bash
tmux kill-session -t agrisarthi_sitl
```

---

## Reattach / Inspect tmux sessions

- For hardware: `tmux attach -t agrisarthi_hardware`
- For SITL: `tmux attach -t agrisarthi_sitl`

Inside tmux:
- `Ctrl+B` then `0` -> first window
- `Ctrl+B` then `1` -> second window
- `Ctrl+B` then `D` -> detach safely without stopping processes
