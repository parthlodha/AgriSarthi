#!/usr/bin/env bash
set -Ee -o pipefail

# Agrisarthi ROS 2 Hardware Startup Script for Raspberry Pi 5.
# Launches the physical hardware ROS stack and rosbridge in a tmux session.

REPO_ROOT="/home/parth/projects/Agrisarthi"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
WORKSPACE_SETUP="${REPO_ROOT}/install/setup.bash"
SESSION="agrisarthi_hardware"

HARDWARE_LAUNCH="ros2 launch agrisarthi_gps_nav hardware.launch.py"
BRIDGE_LAUNCH="ros2 launch rosbridge_server rosbridge_websocket_launch.xml address:=0.0.0.0 port:=9090"

echo "=========================================================="
echo "         AGRISARTHI ROS 2 HARDWARE STACK STARTUP"
echo "=========================================================="

if ! command -v tmux >/dev/null 2>&1; then
    echo "ERROR: tmux is not installed."
    echo "Install it once with: sudo apt update && sudo apt install tmux"
    exit 1
fi

if [[ ! -f "${ROS_SETUP}" ]]; then
    echo "ERROR: ROS 2 Jazzy setup not found: ${ROS_SETUP}"
    exit 1
fi

if [[ ! -f "${WORKSPACE_SETUP}" ]]; then
    echo "ERROR: Workspace setup not found: ${WORKSPACE_SETUP}"
    echo "Build the workspace first from ${REPO_ROOT} with:"
    echo "  cd agrisarthi_ros_rover && colcon build --symlink-install"
    exit 1
fi

if [[ ! -d "${REPO_ROOT}" ]]; then
    echo "ERROR: Repository directory not found: ${REPO_ROOT}"
    exit 1
fi

# Source environments to verify packages
# shellcheck disable=SC1090
source "${ROS_SETUP}"
# shellcheck disable=SC1090
source "${WORKSPACE_SETUP}"

if ! ros2 pkg prefix agrisarthi_gps_nav >/dev/null 2>&1; then
    echo "ERROR: ROS package agrisarthi_gps_nav is not found in the sourced workspace."
    exit 1
fi

if ! ros2 pkg prefix rosbridge_server >/dev/null 2>&1; then
    echo "ERROR: rosbridge_server is not available."
    echo "Install with: sudo apt install ros-jazzy-rosbridge-server"
    exit 1
fi

if tmux has-session -t "${SESSION}" 2>/dev/null; then
    echo "A tmux session named '${SESSION}' is already running."
    echo "Attach:  tmux attach -t ${SESSION}"
    echo "Stop:    tmux kill-session -t ${SESSION}"
    exit 0
fi

# Start hardware nodes and rosbridge in separate tmux windows
tmux new-session -d -s "${SESSION}" -n hardware
tmux set-window-option -g remain-on-exit on
tmux set-window-option -g history-limit 10000

HW_CMD="source '${ROS_SETUP}' && source '${WORKSPACE_SETUP}' && cd '${REPO_ROOT}' && ${HARDWARE_LAUNCH}"
BRIDGE_CMD="source '${ROS_SETUP}' && source '${WORKSPACE_SETUP}' && cd '${REPO_ROOT}' && ${BRIDGE_LAUNCH}"

tmux send-keys -t "${SESSION}:hardware" "bash -lc $(printf '%q' "${HW_CMD}")" C-m
tmux new-window -t "${SESSION}" -n rosbridge
tmux send-keys -t "${SESSION}:rosbridge" "bash -lc $(printf '%q' "${BRIDGE_CMD}")" C-m

echo
echo "Hardware stack running in background tmux session: ${SESSION}"
echo
echo "Attach to view terminal output:"
echo "  tmux attach -t ${SESSION}"
echo
echo "Inside tmux:"
echo "  Ctrl+B then 0  -> hardware nodes (navigator, manager, GPS, heading)"
echo "  Ctrl+B then 1  -> rosbridge websocket server (:9090)"
echo "  Ctrl+B then D  -> detach (processes continue running in background)"
echo
echo "Stop all hardware processes:"
echo "  tmux kill-session -t ${SESSION}"
echo
echo "Attaching now..."
exec tmux attach -t "${SESSION}"
