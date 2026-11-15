#!/usr/bin/env bash
set -Ee -o pipefail

# Agrisarthi ROS 2 SITL startup.
# Uses tmux windows so simulation and rosbridge can run together.
REPO_ROOT="/home/parth/projects/Agrisarthi"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
WORKSPACE_SETUP="${REPO_ROOT}/install/setup.bash"
ROS_HOME_DIR="/tmp/agrisarthi-sitl-ros-home"
SESSION="agrisarthi_sitl"

SIM_LAUNCH="ros2 launch agrisarthi_gps_nav simulation.launch.py"
BRIDGE_LAUNCH="ros2 launch rosbridge_server rosbridge_websocket_launch.xml address:=127.0.0.1 port:=9090"

echo "======================================"
echo "      AGRISARTHI ROS SITL STARTUP"
echo "======================================"

if ! command -v tmux >/dev/null 2>&1; then
    echo "ERROR: tmux is not installed."
    echo "Install it once with:"
    echo "  sudo apt update && sudo apt install tmux"
    exit 1
fi

if [[ ! -f "${ROS_SETUP}" ]]; then
    echo "ERROR: ROS 2 Jazzy setup not found: ${ROS_SETUP}"
    exit 1
fi

if [[ ! -f "${WORKSPACE_SETUP}" ]]; then
    echo "ERROR: Workspace setup not found: ${WORKSPACE_SETUP}"
    echo "Build the workspace first from ${REPO_ROOT}."
    exit 1
fi

if [[ ! -d "${REPO_ROOT}" ]]; then
    echo "ERROR: Repository not found: ${REPO_ROOT}"
    exit 1
fi

# Verify required packages in the sourced environment before starting tmux.
# shellcheck disable=SC1090
source "${ROS_SETUP}"
# shellcheck disable=SC1090
source "${WORKSPACE_SETUP}"

if ! ros2 pkg prefix agrisarthi_gps_nav >/dev/null 2>&1; then
    echo "ERROR: ROS package agrisarthi_gps_nav is not available."
    echo "Build/source the workspace and verify the package name."
    exit 1
fi

if ! ros2 pkg prefix rosbridge_server >/dev/null 2>&1; then
    echo "ERROR: rosbridge_server is not available in this ROS environment."
    echo "Install/check rosbridge_suite before starting."
    exit 1
fi

if tmux has-session -t "${SESSION}" 2>/dev/null; then
    echo "A tmux session named '${SESSION}' already exists."
    echo
    echo "Attach to it:"
    echo "  tmux attach -t ${SESSION}"
    echo
    echo "Stop it:"
    echo "  tmux kill-session -t ${SESSION}"
    exit 0
fi

mkdir -p "${ROS_HOME_DIR}"

# Start simulation and rosbridge in separate tmux windows.
tmux new-session -d -s "${SESSION}" -n simulation
tmux set-window-option -g remain-on-exit on
tmux set-window-option -g history-limit 10000

# Each window sources ROS independently and uses the same ROS_HOME.
SIM_CMD="source '${ROS_SETUP}' && source '${WORKSPACE_SETUP}' && export ROS_HOME='${ROS_HOME_DIR}' && cd '${REPO_ROOT}' && ${SIM_LAUNCH}"
BRIDGE_CMD="source '${ROS_SETUP}' && source '${WORKSPACE_SETUP}' && export ROS_HOME='${ROS_HOME_DIR}' && cd '${REPO_ROOT}' && ${BRIDGE_LAUNCH}"

tmux send-keys -t "${SESSION}:simulation" "bash -lc $(printf '%q' "${SIM_CMD}")" C-m
tmux new-window -t "${SESSION}" -n rosbridge
tmux send-keys -t "${SESSION}:rosbridge" "bash -lc $(printf '%q' "${BRIDGE_CMD}")" C-m

echo
echo "SITL processes launched in tmux session: ${SESSION}"
echo
echo "Attach to view both terminal windows:"
echo "  tmux attach -t ${SESSION}"
echo
echo "Inside tmux:"
echo "  Ctrl+B then N  -> next window"
echo "  Ctrl+B then P  -> previous window"
echo "  Ctrl+B then 0  -> simulation"
echo "  Ctrl+B then 1  -> rosbridge"
echo "  Ctrl+B then D  -> detach (processes keep running)"
echo
echo "Stop all SITL ROS processes:"
echo "  tmux kill-session -t ${SESSION}"
echo
echo "WebSocket endpoint: ws://127.0.0.1:9090"
echo "Simulation launch:  ${SIM_LAUNCH}"
echo
echo "Attaching now..."
exec tmux attach -t "${SESSION}"
