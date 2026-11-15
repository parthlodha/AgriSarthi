#!/usr/bin/env bash
set -Eeuo pipefail

# Agrisarthi PC / Streamlit startup (SITL)
REPO_ROOT="/home/parth/projects/Agrisarthi"
APP_DIR="${REPO_ROOT}/rover_field_planner"
VENV_DIR="${APP_DIR}/.venv"

echo "======================================"
echo "       AGRISARTHI PC STARTUP"
echo "======================================"

if [[ ! -d "${APP_DIR}" ]]; then
    echo "ERROR: Streamlit application directory not found:"
    echo "  ${APP_DIR}"
    echo "Edit REPO_ROOT in this script if your repository is elsewhere."
    exit 1
fi

if [[ ! -f "${VENV_DIR}/bin/activate" ]]; then
    echo "ERROR: Python virtual environment not found:"
    echo "  ${VENV_DIR}"
    echo
    echo "Create it once with:"
    echo "  cd '${APP_DIR}'"
    echo "  python3 -m venv .venv"
    echo "  source .venv/bin/activate"
    echo "  python -m pip install -r requirements.txt"
    exit 1
fi

cd "${APP_DIR}"
# shellcheck disable=SC1091
source "${VENV_DIR}/bin/activate"

export AGRISARTHI_ROSBRIDGE_URL="ws://127.0.0.1:9090"
export AGRISARTHI_GPS_TOPIC="/sim/gps/fix"
export AGRISARTHI_SITL_DEMO="1"

if ! python -c "import streamlit" >/dev/null 2>&1; then
    echo "ERROR: Streamlit is not installed in ${VENV_DIR}."
    echo "Install project requirements with:"
    echo "  python -m pip install -r requirements.txt"
    exit 1
fi

echo "Repository : ${REPO_ROOT}"
echo "App        : ${APP_DIR}/app.py"
echo "Python     : $(command -v python)"
echo "ROS Bridge : ${AGRISARTHI_ROSBRIDGE_URL}"
echo "GPS topic  : ${AGRISARTHI_GPS_TOPIC}"
echo "Mode       : SITL"
echo
echo "Starting Streamlit..."
echo "Open http://localhost:8501 in your browser."
echo "Press Ctrl+C to stop Streamlit."
echo

exec python -m streamlit run app.py
