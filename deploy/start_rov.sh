#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────────────────
# start_rov.sh — Launch the RYUGU ROV production stack
# ─────────────────────────────────────────────────────────────────────────────
# Designed to be called by systemd (ryugu-rov.service) at boot.
# Sources ROS 2 Humble and the workspace overlay, then exec's the production
# launch file.  The exec keeps systemd's MainPID pointing at the real ROS
# process, so SIGINT / SIGKILL delivery works correctly.
#
# Manual test (from an SSH session):
#   sudo systemctl stop ryugu-rov
#   /home/icad/RYUGU-ROV-CompanionComputer/deploy/start_rov.sh
# ─────────────────────────────────────────────────────────────────────────────

# NOTE: do NOT use `set -u` — ROS 2 setup.bash references variables
# (e.g. AMENT_TRACE_SETUP_FILES) that aren't set in systemd's minimal env.
set -eo pipefail

# ── Paths ───────────────────────────────────────────────────────────────────
readonly ROS_SETUP="/opt/ros/humble/setup.bash"
readonly WS_SETUP="/home/icad/RYUGU-ROV-CompanionComputer/install/setup.bash"

readonly FRONT_CAM="/dev/v4l/by-id/usb-Xiongmai_web_camera_12345678-video-index0"
readonly BOTTOM_CAM="/dev/v4l/by-id/usb-JETE-W7_JETE-W7_202503051344-video-index0"

# ── Log helper ──────────────────────────────────────────────────────────────
log() { echo "[ryugu-rov] $(date '+%Y-%m-%dT%H:%M:%S%z') — $*" >&2; }

# ── CUDA environment (TensorRT / hook_detection_node) ───────────────────────
# systemd starts with a minimal environment — no CUDA paths.  Export them
# BEFORE sourcing ROS 2 so every child process (MAVROS, webcam_streamer,
# hook_detection_node) inherits the runtime libraries.
export CUDA_HOME="/usr/local/cuda"
export PATH="${CUDA_HOME}/bin:${PATH}"
export LD_LIBRARY_PATH="${CUDA_HOME}/lib64:/usr/local/cuda-12.6/lib64:${LD_LIBRARY_PATH:-}"

# ── Source ROS 2 Humble ─────────────────────────────────────────────────────
if [ -f "${ROS_SETUP}" ]; then
    # shellcheck source=/dev/null
    source "${ROS_SETUP}"
    log "Sourced ${ROS_SETUP}"
else
    log "FATAL: ROS 2 Humble setup not found at ${ROS_SETUP}"
    exit 1
fi

# ── Source workspace overlay ────────────────────────────────────────────────
if [ -f "${WS_SETUP}" ]; then
    # shellcheck source=/dev/null
    source "${WS_SETUP}"
    log "Sourced ${WS_SETUP}"
else
    log "FATAL: Workspace setup not found at ${WS_SETUP} — run 'colcon build --symlink-install' first"
    exit 1
fi

# ── Warn about missing camera devices (non-fatal) ───────────────────────────
for cam in "${FRONT_CAM}" "${BOTTOM_CAM}"; do
    if [ ! -e "${cam}" ]; then
        log "WARNING: Camera device not found: ${cam} — will use fallback"
    fi
done

# ── Launch ──────────────────────────────────────────────────────────────────
log "Starting ryugu_GoProduction_QGC.launch.py …"

exec ros2 launch ryugu_control ryugu_GoProduction_QGC.launch.py \
    front_dev:=${FRONT_CAM} \
    bottom_dev:=${BOTTOM_CAM}
