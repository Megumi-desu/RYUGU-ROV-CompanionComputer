#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────────────────
# start_rov.sh — Start the RYUGU ROV production stack
# ─────────────────────────────────────────────────────────────────────────────
# This script is designed to be executed by systemd at boot.
# It sources the ROS2 Humble environment and the RYUGU-ROV workspace.
# ─────────────────────────────────────────────────────────────────────────────

# Exit immediately if a command exits with a non-zero status
set -e

# Source ROS2 Humble global environment
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
else
    echo "Error: ROS2 Humble setup.bash not found at /opt/ros/humble/setup.bash" >&2
    exit 1
fi

# Source RYUGU ROV workspace environment
WORKSPACE_SETUP="/home/icad/RYUGU-ROV/install/setup.bash"
if [ -f "$WORKSPACE_SETUP" ]; then
    source "$WORKSPACE_SETUP"
else
    echo "Error: Workspace setup not found at $WORKSPACE_SETUP. Did you run colcon build?" >&2
    exit 1
fi

# Run the launch command with persistent camera device IDs to prevent swapping
exec ros2 launch ryugu_control ryugu_GoProduction_QGC.launch.py \
    front_dev:=/dev/v4l/by-id/usb-Xiongmai_web_camera_12345678-video-index0 \
    bottom_dev:=/dev/v4l/by-id/usb-JETE-W7_JETE-W7_202503051344-video-index0
