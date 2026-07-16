#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────────────────
# verify_mavlink_router.sh — End-to-end sanity checks for the telemetry chain
# ─────────────────────────────────────────────────────────────────────────────
#
# Run this on the Jetson after deploying mavlink-router and the ROS2 stack.
# It verifies each link in the chain so you can isolate failures quickly.
#
# Usage:
#   chmod +x scripts/verify_mavlink_router.sh
#   ./scripts/verify_mavlink_router.sh
# ─────────────────────────────────────────────────────────────────────────────

set -euo pipefail

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

PASS=0
FAIL=0

check() {
    local desc="$1"
    shift
    echo -ne "${CYAN}[CHECK]${NC} $desc ... "
    if "$@" &>/dev/null; then
        echo -e "${GREEN}PASS${NC}"
        ((PASS++)) || true
    else
        echo -e "${RED}FAIL${NC}"
        ((FAIL++)) || true
        echo -e "        ${YELLOW}→ Run manually: $*${NC}"
    fi
}

echo ""
echo "══════════════════════════════════════════════════════════════════"
echo "  RYUGU ROV — mavlink-router Verification Suite"
echo "══════════════════════════════════════════════════════════════════"
echo ""

# ── 1. mavlink-router binary ───────────────────────────────────────────────
echo "── 1. mavlink-router Installation ────────────────────────────────"
check "mavlink-routerd binary exists"         which mavlink-routerd
check "/etc/mavlink-router/main.conf exists"  test -f /etc/mavlink-router/main.conf

# ── 2. systemd service ─────────────────────────────────────────────────────
echo ""
echo "── 2. systemd Service ────────────────────────────────────────────"
check "systemd unit file exists"              test -f /etc/systemd/system/mavlink-router.service
check "mavlink-router service is enabled"     systemctl is-enabled mavlink-router

echo ""
echo "    Service status:"
systemctl is-active mavlink-router &>/dev/null && \
    echo -e "    ${GREEN}●${NC} mavlink-router is ACTIVE (running)" || \
    echo -e "    ${RED}●${NC} mavlink-router is NOT running — start it with:"
systemctl is-active mavlink-router &>/dev/null || \
    echo "      sudo systemctl start mavlink-router"

# ── 3. UDP ports ───────────────────────────────────────────────────────────
echo ""
echo "── 3. UDP Endpoints ─────────────────────────────────────────────"
check "UDP 127.0.0.1:14555 is open (MAVROS)"  ss -tuln | grep -q '127.0.0.1:14555'
check "UDP *:14550 is open (QGC outbound)"    ss -tuln | grep -q ':14550'

# ── 4. Serial port ─────────────────────────────────────────────────────────
echo ""
echo "── 4. Pixhawk Serial Link ───────────────────────────────────────"
if test -e /dev/ttyACM0; then
    echo -e "    ${GREEN}●${NC} /dev/ttyACM0 is present"
else
    echo -e "    ${RED}●${NC} /dev/ttyACM0 is MISSING — is the Pixhawk connected?"
fi

# ── 5. MAVROS heartbeat check ──────────────────────────────────────────────
echo ""
echo "── 5. MAVROS Heartbeat (run while ROS2 stack is active) ──────────"
if command -v ros2 &>/dev/null; then
    # Check if mavros node is running
    if ros2 node list 2>/dev/null | grep -q '/mavros/mavros_node'; then
        echo -e "    ${GREEN}●${NC} MAVROS node is running"
    else
        echo -e "    ${YELLOW}●${NC} MAVROS node not detected — is the ROS2 stack launched?"
        echo "      ros2 launch ryugu_control ryugu_GoProduction_QGC.launch.py"
    fi

    # Try to read one IMU message (2-second timeout)
    echo ""
    echo "    Checking IMU data on /mavros/imu/data (2 s timeout)..."
    if timeout 3 ros2 topic echo /mavros/imu/data --once 2>/dev/null; then
        echo -e "    ${GREEN}●${NC} IMU telemetry flowing from Pixhawk → mavlink-router → MAVROS"
    else
        echo -e "    ${YELLOW}●${NC} Could not read IMU data — check:"
        echo "      - Is mavlink-router running?  (systemctl status mavlink-router)"
        echo "      - Is the Pixhawk powered and armed?"
        echo "      - Is MAVROS connected?  (ros2 topic echo /mavros/state)"
    fi
else
    echo -e "    ${YELLOW}●${NC} ros2 CLI not found — source ROS2 setup.bash first"
fi

# ── 6. QGC connectivity (informational) ────────────────────────────────────
echo ""
echo "── 6. QGroundControl Connectivity ───────────────────────────────"
echo "    On the GCS laptop (192.168.1.100):"
echo "      1. Open QGroundControl"
echo "      2. Click the QGC icon → Application Settings → Comm Links"
echo "      3. Verify a UDP link on port 14550 shows a green heartbeat"
echo "      4. Check the 'Vehicle' icon in the top bar — should show"
echo "         'ArduSub v4.5.7' with battery, mode, and GPS status"
echo "      5. Go to Sensors → Calibrate Sensors to begin IMU/compass cal"

# ── 7. Quick-fix helpers ───────────────────────────────────────────────────
echo ""
echo "── 7. Common Issues & Quick Fixes ───────────────────────────────"
echo ""
echo "    Serial port busy:"
echo "      sudo lsof /dev/ttyACM0         # find the process hogging the port"
echo "      sudo fuser -k /dev/ttyACM0     # kill it"
echo ""
echo "    MAVROS can't connect:"
echo "      ss -tuln | grep 14555          # is the port open?"
echo "      sudo systemctl restart mavlink-router"
echo ""
echo "    QGC can't find Pixhawk:"
echo "      # On GCS laptop, verify connectivity:"
echo "      nc -u 192.168.1.10 14550       # should not error"
echo "      # Check GCS firewall:"
echo "      sudo ufw status"
echo ""

# ── Summary ────────────────────────────────────────────────────────────────
echo "══════════════════════════════════════════════════════════════════"
echo -e "  Results: ${GREEN}${PASS} passed${NC}, ${RED}${FAIL} failed${NC}"
echo "══════════════════════════════════════════════════════════════════"
