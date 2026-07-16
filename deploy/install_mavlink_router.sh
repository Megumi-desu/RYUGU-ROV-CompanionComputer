#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────────────────
# install_mavlink_router.sh — Build & deploy mavlink-router on Jetson Orin Nano
# ─────────────────────────────────────────────────────────────────────────────
#
# This script compiles mavlink-router from source (meson + ninja) on a Jetson
# Orin Nano running Ubuntu 22.04 / ROS2 Humble, installs the binary to
# /usr/local/bin, deploys the configuration file to /etc/mavlink-router/, and
# enables the systemd service for automatic start at boot.
#
# Usage (run on Jetson):
#   chmod +x deploy/install_mavlink_router.sh
#   sudo ./deploy/install_mavlink_router.sh
#
# After installation:
#   sudo systemctl start  mavlink-router
#   sudo systemctl status mavlink-router
#   journalctl -u mavlink-router -f   # follow logs
# ─────────────────────────────────────────────────────────────────────────────

set -euo pipefail

# ── Colour helpers ──────────────────────────────────────────────────────────
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'  # No Colour

info()  { echo -e "${CYAN}[INFO]${NC}  $*"; }
ok()    { echo -e "${GREEN}[OK]${NC}    $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
err()   { echo -e "${RED}[ERROR]${NC} $*"; }

# ── Must be root ────────────────────────────────────────────────────────────
if [[ $EUID -ne 0 ]]; then
    err "This script must be run as root (sudo)."
    exit 1
fi

# ── Paths ───────────────────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
MAVLINK_ROUTER_REPO="https://github.com/mavlink-router/mavlink-router.git"
BUILD_DIR="/tmp/mavlink-router-build"
INSTALL_PREFIX="/usr/local"
CONF_SRC="${REPO_ROOT}/deploy/mavlink-router/main.conf"
CONF_DST="/etc/mavlink-router/main.conf"
SERVICE_SRC="${REPO_ROOT}/deploy/mavlink-router/mavlink-router.service"
SERVICE_DST="/etc/systemd/system/mavlink-router.service"

info "RYUGU ROV — mavlink-router installer"
info "====================================="

# ── Step 1: Install build dependencies ──────────────────────────────────────
info "Step 1/6: Installing build dependencies..."
apt-get update -qq
apt-get install -y -qq \
    git \
    meson \
    ninja-build \
    pkg-config \
    gcc \
    g++ \
    python3-pip

# mavlink-router requires pymavlink at build time for dialect generation
pip3 install --quiet pymavlink

ok "Build dependencies installed."

# ── Step 2: Clone mavlink-router ────────────────────────────────────────────
info "Step 2/6: Cloning mavlink-router from GitHub..."
if [[ -d "$BUILD_DIR" ]]; then
    warn "Build directory $BUILD_DIR already exists — removing."
    rm -rf "$BUILD_DIR"
fi
git clone --recurse-submodules --depth 1 "$MAVLINK_ROUTER_REPO" "$BUILD_DIR"
ok "mavlink-router cloned to $BUILD_DIR."

# ── Step 3: Build with meson ────────────────────────────────────────────────
info "Step 3/6: Configuring build with meson..."
cd "$BUILD_DIR"
meson setup build \
    --prefix="$INSTALL_PREFIX" \
    --buildtype=release \
    -Dsystemdsystemunitdir=/etc/systemd/system
ok "Meson configuration complete."

info "Step 4/6: Compiling with ninja..."
ninja -C build
ok "Compilation complete."

# ── Step 4: Install ─────────────────────────────────────────────────────────
info "Step 5/6: Installing mavlink-router to $INSTALL_PREFIX..."
ninja -C build install
ldconfig  # refresh shared library cache
ok "mavlink-router installed."

# ── Step 5: Deploy configuration and systemd unit ────────────────────────────
info "Step 6/6: Deploying configuration and systemd service..."

# Config file
mkdir -p "$(dirname "$CONF_DST")"
if [[ -f "$CONF_SRC" ]]; then
    cp "$CONF_SRC" "$CONF_DST"
    ok "Config deployed → $CONF_DST"
else
    err "Config source not found: $CONF_SRC"
    exit 1
fi

# systemd unit
if [[ -f "$SERVICE_SRC" ]]; then
    cp "$SERVICE_SRC" "$SERVICE_DST"
    ok "systemd unit deployed → $SERVICE_DST"
else
    err "systemd unit source not found: $SERVICE_SRC"
    exit 1
fi

systemctl daemon-reload
ok "systemd daemon reloaded."

# Enable the service (but don't start yet — user should verify first)
systemctl enable mavlink-router
ok "mavlink-router enabled for auto-start at boot."

# ── Done ────────────────────────────────────────────────────────────────────
echo ""
info "══════════════════════════════════════════════════════════════════"
info "  Installation complete!"
info ""
info "  Config:       $CONF_DST"
info "  Service:      $SERVICE_DST"
info "  Binary:       $INSTALL_PREFIX/bin/mavlink-routerd"
info ""
info "  Start now:"
info "    sudo systemctl start mavlink-router"
info ""
info "  Check status:"
info "    sudo systemctl status mavlink-router"
info "    journalctl -u mavlink-router -f"
info ""
info "  Verify UDP ports are open:"
info "    ss -tuln | grep -E '14550|14555'"
info ""
info "  Then launch the ROS2 stack:"
info "    ros2 launch ryugu_control ryugu_GoProduction_QGC.launch.py"
info "══════════════════════════════════════════════════════════════════"
