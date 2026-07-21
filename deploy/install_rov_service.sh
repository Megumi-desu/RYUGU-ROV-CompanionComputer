#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────────────────
# install_rov_service.sh — Deploy & enable RYUGU ROV systemd service
# ─────────────────────────────────────────────────────────────────────────────
# Usage:
#   chmod +x deploy/install_rov_service.sh
#   sudo ./deploy/install_rov_service.sh
# ─────────────────────────────────────────────────────────────────────────────

set -euo pipefail

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

info()  { echo -e "${CYAN}[INFO]${NC}  $*"; }
ok()    { echo -e "${GREEN}[OK]${NC}    $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
err()   { echo -e "${RED}[ERROR]${NC} $*"; }

if [[ $EUID -ne 0 ]]; then
    err "This script must be run as root (sudo)."
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"

SERVICE_SRC="${REPO_ROOT}/deploy/ryugu-rov.service"
SERVICE_DST="/etc/systemd/system/ryugu-rov.service"
START_SCRIPT="${REPO_ROOT}/deploy/start_rov.sh"

info "RYUGU ROV — systemd Service Installer"
info "========================================"

# Make launcher script executable
if [[ -f "$START_SCRIPT" ]]; then
    chmod +x "$START_SCRIPT"
    chown icad:icad "$START_SCRIPT"
    ok "Made startup script executable: $START_SCRIPT"
else
    err "Startup script not found: $START_SCRIPT"
    exit 1
fi

# Deploy systemd unit file
if [[ -f "$SERVICE_SRC" ]]; then
    cp "$SERVICE_SRC" "$SERVICE_DST"
    chmod 644 "$SERVICE_DST"
    ok "systemd unit deployed → $SERVICE_DST"
else
    err "systemd service source not found: $SERVICE_SRC"
    exit 1
fi

# Reload systemd configuration
systemctl daemon-reload
ok "systemd daemon reloaded."

# Enable the service
systemctl enable ryugu-rov
ok "ryugu-rov service enabled for auto-start at boot."

echo ""
info "══════════════════════════════════════════════════════════════════"
info "  RYUGU ROV service configuration complete!"
info ""
info "  Start now:"
info "    sudo systemctl start ryugu-rov"
info ""
info "  Stop service:"
info "    sudo systemctl stop ryugu-rov"
info ""
info "  Check status:"
info "    sudo systemctl status ryugu-rov"
info "    journalctl -u ryugu-rov -f"
info "══════════════════════════════════════════════════════════════════"
