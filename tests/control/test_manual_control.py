#!/usr/bin/env python3
"""
test_manual_control.py — Standalone MANUAL_CONTROL MAVLink Test (Phase 1)

Purpose
-------
Prove the ArduSub manual-control chain end-to-end BEFORE the control path is
(re)integrated into the ryugu_control ROS 2 package.  This script has NO ROS 2
dependencies — pymavlink only — and reproduces the exact MAVLink traffic that
QGroundControl's joystick path generates:

    • 1 Hz  HEARTBEAT with MAV_TYPE_GCS               (GCS identity → arm permission)
    • 25 Hz MAVLINK_MSG_ID_MANUAL_CONTROL             (surge x, sway y, heave z, yaw r)
    • SET_MODE + MAV_CMD_COMPONENT_ARM_DISARM         (arming / mode sequence)

Link (default — matches the production architecture):
    This script ──UDP :14555──▶ mavlink-router ──/dev/ttyACM0──▶ Pixhawk

    IMPORTANT: while the ROS 2 stack is running, MAVROS owns 127.0.0.1:14555
    (listen mode) and consumes the FCU stream — a second client there would
    land in MAVROS's FCU-link socket and confuse its peer tracking.  Stop the
    ROS stack (or at least MAVROS) before testing through :14555.
    Use --device /dev/ttyACM0 for direct serial instead — mavlink-router must
    be STOPPED first, because its systemd service owns the port.

GCS identity
------------
ArduPilot gates arm commands (and, on several ArduSub versions, MANUAL_CONTROL
acceptance) on the sender being "my GCS" (SYSID_MYGCS, default 255).  This
script therefore mirrors QGroundControl: sysid=255, compid=190
(MAV_COMP_ID_MISSIONPLANNER), MAV_TYPE_GCS heartbeat.

SAFETY FIRST
------------
    • Bench test #1 MUST run with ESC power DISCONNECTED — verify that
      SERVO_OUTPUT_RAW reacts to setpoints before anything can spin.
    • Close QGroundControl while this script runs.  Two MANUAL_CONTROL sources
      fight each other on the Pixhawk (last packet wins) — the same sysid
      collision hazard documented in gcs_bridge_node.py.
    • Do not run this alongside test_arming_mode / any other command sender.
    • If the script is killed hard (SIGKILL, SSH drop), the neutral stream
      stops and ArduSub's FS_GCS_ENABLE failsafe takes over (Sub: surface
      script ending disarmed).  Verify FS_GCS_ENABLE on the Pixhawk.

DSHOT note (BLHeli_S DSHOT600 ESCs)
-----------------------------------
With MOT_PWM_TYPE=6 (DSHOT600), SERVO_OUTPUT_RAW reports the PWM-equivalent
output in µs-scale 1000–2000 (neutral ≈ 1500), NOT raw DSHOT codes.  Values
below ~1500 = reverse thrust, above ~1500 = forward (bidirectional DSHOT 3D
flashed on BLHeli_S).  Disarmed = 0.  Raw DSHOT codes (0–2047, idle ≈ 1047)
exist only on the ESC link.  Verified on bench: horizontals rest ~1497–1503
at neutral; vertical pair rests ~1202 (symmetric bias, not an error).

Usage
-----
    python3 tests/control/test_manual_control.py                       # via mavlink-router
    python3 tests/control/test_manual_control.py --dry-run             # telemetry only, no control
    python3 tests/control/test_manual_control.py --scripted --auto-arm --max-pct 25
    python3 tests/control/test_manual_control.py --device /dev/ttyACM0  # router must be stopped

Keys (interactive)
------------------
    w/s   surge fwd/back        a/d   sway left/right
    r/f   heave up/down         q/e   yaw left/right
    k     all axes → neutral    m     set MANUAL mode
    SPACE arm / disarm          h     help
    Ctrl+C  quit (neutral burst + disarm)
"""

import argparse
import os
import select
import sys
import termios
import threading
import time
from collections import deque

from pymavlink import mavutil

# ═══════════════════════════════════════════════════════════════════════════
#  ANSI colours
# ═══════════════════════════════════════════════════════════════════════════
class C:
    RESET   = '\033[0m'
    BOLD    = '\033[1m'
    DIM     = '\033[2m'
    RED     = '\033[91m'
    GREEN   = '\033[92m'
    YELLOW  = '\033[93m'
    CYAN    = '\033[96m'

# ═══════════════════════════════════════════════════════════════════════════
#  MAVLink identity — mirrors QGroundControl so ArduSub grants GCS authority
# ═══════════════════════════════════════════════════════════════════════════
DEFAULT_SYSID  = 255
DEFAULT_COMPID = 190        # MAV_COMP_ID_MISSIONPLANNER
TARGET_SYSID   = 1          # Pixhawk default
TARGET_COMPID  = 1

# ── MANUAL_CONTROL ─────────────────────────────────────────────────────────
DEFAULT_RATE = 25.0         # Hz — matches QGC joystick rate
NEUTRAL      = (0, 0, 0, 500)   # x, y, z, r — z=500 is NEUTRAL (zero thrust).
                                 # ArduSub scales z bidirectionally: z→2*(z/1000)-1
                                 #   z=0   → throttle -1 (FULL DOWN)
                                 #   z=500 → throttle  0 (NEUTRAL hover)
                                 #   z=1000→ throttle +1 (FULL UP)
                                 # x, y, r are BIPOLAR [-1000..+1000]: 0=neutral.
NEUTRAL_Z    = 500               # Neutral hover (ArduSub bidirectional throttle midpoint)
MAX_INPUT    = 1000
LINK_TIMEOUT = 2.0          # s without FCU heartbeat → stop sending control

# ── Gripper servo constants (MAIN 1 = Servo channel 1) ─────────────────────
GRIPPER_SERVO_CHANNEL = 1
GRIPPER_PWM_OPEN      = 1300     # µs — full open
GRIPPER_PWM_CLOSE     = 2000     # µs — full close
GRIPPER_PWM_STOP      = 1500     # µs — stop / neutral
MAV_CMD_DO_SET_SERVO  = 183

# ── Joystick deadband helper ───────────────────────────────────────────────
DEFAULT_DEADBAND_PCT  = 5.0      # % deadband around neutral


def apply_deadband(value, neutral=0, deadband_pct=DEFAULT_DEADBAND_PCT, max_range=None):
    """
    Apply deadband around neutral with smooth linear rescaling.

    Args:
        value:        Raw input integer/float (e.g. -1000..+1000 or 0..1000)
        neutral:      Neutral point (0 for x/y/r, 500 for z)
        deadband_pct: Deadband as percentage of full range (e.g. 5.0)
        max_range:    Full-scale range magnitude from neutral (default: 500 for z, 1000 for x/y/r)

    Returns:
        Adjusted value (int) with deadband applied and smoothly rescaled.
    """
    if max_range is None:
        max_range = 500.0 if neutral == 500 else 1000.0
    deadband = max_range * (deadband_pct / 100.0)
    offset = value - neutral
    if abs(offset) <= deadband:
        return int(neutral)
    sign = 1 if offset > 0 else -1
    rescaled = (abs(offset) - deadband) * max_range / (max_range - deadband)
    return int(neutral + sign * min(rescaled, max_range))


# ── ArduSub mode numbers (custom_mode).  Mode 1 is ACRO internally;
#    QGC labels it STABILIZE — same mapping as gcs_bridge_node.py MODE_MAP. ──
MODE_MANUAL = 0
MODE_MAP = {
    0: 'MANUAL', 1: 'STABILIZE', 2: 'ALT_HOLD', 3: 'AUTO',
    4: 'GUIDED', 7: 'CIRCLE', 9: 'SURFACE', 16: 'POSHOLD', 17: 'MOTOR_DETECT',
}

# ── Thruster output channels: AUX 1–6 = RC channels 9–14 (indices 8–13) ──
#    Mirrors THRUSTER_START_IDX / THRUSTER_COUNT in gcs_bridge_node.py
THRUSTER_START_IDX = 8
THRUSTER_COUNT     = 6

ACK_RESULT_NAMES = {
    0: 'ACCEPTED', 1: 'TEMPORARILY_REJECTED', 2: 'DENIED',
    3: 'UNSUPPORTED', 4: 'FAILED', 5: 'IN_PROGRESS', 6: 'CANCELLED',
}

SEVERITY_NAMES = {
    0: 'EMERGENCY', 1: 'ALERT', 2: 'CRITICAL', 3: 'ERROR',
    4: 'WARNING', 5: 'NOTICE', 6: 'INFO', 7: 'DEBUG',
}

# ═══════════════════════════════════════════════════════════════════════════
#  ManualControlTester
# ═══════════════════════════════════════════════════════════════════════════
class ManualControlTester:
    """
    Standalone MANUAL_CONTROL transmitter + arming test harness.

    Threads:
      - heartbeat thread     (1 Hz)   — GCS identity / arm permission
      - manual-control thread (--rate) — always transmits; neutral unless a
        setpoint is actively held; stops if the FCU link is lost
    Main loop:
      - MAVLink receiver (recv_match), keyboard UI, arm/disarm flow state
        machine, scripted-sequence driver, 4 Hz status line
    """

    KEY_ACTIONS = {
        'w': (0, +1), 's': (0, -1),      # surge  (x)
        'a': (1, -1), 'd': (1, +1),      # sway   (y)
        'r': (2, +1), 'f': (2, -1),      # heave  (z)
        'q': (3, -1), 'e': (3, +1),      # yaw    (r)
    }

    def __init__(self, conn, args):
        self.conn = conn
        self.rate = args.rate
        self.dry_run = args.dry_run
        self.auto_arm = args.auto_arm
        self.scripted = args.scripted
        self.target_sysid = args.target_sysid
        self.step = args.step
        self.amp = getattr(args, 'amp', 1000)
        self.deadband_pct = getattr(args, 'deadband_pct', DEFAULT_DEADBAND_PCT)
        self.servo_chan = getattr(args, 'servo_chan', GRIPPER_SERVO_CHANNEL)
        self.servo_step = getattr(args, 'servo_step', 50)

        self._running = threading.Event()
        self._print_lock = threading.Lock()
        self._state_lock = threading.Lock()

        # ── Link / vehicle state (written by main loop, read by threads) ──
        self._last_fcu_hb = 0.0
        self._armed = False
        self._custom_mode = -1
        self._batt_v = 0.0
        self._servo_raw = [0] * 16

        # ── Gripper / Servo state ──────────────────────────────────────
        self._current_gripper_pwm = GRIPPER_PWM_STOP
        self._servo_sweep_active = False
        self._servo_sweep_dir = 1
        self._last_servo_sweep_t = 0.0

        # ── Setpoints: [x, y, z, r] ────────────────────────────────────
        self._setpoints = list(NEUTRAL)

        # ── Counters ───────────────────────────────────────────────────
        self._rx_count = 0
        self._mc_sent = 0
        self._started_at = time.monotonic()

        # ── STATUSTEXT log (PreArm diagnostics) ────────────────────────
        self._statustext_log = deque(maxlen=12)

        # ── Arm/disarm flow state machine ──────────────────────────────
        self._flow = {'state': 'IDLE', 'deadline': 0.0, 'ack_result': None}

        # ── Scripted sequence state ────────────────────────────────────
        self._script = None
        self._exit_code = 0
        self._quit = False

        # ── Terminal raw mode ──────────────────────────────────────────
        self._termios_old = None
        self._tty = sys.stdin.isatty() and not self.scripted

        # ── Threads ────────────────────────────────────────────────────
        self._hb_thread = None
        self._mc_thread = None

    # ═══════════════════════════════════════════════════════════════════
    #  Threads
    # ═══════════════════════════════════════════════════════════════════
    def start(self):
        """Start heartbeat thread (always) and MANUAL_CONTROL thread (unless dry-run)."""
        self._running.set()
        self._hb_thread = threading.Thread(
            target=self._heartbeat_loop, name='hb', daemon=True)
        self._hb_thread.start()
        if not self.dry_run:
            self._mc_thread = threading.Thread(
                target=self._manual_control_loop, name='mc', daemon=True)
            self._mc_thread.start()

    def _heartbeat_loop(self):
        """1 Hz GCS heartbeat — establishes identity so ArduSub grants arm/control authority."""
        m = self.conn.mav
        while self._running.is_set():
            m.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            self._running.wait(1.0)

    def _manual_control_loop(self):
        """
        Rate-limited MANUAL_CONTROL sender.

        Always transmits the current setpoints (neutral unless actively held),
        filtered with apply_deadband and paced with time.monotonic(). If the FCU
        link is lost, transmission stops entirely so ArduSub failsafe takes over.
        """
        interval = 1.0 / self.rate
        next_t = time.monotonic()
        warned = False
        while self._running.is_set():
            with self._state_lock:
                rx, ry, rz, rr = self._setpoints
                link_ok = (time.monotonic() - self._last_fcu_hb) <= LINK_TIMEOUT

            # Apply deadband filter to setpoints before sending
            db = getattr(self, 'deadband_pct', DEFAULT_DEADBAND_PCT)
            x = apply_deadband(rx, neutral=0, deadband_pct=db)
            y = apply_deadband(ry, neutral=0, deadband_pct=db)
            z = apply_deadband(rz, neutral=NEUTRAL_Z, deadband_pct=db)
            r = apply_deadband(rr, neutral=0, deadband_pct=db)

            if not link_ok:
                if not warned:
                    self._event(
                        f'{C.RED}FCU link lost — control halted; '
                        f'ArduSub failsafe takes over.{C.RESET}')
                    warned = True
            else:
                warned = False
                self.conn.mav.manual_control_send(self.target_sysid, x, y, z, r, 0)
                with self._state_lock:
                    self._mc_sent += 1
            next_t += interval
            delay = next_t - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_t = time.monotonic()   # fell behind — resync the pacer

    def send_gripper_pwm(self, pwm_us, label=""):
        """Send MAV_CMD_DO_SET_SERVO (command 183) to control gripper/servo."""
        pwm_us = max(1000, min(2000, int(pwm_us)))
        with self._state_lock:
            self._current_gripper_pwm = pwm_us
        if self.dry_run:
            self._event(f'{C.YELLOW}DRY-RUN: gripper command suppressed ({label} {pwm_us}µs).{C.RESET}')
            return
        chan = self.servo_chan
        self._event(f'{C.CYAN}Gripper {label} → PWM={pwm_us}µs (Servo {chan}){C.RESET}')
        self.conn.mav.command_long_send(
            self.target_sysid, TARGET_COMPID,
            MAV_CMD_DO_SET_SERVO, 0,
            float(chan), float(pwm_us),
            0, 0, 0, 0, 0)

    def _tick_servo_sweep(self, now):
        """Automatically sweep gripper servo between 1300µs and 2000µs when enabled."""
        if not self._servo_sweep_active:
            return
        if now - self._last_servo_sweep_t < 0.15:
            return
        self._last_servo_sweep_t = now
        curr = self._current_gripper_pwm
        step = 50 * self._servo_sweep_dir
        nxt = curr + step
        if nxt >= GRIPPER_PWM_CLOSE:
            nxt = GRIPPER_PWM_CLOSE
            self._servo_sweep_dir = -1
        elif nxt <= GRIPPER_PWM_OPEN:
            nxt = GRIPPER_PWM_OPEN
            self._servo_sweep_dir = 1
        self.send_gripper_pwm(nxt, label=f"SWEEP {'▲' if self._servo_sweep_dir > 0 else '▼'}")

    # ═══════════════════════════════════════════════════════════════════
    #  Link bring-up
    # ═══════════════════════════════════════════════════════════════════
    def wait_for_fcu(self, timeout):
        """
        Block until the Pixhawk HEARTBEAT is seen (or timeout).

        MAVROS (if running) also sends heartbeats with sysid 1, so we require
        the autopilot field to be MAV_AUTOPILOT_ARDUPILOTMEGA — only the real
        FCU reports that.
        """
        deadline = time.monotonic() + timeout
        last_dot = 0.0
        while time.monotonic() < deadline:
            msg = self.conn.recv_match(blocking=True, timeout=0.5)
            if msg:
                with self._state_lock:
                    self._rx_count += 1
                if self._is_fcu_heartbeat(msg):
                    self._handle_heartbeat(msg)
                    self._event(
                        f'{C.GREEN}FCU detected — ArduSub HEARTBEAT received '
                        f'(mode={MODE_MAP.get(self._custom_mode, "?")}, '
                        f'{"ARMED" if self._armed else "DISARMED"}){C.RESET}')
                    return True
            if time.monotonic() - last_dot > 2.0:
                self._event(f'{C.DIM}waiting for Pixhawk HEARTBEAT ...{C.RESET}')
                last_dot = time.monotonic()
        return False

    def _is_fcu_heartbeat(self, msg):
        return (
            msg.get_type() == 'HEARTBEAT'
            and msg.get_srcSystem() == self.target_sysid
            and msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA
        )

    def request_streams(self):
        """Set message intervals on the FCU (STATUSTEXT is event-driven, not rate-limited)."""
        streams = (
            (mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 2),
            (mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, 20),
        )
        for msg_id, hz in streams:
            self.conn.mav.command_long_send(
                self.target_sysid, TARGET_COMPID,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                msg_id, int(1e6 / hz), 0, 0, 0, 0, 0)
        self._event(
            f'{C.DIM}requested streams: SYS_STATUS 2 Hz, '
            f'SERVO_OUTPUT_RAW 20 Hz{C.RESET}')

    # ═══════════════════════════════════════════════════════════════════
    #  Mode / arm / disarm commands
    # ═══════════════════════════════════════════════════════════════════
    def _send_set_mode(self, custom_mode):
        name = MODE_MAP.get(custom_mode, f'?{custom_mode}')
        self._event(f'{C.CYAN}SET_MODE → {name} (custom_mode={custom_mode}){C.RESET}')
        self.conn.mav.set_mode_send(
            self.target_sysid,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            custom_mode)

    def request_arm(self, confirm=True):
        """Start the arm flow: MANUAL mode → ARM command → wait for armed bit."""
        if self.dry_run:
            self._event(f'{C.YELLOW}DRY-RUN: arming disabled by --dry-run.{C.RESET}')
            return
        if self._flow['state'] != 'IDLE':
            self._event(
                f'{C.YELLOW}Arming flow busy ({self._flow["state"]}) — wait.{C.RESET}')
            return
        if confirm and not self._confirm('ARM the vehicle? [y/N] ', timeout=10.0):
            self._event('Arm cancelled.')
            return
        self._set_neutral()
        self._event(f'{C.GREEN}Setpoints reset to NEUTRAL (0, 0, 500, 0) before arming.{C.RESET}')
        if self._custom_mode != MODE_MANUAL:
            self._event('Setting MANUAL mode first '
                        '(MANUAL_CONTROL only drives motors in MANUAL) ...')
            self._send_set_mode(MODE_MANUAL)
            self._flow = {
                'state': 'WAIT_MODE',
                'deadline': time.monotonic() + 5.0,
                'ack_result': None,
            }
        else:
            self._send_arm_cmd()

    def _send_arm_cmd(self):
        self._event(
            f'{C.YELLOW}Sending ARM command '
            f'(MAV_CMD_COMPONENT_ARM_DISARM=1) ...{C.RESET}')
        self.conn.mav.command_long_send(
            self.target_sysid, TARGET_COMPID,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1, 0, 0, 0, 0, 0, 0)
        self._flow = {
            'state': 'WAIT_ACK_ARM',
            'deadline': time.monotonic() + 5.0,
            'ack_result': None,
        }

    def request_disarm(self):
        """Start the disarm flow: DISARM command → wait for disarmed bit."""
        if self._flow['state'] != 'IDLE':
            self._event(
                f'{C.YELLOW}Command flow busy ({self._flow["state"]}) — wait.{C.RESET}')
            return
        self._event(f'{C.YELLOW}Sending DISARM command ...{C.RESET}')
        self.conn.mav.command_long_send(
            self.target_sysid, TARGET_COMPID,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            0, 0, 0, 0, 0, 0, 0)
        self._flow = {
            'state': 'WAIT_ACK_DISARM',
            'deadline': time.monotonic() + 5.0,
            'ack_result': None,
        }

    # ═══════════════════════════════════════════════════════════════════
    #  MAVLink message handling
    # ═══════════════════════════════════════════════════════════════════
    def _handle_msg(self, msg):
        with self._state_lock:
            self._rx_count += 1
        mtype = msg.get_type()
        if mtype == 'HEARTBEAT' and self._is_fcu_heartbeat(msg):
            self._handle_heartbeat(msg)
        elif mtype == 'STATUSTEXT':
            self._handle_statustext(msg)
        elif mtype == 'SYS_STATUS':
            with self._state_lock:
                self._batt_v = msg.voltage_battery / 1000.0
        elif mtype == 'SERVO_OUTPUT_RAW':
            vals = [getattr(msg, f'servo{i}_raw', 0) for i in range(1, 17)]
            with self._state_lock:
                self._servo_raw = vals
        elif mtype == 'COMMAND_ACK':
            self._handle_ack(msg)

    def _handle_heartbeat(self, msg):
        with self._state_lock:
            self._last_fcu_hb = time.monotonic()
            self._armed = bool(
                msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            self._custom_mode = msg.custom_mode

    def _handle_statustext(self, msg):
        sev = msg.severity
        text = msg.text.rstrip('\x00').strip()
        sev_name = SEVERITY_NAMES.get(sev, f'?{sev}')
        colour = C.RED if sev <= 2 else (C.YELLOW if sev <= 4 else C.DIM)
        self._event(f'{colour}[STATUSTEXT {sev_name}] {text}{C.RESET}')
        if sev <= 2 or 'PreArm' in text:
            self._statustext_log.append((time.monotonic(), sev_name, text))

    def _handle_ack(self, msg):
        if msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            res = ACK_RESULT_NAMES.get(msg.result, f'UNKNOWN({msg.result})')
            self._event(f'{C.DIM}COMMAND_ACK: ARM/DISARM → {res}{C.RESET}')
            self._flow['ack_result'] = msg.result
        elif msg.command == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL:
            pass  # silent — stream setup ACK
        else:
            res = ACK_RESULT_NAMES.get(msg.result, f'UNKNOWN({msg.result})')
            self._event(f'{C.DIM}COMMAND_ACK: #{msg.command} → {res}{C.RESET}')

    # ═══════════════════════════════════════════════════════════════════
    #  Arm/disarm flow state machine (ticked from the main loop)
    # ═══════════════════════════════════════════════════════════════════
    def _tick_flow(self, now):
        st = self._flow['state']
        if st == 'IDLE':
            return
        if st == 'WAIT_MODE':
            if self._custom_mode == MODE_MANUAL:
                self._send_arm_cmd()
            elif now > self._flow['deadline']:
                mode = MODE_MAP.get(self._custom_mode, f'?{self._custom_mode}')
                self._flow['state'] = 'IDLE'
                self._arm_failed(f'mode change not confirmed (still {mode})')
        elif st == 'WAIT_ACK_ARM':
            r = self._flow['ack_result']
            if r is not None:
                if r in (mavutil.mavlink.MAV_RESULT_ACCEPTED,
                         mavutil.mavlink.MAV_RESULT_IN_PROGRESS):
                    self._flow = {
                        'state': 'WAIT_ARMED',
                        'deadline': now + 10.0,
                        'ack_result': None,
                    }
                    self._event(
                        f'{C.YELLOW}Arm command accepted — '
                        f'waiting for ARMED state ...{C.RESET}')
                else:
                    self._flow['state'] = 'IDLE'
                    self._arm_failed(ACK_RESULT_NAMES.get(r, f'result {r}'))
            elif now > self._flow['deadline']:
                self._flow['state'] = 'IDLE'
                self._arm_failed('no COMMAND_ACK received')
        elif st == 'WAIT_ARMED':
            if self._armed:
                self._flow['state'] = 'IDLE'
                self._event(
                    f'{C.GREEN}{C.BOLD}✓ ARMED — AUX outputs now live '
                    f'(verify SERVO_OUTPUT_RAW before connecting ESCs){C.RESET}')
            elif now > self._flow['deadline']:
                self._flow['state'] = 'IDLE'
                self._arm_failed('armed bit not set in HEARTBEAT')
        elif st == 'WAIT_ACK_DISARM':
            if self._flow['ack_result'] is not None:
                self._flow = {
                    'state': 'WAIT_DISARMED',
                    'deadline': now + 5.0,
                    'ack_result': None,
                }
            elif now > self._flow['deadline']:
                self._flow['state'] = 'IDLE'
                self._event(
                    f'{C.YELLOW}No COMMAND_ACK for DISARM — '
                    f'check vehicle state!{C.RESET}')
        elif st == 'WAIT_DISARMED':
            if not self._armed:
                self._flow['state'] = 'IDLE'
                self._event(f'{C.GREEN}✓ DISARMED{C.RESET}')
            elif now > self._flow['deadline']:
                self._flow['state'] = 'IDLE'
                self._event(
                    f'{C.RED}Disarm NOT confirmed — check vehicle state!{C.RESET}')

    def _arm_failed(self, reason):
        self._event(f'{C.RED}{C.BOLD}✗ ARM FAILED — {reason}{C.RESET}')
        prearm = [e for e in self._statustext_log if 'PreArm' in e[2]]
        if prearm:
            self._event(f'{C.RED}Recent PreArm messages from ArduSub:{C.RESET}')
            for _, sev, text in prearm[-6:]:
                self._event(f'{C.DIM}  [{sev}] {text}{C.RESET}')
        else:
            self._event(
                f'{C.YELLOW}No PreArm STATUSTEXT received.  Check: safety switch '
                f'pressed, battery voltage, EKF/AHRS settled after boot (~30 s).{C.RESET}')
        if self._script is not None and self._script['phase'] == 'WAIT_ARM':
            self._event(f'{C.RED}Scripted run aborted (arm failed).{C.RESET}')
            self._exit_code = 1
            self._quit = True

    # ═══════════════════════════════════════════════════════════════════
    #  Setpoints (thread-safe, clamped)
    # ═══════════════════════════════════════════════════════════════════
    def _clamp_axis(self, idx, value):
        lo, hi = -self.amp, self.amp
        return int(max(lo, min(hi, value)))

    def _set_axis(self, idx, delta):
        with self._state_lock:
            self._setpoints[idx] = self._clamp_axis(
                idx, self._setpoints[idx] + delta)

    def _set_neutral(self):
        with self._state_lock:
            self._setpoints = list(NEUTRAL)

    def _set_setpoints(self, x, y, z, r):
        with self._state_lock:
            self._setpoints = [
                self._clamp_axis(0, x), self._clamp_axis(1, y),
                self._clamp_axis(2, z), self._clamp_axis(3, r)]

    # ═══════════════════════════════════════════════════════════════════
    #  Keyboard UI
    # ═══════════════════════════════════════════════════════════════════
    def _enable_raw_mode(self):
        if not self._tty:
            self._event(
                f'{C.YELLOW}stdin is not a TTY — keyboard control disabled '
                f'(telemetry + arm flow only).{C.RESET}')
            return
        fd = sys.stdin.fileno()
        self._termios_old = termios.tcgetattr(fd)
        new = termios.tcgetattr(fd)
        new[3] &= ~(termios.ICANON | termios.ECHO | termios.ISIG)
        new[6][termios.VMIN] = 0
        new[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, new)

    def _restore_terminal(self):
        if self._termios_old is not None:
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSANOW, self._termios_old)
            self._termios_old = None

    def _poll_keys(self):
        if not self._tty:
            return
        while select.select([sys.stdin], [], [], 0)[0]:
            data = os.read(sys.stdin.fileno(), 64)
            if not data:
                return
            for ch in data.decode('utf-8', errors='ignore'):
                self._on_key(ch)

    def _on_key(self, ch):
        if ch == '\x03':        # Ctrl+C (raw mode → arrives as a byte, not SIGINT)
            self._quit = True
            return
        key = ch.lower()
        if key in self.KEY_ACTIONS:
            idx, direction = self.KEY_ACTIONS[key]
            self._set_axis(idx, direction * self.step)
        elif ch == ' ':
            if self._armed:
                self.request_disarm()
            else:
                self.request_arm(confirm=not self.auto_arm)
        elif key == 'k':
            self._set_neutral()
            self._event(f'{C.GREEN}Setpoints → NEUTRAL (0, 0, 500, 0){C.RESET}')
        elif key == 'm':
            self._send_set_mode(MODE_MANUAL)
        elif key == 'o':
            self._servo_sweep_active = False
            self.send_gripper_pwm(GRIPPER_PWM_OPEN, label="OPEN")
        elif key == 'c':
            self._servo_sweep_active = False
            self.send_gripper_pwm(GRIPPER_PWM_CLOSE, label="CLOSE")
        elif key == 'v':
            self._servo_sweep_active = False
            self.send_gripper_pwm(GRIPPER_PWM_STOP, label="STOP")
        elif ch == '[':
            self._servo_sweep_active = False
            nxt = self._current_gripper_pwm - self.servo_step
            self.send_gripper_pwm(nxt, label=f"STEP -{self.servo_step}")
        elif ch == ']':
            self._servo_sweep_active = False
            nxt = self._current_gripper_pwm + self.servo_step
            self.send_gripper_pwm(nxt, label=f"STEP +{self.servo_step}")
        elif key == 'p':
            self._servo_sweep_active = not self._servo_sweep_active
            st = "ENABLED (1300<->2000µs)" if self._servo_sweep_active else "DISABLED"
            self._event(f'{C.CYAN}Gripper Servo Sweep Test: {st}{C.RESET}')
        elif key == 'h':
            self._print_help()

    def _confirm(self, prompt, timeout=10.0):
        """Yes/no prompt compatible with raw-mode stdin.  Returns True on 'y'."""
        sys.stdout.write('\r\x1b[K' + prompt)
        sys.stdout.flush()
        if not self._tty:
            self._event('(not a TTY — cannot confirm)')
            return False
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if select.select([sys.stdin], [], [], 0.2)[0]:
                ch = os.read(sys.stdin.fileno(), 1).decode('utf-8', errors='ignore')
                if ch.lower() == 'y':
                    sys.stdout.write('y\n')
                    sys.stdout.flush()
                    return True
                if ch in ('n', 'N', '\x1b', '\x03'):
                    sys.stdout.write('n\n')
                    sys.stdout.flush()
                    return False
        sys.stdout.write('(timeout)\n')
        sys.stdout.flush()
        return False

    def _print_help(self):
        for line in (
            f'{C.BOLD}Keys:{C.RESET}',
            f'  {C.CYAN}w/s{C.RESET} surge fwd/back     {C.CYAN}a/d{C.RESET} sway left/right',
            f'  {C.CYAN}r/f{C.RESET} heave up/down      {C.CYAN}q/e{C.RESET} yaw left/right',
            f'  {C.CYAN}k{C.RESET}   all → neutral       {C.CYAN}m{C.RESET}   set MANUAL mode',
            f'  {C.CYAN}o{C.RESET}   gripper OPEN (1300) {C.CYAN}c{C.RESET}   gripper CLOSE (2000)',
            f'  {C.CYAN}v{C.RESET}   gripper STOP (1500) {C.CYAN}p{C.RESET}   toggle SERVO SWEEP',
            f'  {C.CYAN}[{C.RESET}   servo step -{self.servo_step}µs     {C.CYAN}]{C.RESET}   servo step +{self.servo_step}µs',
            f'  {C.CYAN}SPACE{C.RESET} arm / disarm        {C.CYAN}h{C.RESET}   help   {C.CYAN}Ctrl+C{C.RESET} quit',
            f'{C.DIM}  step={self.step}  rate={self.rate:.0f} Hz  '
            f'amplitude=±{self.amp}  deadband={self.deadband_pct:.1f}%  '
            f'servo_chan=Servo {self.servo_chan}{C.RESET}',
        ):
            self._event(line)

    # ═══════════════════════════════════════════════════════════════════
    #  Scripted sequence (SSH-safe, no keyboard)
    # ═══════════════════════════════════════════════════════════════════
    def _build_script(self):
        amp = self.amp
        steps = (
            ('neutral hold', 2.0,   0,   0, 0, 0),
            ('surge +',      2.0, amp,   0, 0, 0),
            ('neutral hold', 1.5,   0,   0, 0, 0),
            ('sway +',       2.0,   0, amp, 0, 0),
            ('neutral hold', 1.5,   0,   0, 0, 0),
            ('yaw +',        2.0,   0,   0, 0, amp),
            ('neutral hold', 2.0,   0,   0, 0, 0),
        )
        self._script = {
            'steps': steps, 'idx': 0, 'step_start': 0.0,
            'phase': 'WAIT_ARM', 'started': time.monotonic(),
            'finish_started': 0.0,
        }

    def _tick_scripted(self, now):
        sc = self._script
        if sc is None:
            return
        if sc['phase'] == 'WAIT_ARM':
            if self._armed:
                sc['phase'] = 'RUN'
                sc['step_start'] = now
                self._event(f'{C.GREEN}Armed — running scripted sequence ...{C.RESET}')
            elif now - sc['started'] > 60.0:
                self._event(f'{C.RED}Scripted run aborted: arm did not complete within 60 s.{C.RESET}')
                self._exit_code = 1
                self._quit = True
            return
        if sc['phase'] == 'RUN':
            _label, dur, x, y, z, r = sc['steps'][sc['idx']]
            self._set_setpoints(x, y, z, r)
            if now - sc['step_start'] >= dur:
                sc['idx'] += 1
                if sc['idx'] >= len(sc['steps']):
                    sc['phase'] = 'FINISHING'
                    sc['finish_started'] = now
                    self._set_neutral()
                    self._event(f'{C.GREEN}Sequence complete — neutral, disarming ...{C.RESET}')
                    self.request_disarm()
                else:
                    sc['step_start'] = now
                    self._event(f'{C.DIM}step: {sc["steps"][sc["idx"]][0]}{C.RESET}')
            return
        if sc['phase'] == 'FINISHING':
            if not self._armed:
                self._event(f'{C.GREEN}✓ Scripted run complete — disarmed.{C.RESET}')
                self._quit = True
            elif now - sc['finish_started'] > 15.0:
                self._event(
                    f'{C.RED}Disarm not confirmed after 15 s — quitting; '
                    f'ArduSub FS_GCS failsafe takes over.{C.RESET}')
                self._exit_code = 1
                self._quit = True

    # ═══════════════════════════════════════════════════════════════════
    #  Terminal output
    # ═══════════════════════════════════════════════════════════════════
    def _event(self, text):
        """Print a full-line event, clearing the live status line first."""
        with self._print_lock:
            sys.stdout.write('\r\x1b[K' + text + '\n')
            sys.stdout.flush()

    def _statusline(self):
        """Live single-line HUD, rewritten at 4 Hz."""
        with self._state_lock:
            mode = MODE_MAP.get(self._custom_mode, f'?{self._custom_mode}')
            armed = self._armed
            batt = self._batt_v
            x, y, z, r = self._setpoints
            aux = self._servo_raw[THRUSTER_START_IDX:
                                  THRUSTER_START_IDX + THRUSTER_COUNT]
            main_idx = self.servo_chan - 1
            main_pwm = self._servo_raw[main_idx] if 0 <= main_idx < len(self._servo_raw) else 0
            tgt_pwm = self._current_gripper_pwm
            link_ok = (time.monotonic() - self._last_fcu_hb) <= LINK_TIMEOUT
            mc = self._mc_sent
        arm_s = f'{C.RED}{C.BOLD}ARMED{C.RESET}' if armed else f'{C.GREEN}DISARMED{C.RESET}'
        link_s = f'{C.GREEN}LINK{C.RESET}' if link_ok else f'{C.RED}NO LINK{C.RESET}'
        aux_s = ' '.join(f'{v:>4}' for v in aux)
        flow_s = ''
        if self._flow['state'] != 'IDLE':
            flow_s = f' {C.YELLOW}[{self._flow["state"]}]{C.RESET}'
        script_s = ''
        if self._script is not None and self._script['phase'] == 'RUN':
            script_s = f' {C.CYAN}[{self._script["steps"][self._script["idx"]][0]}]{C.RESET}'
        sweep_s = f' {C.YELLOW}[SWEEP]{C.RESET}' if self._servo_sweep_active else ''
        line = (f'{C.CYAN}[{mode}]{C.RESET} {arm_s} {link_s} '
                f'{batt:5.1f}V  '
                f'x={int(x):+05d} y={int(y):+05d} z={int(z):04d} r={int(r):+05d}  '
                f'MAIN{self.servo_chan}:{main_pwm:>4}µs  '
                f'AUX[1-6]: {aux_s}  MC:{mc}{flow_s}{script_s}{sweep_s}')
        with self._print_lock:
            sys.stdout.write('\r\x1b[K' + line)
            sys.stdout.flush()

    # ═══════════════════════════════════════════════════════════════════
    #  Main loop / lifecycle
    # ═══════════════════════════════════════════════════════════════════
    def begin(self):
        """Post-connect setup: scripted arm request or interactive keyboard UI."""
        if self.scripted:
            self._build_script()
            if self._armed:
                self._script['phase'] = 'RUN'
                self._script['step_start'] = time.monotonic()
                self._event(f'{C.GREEN}Already armed — starting sequence ...{C.RESET}')
            else:
                self._event(
                    f'{C.YELLOW}Requesting arm (--auto-arm) — ESC power should '
                    f'still be OFF for the first bench run!{C.RESET}')
                self.request_arm(confirm=False)
        else:
            self._enable_raw_mode()
            self._print_help()
            if self.dry_run:
                self._event(
                    f'{C.YELLOW}DRY-RUN: MANUAL_CONTROL disabled, '
                    f'arming disabled.{C.RESET}')
            else:
                self._event(
                    f'{C.DIM}MANUAL_CONTROL stream running at {self.rate:.0f} Hz '
                    f'(neutral).{C.RESET}')

    def run(self):
        """Main loop: receive MAVLink, tick flow/script/servo sweep, poll keys, draw HUD."""
        last_status = 0.0
        while not self._quit:
            msg = self.conn.recv_match(blocking=True, timeout=0.05)
            if msg:
                self._handle_msg(msg)
            now = time.monotonic()
            self._tick_flow(now)
            self._tick_scripted(now)
            self._tick_servo_sweep(now)
            self._poll_keys()
            if now - last_status >= 0.25:
                self._statusline()
                last_status = now
        return self._exit_code

    def shutdown(self):
        """Safe teardown: neutral burst (~1 s) → disarm if armed → close link."""
        was_armed = self._armed
        self._running.clear()
        for t in (self._hb_thread, self._mc_thread):
            if t is not None:
                t.join(timeout=2.0)

        # ── Neutral MANUAL_CONTROL burst (~1 s at current rate) ──────────
        if not self.dry_run:
            interval = 1.0 / self.rate
            for _ in range(int(self.rate)):
                self.conn.mav.manual_control_send(self.target_sysid, *NEUTRAL, 0)
                time.sleep(interval)

        # ── Disarm if still armed ─────────────────────────────────────────
        if was_armed and not self.dry_run:
            self._event(f'{C.YELLOW}Disarming before exit ...{C.RESET}')
            self.conn.mav.command_long_send(
                self.target_sysid, TARGET_COMPID,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                0, 0, 0, 0, 0, 0, 0)
            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline:
                msg = self.conn.recv_match(blocking=True, timeout=0.2)
                if msg:
                    self._handle_msg(msg)
                if not self._armed:
                    self._event(f'{C.GREEN}✓ DISARMED{C.RESET}')
                    break
            else:
                self._event(
                    f'{C.RED}✗ Disarm NOT confirmed before exit — '
                    f'ArduSub FS_GCS failsafe takes over.{C.RESET}')

        self.conn.close()

    def print_stats(self):
        dur = time.monotonic() - self._started_at
        with self._state_lock:
            rx, mc = self._rx_count, self._mc_sent
        self._event(
            f'{C.BOLD}Session stats:{C.RESET} {dur:.1f}s | '
            f'MAVLink rx: {rx} | MANUAL_CONTROL tx: {mc} | '
            f'final state: {"ARMED" if self._armed else "DISARMED"}')


# ═══════════════════════════════════════════════════════════════════════════
#  CLI / main
# ═══════════════════════════════════════════════════════════════════════════
def _parse_args():
    p = argparse.ArgumentParser(
        description='RYUGU ROV — standalone MANUAL_CONTROL MAVLink test '
                    '(pymavlink only, no ROS 2)',
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--endpoint', default='udpin:127.0.0.1:14555',
                   help='MAVLink endpoint via mavlink-router '
                        '(default: udpin:127.0.0.1:14555 — use udpin, NOT udpout; '
                        'mavlink-router Normal-mode UDP endpoints only reply to '
                        'clients that bind/listen, not connected sockets)')
    p.add_argument('--device', default=None,
                   help='Direct serial device instead of UDP (e.g. /dev/ttyACM0). '
                        'mavlink-router must be STOPPED first.')
    p.add_argument('--baud', type=int, default=115200,
                   help='Serial baud for --device (default: 115200)')
    p.add_argument('--sysid', type=int, default=DEFAULT_SYSID,
                   help='Our MAVLink system ID (default: 255 = QGC)')
    p.add_argument('--compid', type=int, default=DEFAULT_COMPID,
                   help='Our MAVLink component ID '
                        '(default: 190 = MAV_COMP_ID_MISSIONPLANNER)')
    p.add_argument('--target-sysid', type=int, default=TARGET_SYSID,
                   help='Pixhawk system ID (default: 1)')
    p.add_argument('--rate', type=float, default=DEFAULT_RATE,
                   help='MANUAL_CONTROL rate in Hz, 10–50 (default: 25)')
    p.add_argument('--step', type=int, default=100,
                   help='Key press increment per axis (default: 100)')
    p.add_argument('--max-pct', type=int, default=None,
                   help='Amplitude clamp in %% (1–100). '
                        'Default: 100 interactive, 25 scripted.')
    p.add_argument('--deadband-pct', type=float, default=DEFAULT_DEADBAND_PCT,
                   help='Joystick deadband in %% around neutral '
                        '(default: 5.0)')
    p.add_argument('--servo-chan', type=int, default=GRIPPER_SERVO_CHANNEL,
                   help='Servo channel for gripper testing (default: 1 = MAIN 1)')
    p.add_argument('--servo-step', type=int, default=50,
                   help='PWM increment step for [ and ] keys (default: 50)')
    p.add_argument('--dry-run', action='store_true',
                   help='Connect + telemetry only: never arm, '
                        'never send MANUAL_CONTROL.')
    p.add_argument('--auto-arm', action='store_true',
                   help='Skip the interactive arm confirmation '
                        '(required with --scripted).')
    p.add_argument('--scripted', action='store_true',
                   help='Run the timed setpoint sequence (SSH-safe, no keyboard).')
    p.add_argument('--wait', type=float, default=15.0,
                   help='Seconds to wait for the Pixhawk HEARTBEAT (default: 15)')
    args = p.parse_args()

    if args.dry_run and args.scripted:
        p.error('--dry-run and --scripted are mutually exclusive')
    if args.scripted and not args.auto_arm:
        p.error('--scripted requires --auto-arm (scripted mode is non-interactive)')

    args.rate = max(10.0, min(50.0, args.rate))
    args.deadband_pct = max(0.0, min(25.0, args.deadband_pct))
    if not 1 <= args.servo_chan <= 16:
        p.error('--servo-chan must be 1..16')
    if not 10 <= args.servo_step <= 500:
        p.error('--servo-step must be 10..500')
    if not 1 <= args.step <= 500:
        p.error('--step must be 1..500')

    if args.max_pct is None:
        default_pct = 25 if args.scripted else 100
        args.amp = int(MAX_INPUT * default_pct / 100)
    else:
        if not 1 <= args.max_pct <= 100:
            p.error('--max-pct must be 1..100')
        args.amp = int(MAX_INPUT * args.max_pct / 100)
    return args


def _print_banner(args):
    mode = 'SCRIPTED' if args.scripted else ('DRY-RUN' if args.dry_run else 'INTERACTIVE')
    print(f'{C.BOLD}{"═" * 64}{C.RESET}')
    print(f'{C.BOLD}{C.CYAN}      RYUGU ROV — Manual Control MAVLink Test (Phase 1){C.RESET}')
    print(f'{C.BOLD}{"═" * 64}{C.RESET}')
    print(f'  Link:      {C.GREEN}{args.device or args.endpoint}{C.RESET}')
    print(f'  Identity:  sysid={args.sysid} compid={args.compid} '
          f'(QGC-mirror — ArduSub arm authority)')
    print(f'  Rate:      {args.rate:.0f} Hz MANUAL_CONTROL  |  mode: {mode}')
    print(f'  Amplitude: ±{args.amp}  |  deadband: {args.deadband_pct:.1f}%  |  neutral = (0, 0, 500, 0)')
    print(f'  Gripper:   o=Open (1300µs) | c=Close (2000µs) | v=Stop (1500µs) on MAIN 1')
    print(f'{C.BOLD}{"─" * 64}{C.RESET}')
    print(f'  {C.RED}{C.BOLD}SAFETY:{C.RESET} bench test #1 with ESC power OFF — '
          f'verify SERVO_OUTPUT_RAW')
    print(f'  {C.RED}{C.BOLD}SAFETY:{C.RESET} close QGroundControl '
          f'(second MANUAL_CONTROL source = conflict)')
    if args.device:
        print(f'  {C.YELLOW}NOTE:  direct serial — stop mavlink-router first '
              f'(it owns the port){C.RESET}')
    print(f'  {C.DIM}DSHOT600: SERVO_OUTPUT_RAW shows DSHOT codes '
          f'(0–2047, neutral≈1047), not µs{C.RESET}')
    print(f'{C.BOLD}{"═" * 64}{C.RESET}')


def main():
    args = _parse_args()
    _print_banner(args)

    endpoint = args.device or args.endpoint
    try:
        conn = mavutil.mavlink_connection(
            endpoint, baud=args.baud, dialect='ardupilotmega',
            source_system=args.sysid, source_component=args.compid)
    except Exception as e:
        print(f'{C.RED}ERROR: cannot open MAVLink connection '
              f'"{endpoint}": {e}{C.RESET}')
        if args.device:
            print(f'  {C.YELLOW}- Is mavlink-router holding the port?  '
                  f'sudo systemctl stop mavlink-router{C.RESET}')
            print(f'  {C.YELLOW}- Are you in the dialout group?{C.RESET}')
        sys.exit(1)

    tester = ManualControlTester(conn, args)
    tester.start()

    try:
        if not tester.wait_for_fcu(args.wait):
            print(f'\n{C.RED}ERROR: no Pixhawk HEARTBEAT within '
                  f'{args.wait:.0f}s on {endpoint}{C.RESET}')
            print(f'  {C.YELLOW}- Is mavlink-router running?          '
                  f'sudo systemctl status mavlink-router{C.RESET}')
            print(f'  {C.YELLOW}- Is the Pixhawk powered?             '
                  f'(solid LED, USB connected){C.RESET}')
            print(f'  {C.YELLOW}- Is the ROS2 stack stopped?          '
                  f'(MAVROS owns 127.0.0.1:14555 while running){C.RESET}')
            print(f'  {C.YELLOW}- Is QGroundControl closed?            '
                  f'(it owns :14550, not :14555){C.RESET}')
            sys.exit(1)
        tester.request_streams()
        tester.begin()
        code = tester.run()
    except KeyboardInterrupt:
        code = 0
    finally:
        tester.shutdown()
        tester._restore_terminal()
        tester.print_stats()
        print(f'{C.YELLOW}Manual control test stopped.{C.RESET}')
    sys.exit(code)


if __name__ == '__main__':
    main()
