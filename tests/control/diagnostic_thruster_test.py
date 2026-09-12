#!/usr/bin/env python3
"""
diagnostic_thruster_test.py — Individual Axis Thruster Diagnostic

Tests each axis separately while monitoring AUX channel outputs.
Purpose: Identify mixing problems by isolating each control axis.

Usage:
    python3 tests/control/diagnostic_thruster_test.py                    # via mavlink-router
    python3 tests/control/diagnostic_thruster_test.py --max-pct 25      # limit amplitude
    python3 tests/control/diagnostic_thruster_test.py --device /dev/ttyACM0  # direct serial
"""

import argparse
import sys
import time
from pymavlink import mavutil

# ANSI colors
class C:
    RESET   = '\033[0m'
    BOLD    = '\033[1m'
    GREEN   = '\033[92m'
    RED     = '\033[91m'
    YELLOW  = '\033[93m'
    CYAN    = '\033[96m'
    DIM     = '\033[2m'

DEFAULT_SYSID = 255
DEFAULT_COMPID = 190
TARGET_SYSID = 1
TARGET_COMPID = 1

# ArduSub modes
MODE_MANUAL = 0
MODE_STABILIZE = 1
MODE_MOTOR_DETECT = 17

# Neutral: z=500 (ArduSub z→2*(z/1000)-1; z=0 is FULL DOWN, z=500 is neutral)
NEUTRAL_Z = 500

ACK_RESULT_NAMES = {
    0: 'ACCEPTED', 1: 'TEMPORARILY_REJECTED', 2: 'DENIED',
    3: 'UNSUPPORTED', 4: 'FAILED', 5: 'IN_PROGRESS', 6: 'CANCELLED',
}


class DiagnosticTester:
    def __init__(self, conn, args):
        self.conn = conn
        self.target_sysid = args.target_sysid
        self.target_compid = args.target_compid
        self.amp = int(1000 * args.max_pct / 100)
        self.dry_run = args.dry_run
        
        self._armed = False
        self._custom_mode = -1
        self._batt_v = 0.0
        self._servo_raw = [0] * 16
        self._last_fcu_hb = 0.0
        
        # Safety parameters to disable
        self._safety_params = {
            'BRD_SAFETYENABLE': 0,      # Disable safety switch requirement
            'ARMING_CHECK': 0,           # Disable all arming checks
            'FS_GCS_ENABLE': 0,          # Disable GCS failsafe (for testing)
            'FS_BATT_ENABLE': 0,         # Disable battery failsafe (for testing)
            'MOT_PWM_TYPE': 6,           # DSHOT600 (keep this)
        }

    def wait_for_fcu(self, timeout=15.0):
        """Wait for FCU heartbeat."""
        deadline = time.monotonic() + timeout
        print(f'{C.DIM}Waiting for Pixhawk HEARTBEAT ...{C.RESET}')
        
        while time.monotonic() < deadline:
            msg = self.conn.recv_match(blocking=True, timeout=0.5)
            if msg and msg.get_type() == 'HEARTBEAT':
                if msg.get_srcSystem() == self.target_sysid:
                    if msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
                        self._last_fcu_hb = time.monotonic()
                        self._armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                        self._custom_mode = msg.custom_mode
                        mode_name = self._get_mode_name(self._custom_mode)
                        print(f'{C.GREEN}FCU detected — mode={mode_name}, {"ARMED" if self._armed else "DISARMED"}{C.RESET}')
                        return True
        return False

    def _get_mode_name(self, mode):
        modes = {0: 'MANUAL', 1: 'STABILIZE', 2: 'ALT_HOLD', 3: 'AUTO', 
                 4: 'GUIDED', 7: 'CIRCLE', 9: 'SURFACE', 16: 'POSHOLD', 17: 'MOTOR_DETECT'}
        return modes.get(mode, f'?{mode}')

    def disable_safety_params(self):
        """Try to disable safety parameters via MAVLink param set."""
        print(f'{C.YELLOW}Attempting to disable safety parameters ...{C.RESET}')
        
        for param_name, value in self._safety_params.items():
            try:
                self.conn.mav.param_set_send(
                    self.target_sysid,
                    self.target_compid,
                    param_name.encode('utf-8'),
                    float(value),
                    mavutil.mavlink.MAV_PARAM_TYPE_REAL32
                )
                time.sleep(0.2)
                # Check for ACK
                msg = self.conn.recv_match(type='PARAM_VALUE', blocking=True, timeout=1.0)
                if msg:
                    print(f'  {C.GREEN}✓ {param_name} = {value}{C.RESET}')
                else:
                    print(f'  {C.DIM}  {param_name} (no ACK){C.RESET}')
            except Exception as e:
                print(f'  {C.RED}✗ {param_name}: {e}{C.RESET}')
        
        print(f'{C.DIM}Note: Some params require reboot to take effect{C.RESET}')

    def set_mode(self, mode):
        """Set flight mode."""
        mode_name = self._get_mode_name(mode)
        print(f'{C.CYAN}Setting mode: {mode_name} (custom_mode={mode}){C.RESET}')
        
        self.conn.mav.set_mode_send(
            self.target_sysid,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode)
        
        # Wait for mode confirmation
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            msg = self.conn.recv_match(blocking=True, timeout=0.5)
            if msg and msg.get_type() == 'HEARTBEAT':
                if msg.get_srcSystem() == self.target_sysid:
                    if msg.custom_mode == mode:
                        self._custom_mode = mode
                        print(f'{C.GREEN}Mode set: {mode_name}{C.RESET}')
                        return True
        print(f'{C.RED}Failed to set mode{C.RESET}')
        return False

    def arm(self):
        """Arm the vehicle."""
        print(f'{C.YELLOW}Arming ...{C.RESET}')
        
        self.conn.mav.command_long_send(
            self.target_sysid, self.target_compid,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1, 0, 0, 0, 0, 0, 0)
        
        # Wait for ACK and armed state
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            msg = self.conn.recv_match(blocking=True, timeout=0.5)
            if msg:
                mtype = msg.get_type()
                if mtype == 'COMMAND_ACK':
                    if msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                        res = ACK_RESULT_NAMES.get(msg.result, f'?{msg.result}')
                        print(f'  ARM ACK: {res}')
                        if msg.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                            print(f'{C.RED}Arm command rejected{C.RESET}')
                            return False
                elif mtype == 'HEARTBEAT':
                    if msg.get_srcSystem() == self.target_sysid:
                        self._armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                        if self._armed:
                            print(f'{C.GREEN}✓ ARMED{C.RESET}')
                            return True
                elif mtype == 'STATUSTEXT':
                    sev = msg.severity
                    text = msg.text.rstrip('\x00')
                    if sev <= 4 or 'PreArm' in text or 'Arm' in text:
                        colour = C.RED if sev <= 2 else C.YELLOW
                        print(f'  {colour}[STATUSTEXT] {text}{C.RESET}')
        
        print(f'{C.RED}Arm timeout{C.RESET}')
        return False

    def disarm(self):
        """Disarm the vehicle."""
        print(f'{C.YELLOW}Disarming ...{C.RESET}')
        
        self.conn.mav.command_long_send(
            self.target_sysid, self.target_compid,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            0, 0, 0, 0, 0, 0, 0)
        
        # Wait for ACK and disarmed state
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            msg = self.conn.recv_match(blocking=True, timeout=0.5)
            if msg and msg.get_type() == 'HEARTBEAT':
                if msg.get_srcSystem() == self.target_sysid:
                    self._armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    if not self._armed:
                        print(f'{C.GREEN}✓ DISARMED{C.RESET}')
                        return True
        print(f'{C.RED}Disarm timeout{C.RESET}')
        return False

    def request_streams(self):
        """Request telemetry streams."""
        # SYS_STATUS at 2 Hz
        self.conn.mav.command_long_send(
            self.target_sysid, self.target_compid,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, int(1e6 / 2), 0, 0, 0, 0, 0)
        
        # SERVO_OUTPUT_RAW at 20 Hz
        self.conn.mav.command_long_send(
            self.target_sysid, self.target_compid,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, int(1e6 / 20), 0, 0, 0, 0, 0)
        
        print(f'{C.DIM}Requested streams: SYS_STATUS 2 Hz, SERVO_OUTPUT_RAW 20 Hz{C.RESET}')
        time.sleep(0.5)

    def send_heartbeat(self):
        """Send GCS heartbeat."""
        self.conn.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

    def send_manual_control(self, x, y, z, r):
        """Send MANUAL_CONTROL message."""
        self.conn.mav.manual_control_send(self.target_sysid, x, y, z, r, 0)

    def receive_messages(self, timeout=0.5):
        """Receive and process messages, updating state."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            msg = self.conn.recv_match(blocking=True, timeout=0.1)
            if msg:
                mtype = msg.get_type()
                if mtype == 'HEARTBEAT' and msg.get_srcSystem() == self.target_sysid:
                    self._last_fcu_hb = time.monotonic()
                    self._armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    self._custom_mode = msg.custom_mode
                elif mtype == 'SYS_STATUS':
                    self._batt_v = msg.voltage_battery / 1000.0
                elif mtype == 'SERVO_OUTPUT_RAW':
                    self._servo_raw = [getattr(msg, f'servo{i}_raw', 0) for i in range(1, 17)]

    def print_aux_status(self, label=""):
        """Print AUX channel status."""
        aux = self._servo_raw[8:14]  # AUX 1-6 = indices 8-13
        main1 = self._servo_raw[0]   # MAIN 1 (gripper)
        
        arm_s = f'{C.RED}ARMED{C.RESET}' if self._armed else f'{C.GREEN}DISARMED{C.RESET}'
        batt_s = f'{self._batt_v:.1f}V'
        
        aux_str = ' '.join(f'{v:>5}' for v in aux)
        print(f'{C.CYAN}[{label}]{C.RESET} {arm_s} {batt_s}  '
              f'MAIN1:{main1:>5}  AUX[1-6]: {aux_str}')

    def test_axis(self, axis_name, x, y, z, r, duration=2.0, settle_time=0.5):
        """
        Test a single axis by sending a command and monitoring AUX outputs.
        
        Args:
            axis_name: Name of the axis (e.g., "FORWARD")
            x, y, z, r: MANUAL_CONTROL values (-1000 to +1000)
            duration: How long to send the command
            settle_time: Time to wait for thrusters to settle
        """
        print(f'\n{C.BOLD}{"=" * 60}{C.RESET}')
        print(f'{C.BOLD}TEST: {axis_name}{C.RESET}')
        print(f'{C.DIM}Command: x={x:+5d} y={y:+5d} z={z:+5d} r={r:+5d}{C.RESET}')
        print(f'{C.BOLD}{"=" * 60}{C.RESET}')
        
        # Ensure neutral first
        print(f'{C.DIM}Setting neutral (z={NEUTRAL_Z}) ...{C.RESET}')
        for _ in range(10):
            self.send_manual_control(0, 0, NEUTRAL_Z, 0)
            self.receive_messages(0.05)
        time.sleep(0.5)
        
        # Capture baseline
        self.print_aux_status("BASELINE (neutral)")
        baseline_aux = list(self._servo_raw[8:14])
        
        # Send test command
        print(f'{C.GREEN}Sending {axis_name} command ...{C.RESET}')
        start = time.monotonic()
        while time.monotonic() - start < duration:
            self.send_manual_control(x, y, z, r)
            self.receive_messages(0.05)
        
        # Capture during command
        self.print_aux_status(f"DURING {axis_name}")
        during_aux = list(self._servo_raw[8:14])
        
        # Settle
        time.sleep(settle_time)
        
        # Calculate deltas
        print(f'\n{C.BOLD}ANALYSIS:{C.RESET}')
        for i in range(6):
            delta = during_aux[i] - baseline_aux[i]
            if abs(delta) > 10:
                print(f'  AUX {i+1}: {baseline_aux[i]:>5} → {during_aux[i]:>5} (delta: {delta:+5d})')
            else:
                print(f'  AUX {i+1}: {baseline_aux[i]:>5} → {during_aux[i]:>5} (no change)')
        
        # Return to neutral
        print(f'{C.DIM}Returning to neutral (z={NEUTRAL_Z}) ...{C.RESET}')
        for _ in range(10):
            self.send_manual_control(0, 0, NEUTRAL_Z, 0)
            self.receive_messages(0.05)
        time.sleep(0.5)
        
        self.print_aux_status("POST-TEST (neutral)")
        
        return baseline_aux, during_aux

    def run_diagnostic(self, amp_pct=100):
        """Run complete diagnostic test suite."""
        amp = int(1000 * amp_pct / 100)
        
        print(f'\n{C.BOLD}{C.CYAN}RYUGU ROV — Thruster Diagnostic Test{C.RESET}')
        print(f'{C.BOLD}{"=" * 60}{C.RESET}')
        print(f'Amplitude: ±{amp} ({amp_pct}%)')
        print(f'{C.BOLD}{"=" * 60}{C.RESET}\n')
        
        # Setup
        self.send_heartbeat()
        if not self.wait_for_fcu():
            print(f'{C.RED}ERROR: No FCU heartbeat{C.RESET}')
            return 1
        
        self.request_streams()
        
        # Try to disable safety params
        self.disable_safety_params()
        time.sleep(1.0)
        
        # Set mode and arm
        if not self._armed:
            if not self.set_mode(MODE_MANUAL):
                print(f'{C.YELLOW}Trying STABILIZE mode instead ...{C.RESET}')
                self.set_mode(MODE_STABILIZE)
            
            if not self.arm():
                print(f'{C.RED}Failed to arm. Check safety parameters.{C.RESET}')
                print(f'{C.DIM}Try: BRD_SAFETYENABLE=0, ARMING_CHECK=0{C.RESET}')
                return 1
        
        print(f'\n{C.GREEN}Vehicle armed — starting diagnostic sequence{C.RESET}\n')
        
        results = {}
        
# Horizontal-axis tests hold z=NEUTRAL_Z so ONLY the axis under test moves.
        # Test 1: SURGE FORWARD (x = +amp)
        results['surge_forward'] = self.test_axis(
            'SURGE FORWARD', x=amp, y=0, z=NEUTRAL_Z, r=0, duration=2.0)
        
        # Test 2: SURGE BACKWARD (x = -amp)
        results['surge_backward'] = self.test_axis(
            'SURGE BACKWARD', x=-amp, y=0, z=NEUTRAL_Z, r=0, duration=2.0)
        
        # Test 3: SWAY RIGHT (y = +amp)
        results['sway_right'] = self.test_axis(
            'SWAY RIGHT', x=0, y=amp, z=NEUTRAL_Z, r=0, duration=2.0)
        
        # Test 4: SWAY LEFT (y = -amp)
        results['sway_left'] = self.test_axis(
            'SWAY LEFT', x=0, y=-amp, z=NEUTRAL_Z, r=0, duration=2.0)
        
        # Test 5: YAW RIGHT (r = +amp)
        results['yaw_right'] = self.test_axis(
            'YAW RIGHT', x=0, y=0, z=NEUTRAL_Z, r=amp, duration=2.0)
        
        # Test 6: YAW LEFT (r = -amp)
        results['yaw_left'] = self.test_axis(
            'YAW LEFT', x=0, y=0, z=NEUTRAL_Z, r=-amp, duration=2.0)
        
        # Test 7: HEAVE UP (z = +1000 → full up)
        results['heave_up'] = self.test_axis(
            'HEAVE UP', x=0, y=0, z=1000, r=0, duration=2.0)

        # Test 8: HEAVE DOWN (z = 0 → full down; ArduSub bidirectional)
        results['heave_down'] = self.test_axis(
            'HEAVE DOWN', x=0, y=0, z=0, r=0, duration=2.0)

        # Test 9: NEUTRAL Z (z = 500 → should be zero thrust)
        results['z_neutral'] = self.test_axis(
            'Z NEUTRAL (hover)', x=0, y=0, z=NEUTRAL_Z, r=0, duration=2.0)
        
        # Summary
        print(f'\n\n{C.BOLD}{C.CYAN}{"=" * 60}{C.RESET}')
        print(f'{C.BOLD}{C.CYAN}SUMMARY — AUX Channel Response Matrix{C.RESET}')
        print(f'{C.BOLD}{C.CYAN}{"=" * 60}{C.RESET}')
        print(f'{"Axis":<20} {"AUX1":>6} {"AUX2":>6} {"AUX3":>6} {"AUX4":>6} {"AUX5":>6} {"AUX6":>6}')
        print(f'{C.DIM}{"-" * 60}{C.RESET}')
        
        for name, (baseline, during) in results.items():
            deltas = [during[i] - baseline[i] for i in range(6)]
            print(f'{name:<20} {deltas[0]:>+6} {deltas[1]:>+6} {deltas[2]:>+6} {deltas[3]:>+6} {deltas[4]:>+6} {deltas[5]:>+6}')
        
        print(f'\n{C.DIM}Expected mapping:{C.RESET}')
        print(f'  AUX 1-4: Horizontal thrusters (surge, sway, yaw)')
        print(f'  AUX 5-6: Vertical thrusters (heave)')
        print(f'  Physical swap: AUX 5 = MOT 6, AUX 6 = MOT 5')
        
        # Disarm
        self.disarm()
        
        return 0


def main():
    parser = argparse.ArgumentParser(description='RYUGU ROV — Thruster Diagnostic')
    parser.add_argument('--endpoint', default='udpin:127.0.0.1:14555',
                       help='MAVLink endpoint (default: udpin:127.0.0.1:14555)')
    parser.add_argument('--device', default=None,
                       help='Direct serial device')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--sysid', type=int, default=DEFAULT_SYSID)
    parser.add_argument('--compid', type=int, default=DEFAULT_COMPID)
    parser.add_argument('--target-sysid', type=int, default=TARGET_SYSID)
    parser.add_argument('--target-compid', type=int, default=TARGET_COMPID)
    parser.add_argument('--max-pct', type=int, default=100,
                       help='Amplitude percentage (1-100)')
    parser.add_argument('--dry-run', action='store_true',
                       help='Connect only, no commands')
    
    args = parser.parse_args()
    args.max_pct = max(1, min(100, args.max_pct))
    
    endpoint = args.device or args.endpoint
    
    try:
        conn = mavutil.mavlink_connection(
            endpoint, baud=args.baud, dialect='ardupilotmega',
            source_system=args.sysid, source_component=args.compid)
    except Exception as e:
        print(f'{C.RED}ERROR: Cannot connect to "{endpoint}": {e}{C.RESET}')
        if args.device:
            print(f'{C.YELLOW}Is mavlink-router stopped? Try: sudo systemctl stop mavlink-router{C.RESET}')
        else:
            print(f'{C.YELLOW}Is mavlink-router running? Try: sudo systemctl status mavlink-router{C.RESET}')
        return 1
    
    tester = DiagnosticTester(conn, args)
    
    try:
        code = tester.run_diagnostic(args.max_pct)
    except KeyboardInterrupt:
        print(f'\n{C.YELLOW}Interrupted{C.RESET}')
        tester.disarm()
        code = 0
    finally:
        conn.close()
    
    return code


if __name__ == '__main__':
    sys.exit(main())
