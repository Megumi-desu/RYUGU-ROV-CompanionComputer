#!/usr/bin/env python3
"""
apply_safety_override.py — Disable safety checks for bench testing

Reads current values, then sets:
  ARMING_CHECK=0      — disable all pre-arm checks
  BRD_SAFETYENABLE=0  — no safety switch required
  FS_GCS_ENABLE=0     — disable GCS failsafe
  FS_BATT_ENABLE=0    — disable battery failsafe

Usage:
    python3 tests/control/apply_safety_override.py
"""

import sys
import time
from pymavlink import mavutil

class C:
    RESET  = '\033[0m'
    BOLD   = '\033[1m'
    GREEN  = '\033[92m'
    RED    = '\033[91m'
    YELLOW = '\033[93m'
    DIM    = '\033[2m'

TARGET_SYSID = 1
TARGET_COMPID = 1

PARAMS_TO_READ = [
    'ARMING_CHECK', 'BRD_SAFETYENABLE', 'FS_GCS_ENABLE', 'FS_BATT_ENABLE',
    'MOT_PWM_TYPE', 'FRAME_CONFIG', 'MOT1_ROLL_FACTOR', 'MOT2_ROLL_FACTOR',
    'MOT3_ROLL_FACTOR', 'MOT4_ROLL_FACTOR', 'MOT5_ROLL_FACTOR', 'MOT6_ROLL_FACTOR',
    'MOT1_PITCH_FACTOR', 'MOT2_PITCH_FACTOR', 'MOT3_PITCH_FACTOR',
    'MOT4_PITCH_FACTOR', 'MOT5_PITCH_FACTOR', 'MOT6_PITCH_FACTOR',
    'MOT1_THST_EXPO', 'MOT2_THST_EXPO', 'MOT3_THST_EXPO',
    'MOT4_THST_EXPO', 'MOT5_THST_EXPO', 'MOT6_THST_EXPO',
    'SERVO9_FUNCTION', 'SERVO10_FUNCTION', 'SERVO11_FUNCTION',
    'SERVO12_FUNCTION', 'SERVO13_FUNCTION', 'SERVO14_FUNCTION',
    'JS_GAIN_DEFAULT', 'MIXING_GAIN',
]

PARAMS_TO_SET = {
    'ARMING_CHECK': 0,
    'BRD_SAFETYENABLE': 0,
    'FS_GCS_ENABLE': 0,
    'FS_BATT_ENABLE': 0,
}


def read_params(conn, names, timeout=3.0):
    """Read a list of parameters from the FCU via MAVLink."""
    found = {}
    pending = set(names)
    # Request all params first
    conn.mav.param_request_list_send(TARGET_SYSID, TARGET_COMPID)
    deadline = time.monotonic() + timeout
    last_request = 0.0
    while time.monotonic() < deadline:
        # Periodically re-request names we haven't found
        if pending and time.monotonic() - last_request > 0.5:
            for name in pending:
                conn.mav.param_request_read_send(
                    TARGET_SYSID, TARGET_COMPID, name.encode('utf-8'), -1)
            last_request = time.monotonic()
        msg = conn.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.5)
        if msg:
            name = msg.param_id.rstrip('\x00')
            if name in pending:
                found[name] = msg.param_value
                pending.discard(name)
    return found


def set_param(conn, name, value):
    """Set a float parameter and wait for confirmation."""
    conn.mav.param_set_send(
        TARGET_SYSID, TARGET_COMPID,
        name.encode('utf-8'), float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    # Wait for the PARAM_VALUE confirming the new value
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        msg = conn.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.5)
        if msg:
            got_name = msg.param_id.rstrip('\x00')
            if got_name == name:
                return msg.param_value
    return None


def select_endpoint(device=None):
    if device:
        return device
    return 'udpin:127.0.0.1:14555'


def main():
    import argparse
    parse = argparse.ArgumentParser()
    parse.add_argument('--endpoint', default='udpin:127.0.0.1:14555')
    parse.add_argument('--device', default=None)
    parse.add_argument('--read-only', action='store_true',
                       help='Only read params, don\'t modify anything')
    args = parse.parse_args()

    endpoint = select_endpoint(args.device)
    print(f'Connecting to {endpoint} ...')
    conn = mavutil.mavlink_connection(
        endpoint, baud=115200, dialect='ardupilotmega',
        source_system=255, source_component=190)
    conn.mav.heartbeat_send(190, 8, 0, 0, 0)  # GCS heartbeat
    time.sleep(0.5)

    # Wait for FCU
    msg = conn.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
    if not msg:
        print(f'{C.RED}No FCU heartbeat — is mavlink-router running?{C.RESET}')
        return 1

    print('Reading current parameters ...\n')
    current = read_params(conn, PARAMS_TO_READ, timeout=4.0)

    if not current:
        print(f'{C.RED}No parameters received.{C.RESET}')
        return 1

    # Display the key ones
    key_display = [
        'ARMING_CHECK', 'BRD_SAFETYENABLE', 'FS_GCS_ENABLE', 'FS_BATT_ENABLE',
        'MOT_PWM_TYPE', 'FRAME_CONFIG', 'JS_GAIN_DEFAULT', 'MIXING_GAIN',
        'SERVO9_FUNCTION', 'SERVO13_FUNCTION', 'SERVO14_FUNCTION',
    ]
    print(f'{C.BOLD}Current key parameters:{C.RESET}')
    for k in key_display:
        if k in current:
            print(f'  {k} = {current[k]:.2f}')
    print()

    if args.read_only:
        print(f'{C.YELLOW}Read-only mode — done.{C.RESET}')
        conn.close()
        return 0

    print(f'{C.YELLOW}Setting safety overrides ...{C.RESET}')
    for name, value in PARAMS_TO_SET.items():
        new_val = set_param(conn, name, value)
        if new_val is not None and abs(new_val - value) < 0.01:
            print(f'  {C.GREEN}✓ {name} = {new_val:.2f}{C.RESET}')
        else:
            got = f'{new_val:.2f}' if new_val is not None else 'no confirmation'
            print(f'  {C.YELLOW}? {name} — {got} (may need reboot){C.RESET}')
        time.sleep(0.3)

    print(f'\n{C.DIM}Note: ARMING_CHECK/Brd_SAFETY EFFECTS apply immediately. '
          f'FRAME/MOT_PWM changes need reboot.{C.RESET}')
    conn.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())