#!/usr/bin/env python3
"""
test_rc_override.py — RC_CHANNELS_OVERRIDE Tester (QGC joystick path)

Drives the ROV with RC_CHANNELS_OVERRIDE exactly like QGroundControl's
ArduSub joystick (ch1=roll, ch2=pitch, ch3=throttle, ch4=yaw).

Idle vertical rest on an ARMED vehicle is the learned hover trim
(~1205) whenever MOT_HOVER_LEARN=2 — expected ArduSub behavior, NOT an
error and NOT specific to this path. They read 1500 only when DISARMED
or when MOT_HOVER_LEARN=0. See AGENTS.md "MOT_HOVER_LEARN".

Protocol support: RC_CHANNELS_OVERRIDE (msgid 70) carries ch1-8 in the
MAVLink1 base payload and ch9-18 in MAVLink2 extensions — so it is NOT
limited to 4 channels.  ArduSub's mixer, however, only consumes ch1-4 as
the roll/pitch/throttle/yaw axes; ch5-8 are accessory channels (RCn_OPTION)
and AUX1-6 are motor outputs commanded through the mixer, not directly
addressable by override.

Verified channel behavior (BlueROV2-heavy, MOT_HOVER_LEARN=2, disarm-safe):
  ch3 thrust 1750/1250 -> heave differential (AUX5/6)
  ch4 yaw    1750/1250 -> yaw diagonal (AUX1-4)
  ch2 pitch  1750/1250 -> verticals common-mode (surge coupling path)
  ch1 roll   1750/1250 -> no response on this FCU
  neutral             -> AUX1-6 = 1500 x6

Usage:
    python3 tests/control/test_rc_override.py                  # keyboard mode (TTY)
    python3 tests/control/test_rc_override.py --sweep          # per-axis scripted test
    python3 tests/control/test_rc_override.py --device /dev/ttyACM0 --sweep
    python3 tests/control/test_rc_override.py --amp 200 --step 25

Keyboard mode keys (hold to ramp, release to spring back to neutral):
    w/s   surge fwd/back (ch2)    a/d   sway (ch1)
    r/f   heave up/down (ch3)     q/e   yaw (ch4)
    k     all -> neutral          SPACE arm/disarm      h help
    1-9   max deflection           +/-  ramp step        ESC/Ctrl+C quit
"""

import argparse
import os
import select
import sys
import termios
import time

from pymavlink import mavutil

class C:
    RESET  = '\033[0m'
    BOLD   = '\033[1m'
    GREEN  = '\033[92m'
    RED    = '\033[91m'
    YELLOW = '\033[93m'
    CYAN   = '\033[96m'
    DIM    = '\033[2m'

DEFAULT_SYSID = 255
DEFAULT_COMPID = 190
TARGET_SYSID = 1
TARGET_COMPID = 1

MODE_MANUAL = 0
NEUTRAL = 1500
MIN_PWM = 1000
MAX_PWM = 2000

ACK_RESULT_NAMES = {
    0: 'ACCEPTED', 1: 'TEMPORARILY_REJECTED', 2: 'DENIED',
    3: 'UNSUPPORTED', 4: 'FAILED', 5: 'IN_PROGRESS', 6: 'CANCELLED',
}

# (channel_index_0based, direction): +1 raises channel toward MAX, -1 toward MIN
KEY_ACTIONS = {
    'w': (1, +1),  # ch2 pitch -> surge forward
    's': (1, -1),  # ch2 pitch -> surge back
    'a': (0, -1),  # ch1 roll  -> sway left
    'd': (0, +1),  # ch1 roll  -> sway right
    'r': (2, +1),  # ch3 throttle -> heave up
    'f': (2, -1),  # ch3 throttle -> heave down
    'q': (3, +1),  # ch4 yaw
    'e': (3, -1),  # ch4 yaw
}
AXIS_NAMES = {0: 'roll', 1: 'pitch', 2: 'throttle', 3: 'yaw'}


class RcOverrideTester:
    def __init__(self, conn, args):
        self.conn = conn
        self.target_sysid = args.target_sysid
        self.target_compid = args.target_compid
        self.hold_sec = args.hold
        self.amp = max(0, min(int(args.amp), 500))
        self.step = max(1, int(args.step))
        self._armed = False
        self._custom_mode = -1
        self._servo_raw = [0] * 16

    def wait_for_fcu(self, timeout=15.0):
        deadline = time.monotonic() + timeout
        print(f'{C.DIM}Waiting for Pixhawk HEARTBEAT ...{C.RESET}')
        while time.monotonic() < deadline:
            msg = self.conn.recv_match(blocking=True, timeout=0.5)
            if msg and msg.get_type() == 'HEARTBEAT' and msg.get_srcSystem() == self.target_sysid:
                self._armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self._custom_mode = msg.custom_mode
                print(f'{C.GREEN}FCU — mode={self._custom_mode}, '
                      f'{"ARMED" if self._armed else "DISARMED"}{C.RESET}')
                return True
        return False

    def set_mode(self, mode):
        print(f'{C.YELLOW}Setting mode {mode} (MANUAL) ...{C.RESET}')
        self.conn.mav.set_mode_send(self.target_sysid,
                                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode)
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            msg = self.conn.recv_match(type='HEARTBEAT', blocking=True, timeout=0.5)
            if msg and msg.get_srcSystem() == self.target_sysid and msg.custom_mode == mode:
                print(f'{C.GREEN}  mode confirmed = {mode}{C.RESET}')
                return True
        print(f'{C.RED}  mode NOT confirmed{C.RESET}')
        return False

    def arm(self, arm_state):
        self.conn.mav.command_long_send(
            self.target_sysid, self.target_compid,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            float(arm_state), 0, 0, 0, 0, 0, 0)
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            msg = self.conn.recv_match(type='COMMAND_ACK', blocking=True, timeout=0.5)
            if msg and msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                name = ACK_RESULT_NAMES.get(msg.result, f'?{msg.result}')
                print(f'{C.YELLOW}  ARM/DISARM ack = {name}{C.RESET}')
                return True
            hb = self.conn.recv_match(type='HEARTBEAT', blocking=True, timeout=0.1)
            if hb and hb.get_srcSystem() == self.target_sysid:
                if bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) == bool(arm_state):
                    return True
        return False

    def stream_override(self, ch):
        """Single rc_channels_override_send followed by a non-blocking servo
        sample of channels 9..14 (AUX1..AUX6)."""
        self.conn.mav.rc_channels_override_send(
            self.target_sysid, self.target_compid, *ch,
            0, 0, 0, 0, 0, 0, 0, 0)
        msg = self.conn.recv_match(type='SERVO_OUTPUT_RAW', blocking=False)
        if msg:
            self._servo_raw = [msg.servo9_raw, msg.servo10_raw, msg.servo11_raw,
                               msg.servo12_raw, msg.servo13_raw, msg.servo14_raw]

    def hold(self, ch, seconds):
        """Stream RC_CHANNELS_OVERRIDE at ~20 Hz for `seconds`, then return
        the freshest SERVO_OUTPUT_RAW of channels 9..14 (AUX1..AUX6)."""
        deadline = time.monotonic() + seconds
        last = None
        while time.monotonic() < deadline:
            self.stream_override(ch)
            if self._servo_raw != [0] * 6:
                last = list(self._servo_raw)
            time.sleep(0.05)
        return last

    def _set_hover_learn(self, value):
        """Set MOT_HOVER_LEARN robustly.  Must run AFTER wait_for_fcu() —
        param writes sent before the FCU link is established get lost.
        Also retry: udpin REUSEPORT can hash individual datagrams to the
        router's socket instead of ours.  Verify via fetch_all stream."""
        self.conn.wait_heartbeat(timeout=5.0)
        for _ in range(8):
            self.conn.mav.param_set_send(
                self.target_sysid, self.target_compid, b'MOT_HOVER_LEARN',
                float(value), mavutil.mavlink.MAV_PARAM_TYPE_INT8)
            time.sleep(0.3)
        confirmed = self._read_param_value(b'MOT_HOVER_LEARN', timeout=15.0)
        if confirmed is not None and int(confirmed) == int(value):
            print(f'{C.GREEN}✓ MOT_HOVER_LEARN = {int(confirmed)} '
                  f'(set OK — reboot for it to fully apply){C.RESET}')
        else:
            print(f'{C.RED}✗ MOT_HOVER_LEARN readback = {confirmed} '
                  f'(wanted {value}); try --reboot, then this again{C.RESET}')

    def _read_param_value(self, name, timeout=15.0):
        """Robustly read one param: keep requesting, then fall back to a full
        fetch_all (whose PARAM_VALUE flood survives the socket split)."""
        name_s = name if isinstance(name, bytes) else name.encode('utf-8')
        deadline = time.monotonic() + timeout
        got = None
        while time.monotonic() < deadline:
            for _ in range(3):
                self.conn.mav.param_request_read_send(
                    self.target_sysid, self.target_compid, name_s, -1)
            p = self.conn.recv_match(type='PARAM_VALUE', blocking=True, timeout=1.0)
            got = p.param_value if p and p.param_id == name_s.decode('utf-8') else None
            if got is not None:
                return got
            self.conn.param_fetch_all()
            p = self.conn.recv_match(type='PARAM_VALUE', blocking=True, timeout=1.0)
            if p and p.param_id == name_s.decode('utf-8'):
                return p.param_value
        return None

    def reboot(self):
        """REBOOT_AUTOPILOT — the only reliable way to clear the lingering
        ~1200 vertical rest that full-rate senders leave in FCU state.
        Non-volatile params (MOT_HOVER_LEARN etc.) survive."""
        print(f'{C.YELLOW}Sending REBOOT_AUTOPILOT ...{C.RESET}')
        self.conn.mav.command_long_send(
            self.target_sysid, self.target_compid,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0,
            1, 0, 0, 0, 0, 0, 0)
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline:
            msg = self.conn.recv_match(type='HEARTBEAT', blocking=True, timeout=0.5)
            if msg and msg.get_type() == 'COMMAND_ACK' and \
                    msg.command == mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
                print(f'{C.GREEN}  reboot ACK received{C.RESET}')
            if msg and msg.get_srcSystem() == self.target_sysid:
                break
        print(f'{C.DIM}Waiting for FCU to come back up ...{C.RESET}')
        time.sleep(5)
        if self.wait_for_fcu(timeout=25.0):
            print(f'{C.GREEN}✓ FCU rebooted.{C.RESET}')
        else:
            print(f'{C.RED}No heartbeat after reboot — check USB link{C.RESET}')

    # ── Interactive keyboard mode ──────────────────────────────────────
    def _enable_raw_mode(self):
        fd = sys.stdin.fileno()
        self._termios_old = termios.tcgetattr(fd)
        new = termios.tcgetattr(fd)
        new[3] &= ~(termios.ICANON | termios.ECHO | termios.ISIG)
        new[6][termios.VMIN] = 0
        new[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, new)

    def _restore_terminal(self):
        if getattr(self, '_termios_old', None) is not None:
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSANOW, self._termios_old)
            self._termios_old = None

    def _line(self, text):
        sys.stdout.write('\r\x1b[K' + text + '\n')
        sys.stdout.flush()

    def _status(self, ch, defl):
        aux = self._servo_raw if self._servo_raw != [0] * 6 else []
        aux_s = f'AUX1-6={" ".join(f"{v:5d}" for v in aux)}' if aux else 'AUX: (no signal yet)'
        vals = ' '.join(f'{AXIS_NAMES[i]}={ch[i]}' for i in range(4))
        sys.stdout.write(f'\r\x1b[K {C.BOLD}{vals}{C.RESET}   {aux_s}   '
                         f'amp=±{self.amp} step={self.step} '
                         f'{"ARMED" if self._armed else "disarmed"}   [h=help] ')
        sys.stdout.flush()

    def _help(self):
        self._line(
            f'{C.BOLD}Keys (hold to ramp, release to spring to neutral):{C.RESET}\n'
            f'  {C.CYAN}w/s{C.RESET} surge fwd/back   {C.CYAN}r/f{C.RESET} heave up/down\n'
            f'  {C.CYAN}a/d{C.RESET} sway L/R         {C.CYAN}q/e{C.RESET} yaw L/R\n'
            f'  {C.CYAN}k{C.RESET} all neutral      {C.CYAN}SPACE{C.RESET} arm/disarm   '
            f'{C.CYAN}h{C.RESET} help\n'
            f'  {C.CYAN}1-9{C.RESET} max deflection  {C.CYAN}+/{C.RESET} ramp step     '
            f'{C.CYAN}ESC{C.RESET} quit\n'
            f'{C.DIM}ch3 throttle UP = r, DOWN = f — be careful, it is bidirectional.{C.RESET}')

    def run_interactive(self):
        if not sys.stdin.isatty():
            self._line(f'{C.RED}stdin is not a TTY — keyboard control unavailable. '
                       f'Use --sweep instead.{C.RESET}')
            return
        if not self.set_mode(MODE_MANUAL):
            return

        self._enable_raw_mode()
        self._termios_old = getattr(self, '_termios_old', None)
        ch = [NEUTRAL] * 8
        defl = [0.0] * 4
        pressed = {}
        quit_now = False
        base_aux = None

        self._help()
        self._line(f'{C.YELLOW}SPACE to arm/disarm. All channels neutral now.{C.RESET}')

        while not quit_now:
            now = time.monotonic()
            while select.select([sys.stdin], [], [], 0)[0]:
                data = os.read(sys.stdin.fileno(), 64)
                if not data:
                    break
                for chkey in data.decode('utf-8', errors='ignore'):
                    if chkey == '\x03' or chkey == '\x1b':
                        quit_now = True
                        break
                    key = chkey.lower()
                    if key == '\r' or key == '\n':
                        continue
                    if key == 'h':
                        self._help()
                    elif key == 'k':
                        defl = [0.0] * 4
                        pressed = {}
                    elif key == ' ':
                        ok = self.arm(not self._armed)
                        if ok:
                            self._armed = not self._armed
                        self._line(f'{C.YELLOW}{"ARMED" if self._armed else "DISARMED"}{C.RESET}')
                    elif key in '123456789':
                        amp = int(key) * 50
                        self.amp = amp
                        self._line(f'{C.CYAN}max deflection ±{self.amp} (RC units){C.RESET}')
                    elif key in ('+', '='):
                        self.step = min(100, self.step + 10)
                        self._line(f'{C.CYAN}ramp step {self.step}{C.RESET}')
                    elif key == '-':
                        self.step = max(5, self.step - 10)
                        self._line(f'{C.CYAN}ramp step {self.step}{C.RESET}')
                    elif key in KEY_ACTIONS:
                        pressed[key] = True
                    if quit_now:
                        break

            # release-event detection on keys we track
            for key in list(pressed):
                if select.select([sys.stdin], [], [], 0)[0]:
                    data = os.read(sys.stdin.fileno(), 64)
                    if data:
                        for chkey in data.decode('utf-8', errors='ignore'):
                            if chkey.lower() == key:
                                pressed[key] = False

            # ramp deflection toward pressed targets / spring back to 0
            for key, (idx, direction) in KEY_ACTIONS.items():
                target_def = self.amp if pressed.get(key) else 0
                cur = defl[idx]
                if cur < target_def:
                    cur = min(target_def, cur + self.step)
                else:
                    cur = max(target_def, cur - self.step)
                defl[idx] = cur

            for i in range(4):
                defl[i] = max(-self.amp, min(self.amp, defl[i]))
                ch[i] = int(round(NEUTRAL + defl[i] * (1 if KEY_ACTIONS else 1)))
                ch[i] = max(MIN_PWM, min(MAX_PWM, ch[i]))

            self.stream_override(ch)
            if self._servo_raw != [0] * 6 and base_aux is None:
                base_aux = list(self._servo_raw)
            self._status(ch, defl)
            deadline = now + 0.05
            while time.monotonic() < deadline and not quit_now:
                time.sleep(0.005)

        self._line('')
        self._restore_terminal()
        self._line(f'{C.YELLOW}Returning to neutral ...{C.RESET}')
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            self.stream_override([NEUTRAL] * 8)
            time.sleep(0.05)
        if self._armed:
            self.arm(False)
        self._line(f'{C.GREEN}Done — disarmed, channels neutral.{C.RESET}')

    # ── Scripted per-axis mode ─────────────────────────────────────────
    def run_sweep(self):
        if not self.set_mode(MODE_MANUAL):
            return
        if not self.arm(True):
            print(f'{C.RED}Cannot arm — aborting{C.RESET}')
            return

        neutral = [NEUTRAL] * 8
        base = self.hold(neutral, max(2.0, self.hold_sec))
        print(f'{C.CYAN}neutral base  AUX1-6={base}{C.RESET}')

        tests = [
            ('ch2 pitch up  (surge_fwd)',  [NEUTRAL, NEUTRAL + self.amp] + [NEUTRAL] * 6),
            ('ch2 pitch dn  (surge_bck)',  [NEUTRAL, NEUTRAL - self.amp] + [NEUTRAL] * 6),
            ('ch1 roll right (sway)',      [NEUTRAL + self.amp, NEUTRAL] + [NEUTRAL] * 6),
            ('ch1 roll left  (sway)',      [NEUTRAL - self.amp, NEUTRAL] + [NEUTRAL] * 6),
            ('ch4 yaw right',              [NEUTRAL, NEUTRAL, NEUTRAL, NEUTRAL + self.amp] + [NEUTRAL] * 4),
            ('ch4 yaw left',               [NEUTRAL, NEUTRAL, NEUTRAL, NEUTRAL - self.amp] + [NEUTRAL] * 4),
            ('ch3 thrust up (heave)',      [NEUTRAL, NEUTRAL, NEUTRAL + self.amp] + [NEUTRAL] * 5),
            ('ch3 thrust dn (heave)',      [NEUTRAL, NEUTRAL, NEUTRAL - self.amp] + [NEUTRAL] * 5),
        ]
        for name, test_ch in tests:
            aux = self.hold(test_ch, self.hold_sec)
            d = [s - b for s, b in zip(aux, base)] if aux and base else None
            print(f'{name:24s} AUX1-6={aux}' + (f'  d={d}' if d else ''))

        regain = self.hold(neutral, max(2.0, self.hold_sec))
        print(f'{C.CYAN}neutral regain AUX1-6={regain}{C.RESET}')
        self.arm(False)


def main():
    ap = argparse.ArgumentParser(description='RC_CHANNELS_OVERRIDE tester (QGC path)')
    op = ap.add_mutually_exclusive_group()
    op.add_argument('--sweep', action='store_true', help='scripted per-axis test (no keyboard)')
    op.add_argument('--interactive', action='store_true', help='keyboard control (default on a TTY)')
    ap.add_argument('--device', default=None,
                    help='serial device (/dev/ttyACM0) or udpin:127.0.0.1:14555 if omitted')
    ap.add_argument('--hold', type=float, default=4.0, help='seconds per axis hold (sweep)')
    ap.add_argument('--amp', type=int, default=250, help='max deflection off neutral (RC units)')
    ap.add_argument('--step', type=int, default=25, help='ramp step per tick (RC units)')
    ap.add_argument('--reboot', action='store_true',
                    help='restart the FCU to clear lingering vertical-channel state, then exit')
    ap.add_argument('--hover-learn', type=int, choices=[0, 1, 2], default=None,
                    help='set MOT_HOVER_LEARN (2=learn+save hover trim [default], '
                         '0=disable hover trim so armed idle verticals read ~1500), then exit')
    ap.add_argument('--target-sysid', type=int, default=TARGET_SYSID)
    ap.add_argument('--target-compid', type=int, default=TARGET_COMPID)
    args = ap.parse_args()

    if args.device:
        conn = mavutil.mavlink_connection(args.device)
    else:
        conn = mavutil.mavlink_connection('udpin:127.0.0.1:14555',
                                          dialect='ardupilotmega')
    conn.mav.srcSystem = DEFAULT_SYSID
    conn.mav.srcComponent = DEFAULT_COMPID

    tester = RcOverrideTester(conn, args)
    if not tester.wait_for_fcu():
        print(f'{C.RED}No FCU heartbeat{C.RESET}')
        sys.exit(1)
    if args.hover_learn is not None:
        tester._set_hover_learn(args.hover_learn)
        sys.exit(0)
    if args.reboot:
        try:
            tester.reboot()
        finally:
            tester._restore_terminal()
        sys.exit(0)
    use_sweep = args.sweep or not (args.interactive or sys.stdin.isatty())
    try:
        if use_sweep:
            tester.run_sweep()
        else:
            tester.run_interactive()
    finally:
        tester.arm(False)
        tester._restore_terminal()


if __name__ == '__main__':
    main()