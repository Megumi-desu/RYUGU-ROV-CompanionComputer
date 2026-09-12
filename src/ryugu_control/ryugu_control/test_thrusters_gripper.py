#!/usr/bin/env python3
"""
test_thrusters_gripper.py — Interactive Thruster & Gripper Test Node for RYUGU ROV

Tests the 6 DSHOT thrusters (AUX 1–6) via ManualControl messages and the
gripper servo (MAIN 1) via MAV_CMD_DO_SET_SERVO.

Features:
  - 25 Hz ManualControl streaming rate (matching QGroundControl joystick frequency).
  - Live RCOut telemetry monitoring (/mavros/rc/out) for AUX 1-6 and MAIN 1.
  - Configurable test power (default: 70.0% / ±700 out of ±1000).
  - Automatic set mode to MANUAL before running thruster tests.
  - Interactive console menu for all axes (Surge, Sway, Heave, Yaw) and Gripper.

Subscriptions:
  /mavros/state   (mavros_msgs/msg/State)  — monitors armed status and flight mode
  /mavros/rc/out  (mavros_msgs/msg/RCOut)  — monitors live PWM/DShot channel outputs

Publishers:
  /mavros/manual_control/send  (mavros_msgs/msg/ManualControl)  @ 25 Hz

Service Clients:
  /mavros/cmd/command   (mavros_msgs/srv/CommandLong)   — for gripper servo
  /mavros/cmd/arming    (mavros_msgs/srv/CommandBool)   — for arming/disarming
  /mavros/set_mode      (mavros_msgs/srv/SetMode)       — for setting MANUAL mode

Usage:
  ros2 run ryugu_control test_thrusters_gripper
  ros2 run ryugu_control test_thrusters_gripper --ros-args -p power:=70.0
"""

import sys
import select
import termios
import tty
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from mavros_msgs.msg import State, ManualControl, RCOut
from mavros_msgs.srv import CommandLong, CommandBool, SetMode


# ── ANSI colour helpers ─────────────────────────────────────────────────────
class C:
    """Terminal colour constants."""
    RESET     = '\033[0m'
    BOLD      = '\033[1m'
    DIM       = '\033[2m'
    RED       = '\033[91m'
    GREEN     = '\033[92m'
    YELLOW    = '\033[93m'
    BLUE      = '\033[94m'
    MAGENTA   = '\033[95m'
    CYAN      = '\033[96m'
    WHITE     = '\033[97m'
    BG_RED    = '\033[41m'
    BG_GREEN  = '\033[42m'
    BG_BLUE   = '\033[44m'
    BG_YELLOW = '\033[43m'


# ── Constants ────────────────────────────────────────────────────────────────
PUBLISH_RATE_HZ = 25        # ManualControl publish rate (25 Hz matches QGC)
TEST_DURATION_SEC = 2.5     # Duration of each axis test
DEFAULT_POWER_PCT = 70.0    # 70% throttle (700 out of ±1000)

# MAV_CMD_DO_SET_SERVO command ID
MAV_CMD_DO_SET_SERVO = 183

# Gripper servo channel (MAIN 1 = Servo output 1)
GRIPPER_SERVO_CHANNEL = 1

# Gripper PWM values (microseconds)
GRIPPER_PWM_OPEN  = 1300.0   # Full open
GRIPPER_PWM_CLOSE = 2000.0   # Full close
GRIPPER_PWM_STOP  = 1500.0   # Neutral / stop

# Thruster output channel mapping: AUX 1-6 = RC outputs 9-14 (indices 8-13)
THRUSTER_START_IDX = 8
THRUSTER_COUNT     = 6

# Default joystick input deadband percentage (5% = ±50 units)
DEFAULT_DEADBAND_PCT = 5.0


def apply_deadband(value: float, neutral: float = 0.0, deadband_pct: float = DEFAULT_DEADBAND_PCT, max_range: float = None) -> float:
    """
    Apply a deadband around neutral with smooth linear rescaling.

    Args:
        value: Raw input value (-1000..+1000 or 0..1000)
        neutral: Center value (0.0 for x/y/r, 500.0 for z)
        deadband_pct: Deadband as percentage of full range (default 5.0%)
        max_range: Full-scale range magnitude from neutral (default: 500 for z, 1000 for x/y/r)

    Returns:
        Rescaled input float. Returns neutral if within deadband.
    """
    if max_range is None:
        max_range = 500.0 if neutral == 500 else 1000.0
    deadband = max_range * (deadband_pct / 100.0)
    offset = value - neutral
    if abs(offset) <= deadband:
        return float(neutral)
    sign = 1.0 if offset > 0 else -1.0
    rescaled = (abs(offset) - deadband) * max_range / (max_range - deadband)
    return neutral + sign * min(rescaled, max_range)


# ── ROS2 Node ───────────────────────────────────────────────────────────────
class TestThrustersGripperNode(Node):
    """Interactive test node for thrusters (ManualControl) and gripper (servo)."""

    SERVICE_TIMEOUT_SEC = 5.0
    CALL_TIMEOUT_SEC = 10.0

    def __init__(self):
        super().__init__('test_thrusters_gripper')

        # Declare power parameter (default 70.0%)
        self.declare_parameter('power', DEFAULT_POWER_PCT)
        power_val = float(self.get_parameter('power').value)
        self.test_power_pct = max(10.0, min(100.0, power_val))
        self.test_throttle = int(self.test_power_pct * 10.0)   # 70% -> 700

        # ── Current vehicle state ──
        self._connected = False
        self._armed = False
        self._mode = 'UNKNOWN'
        self._rc_out = [0] * 16
        self._state_lock = threading.Lock()

        # ── Current ManualControl setpoint ──
        self._mc_x = 0.0    # Surge  (forward/back)
        self._mc_y = 0.0    # Sway   (left/right)
        self._mc_z = 0.0    # Heave  (up/down)
        self._mc_r = 0.0    # Yaw    (rotate)
        self._mc_lock = threading.Lock()

        # ── Test in-progress flag ──
        self._test_active = False
        self._test_name = ''

        # ── QoS profile ──
        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        rc_out_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        # ── Subscriber: /mavros/state ──
        self._state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self._state_callback,
            state_qos,
        )

        # ── Subscriber: /mavros/rc/out ──
        self._rc_out_sub = self.create_subscription(
            RCOut,
            '/mavros/rc/out',
            self._rc_out_callback,
            rc_out_qos,
        )

        # ── Publisher: /mavros/manual_control/send @ 25 Hz ──
        self._mc_pub = self.create_publisher(
            ManualControl,
            '/mavros/manual_control/send',
            10,
        )

        # ── Timer for continuous ManualControl publishing (25 Hz) ──
        self._mc_timer = self.create_timer(
            1.0 / PUBLISH_RATE_HZ,
            self._publish_manual_control,
        )

        # ── Service clients ──
        self._cmd_client = self.create_client(CommandLong, '/mavros/cmd/command')
        self._arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self._mode_client = self.create_client(SetMode, '/mavros/set_mode')

        self.get_logger().info('TestThrustersGripper node initialised.')
        self.get_logger().info(
            f'ManualControl rate set to {PUBLISH_RATE_HZ} Hz. '
            f'Test power set to {self.test_power_pct:.0f}% (±{self.test_throttle} units).'
        )

    # ── State subscriber callback ────────────────────────────────────────
    def _state_callback(self, msg: State):
        with self._state_lock:
            self._connected = msg.connected
            self._armed = msg.armed
            self._mode = msg.mode

    # ── RCOut subscriber callback ────────────────────────────────────────
    def _rc_out_callback(self, msg: RCOut):
        with self._state_lock:
            chans = list(msg.channels)
            if len(chans) < 16:
                chans.extend([0] * (16 - len(chans)))
            self._rc_out = chans[:16]

    # ── Thread-safe accessors ────────────────────────────────────────────
    @property
    def connected(self) -> bool:
        with self._state_lock:
            return self._connected

    @property
    def armed(self) -> bool:
        with self._state_lock:
            return self._armed

    @property
    def mode(self) -> str:
        with self._state_lock:
            return self._mode

    # ── Status line with RCOut telemetry ──────────────────────────────────
    def status_line(self) -> str:
        conn = f'{C.GREEN}CONNECTED{C.RESET}' if self.connected else f'{C.RED}DISCONNECTED{C.RESET}'
        arm = (f'{C.BG_RED}{C.WHITE}{C.BOLD} ARMED {C.RESET}'
               if self.armed else
               f'{C.BG_GREEN}{C.WHITE}{C.BOLD} DISARMED {C.RESET}')
        mode_str = f'{C.CYAN}{C.BOLD}{self.mode}{C.RESET}'
        with self._mc_lock:
            mc_str = (f'x={self._mc_x:+.0f}  y={self._mc_y:+.0f}  '
                      f'z={self._mc_z:+.0f}  r={self._mc_r:+.0f}')
        with self._state_lock:
            aux = self._rc_out[THRUSTER_START_IDX:THRUSTER_START_IDX + THRUSTER_COUNT]
            main1 = self._rc_out[0] if self._rc_out else 0

        aux_str = ' '.join(f'{v:>4}' for v in aux)
        return (f'  FCU: {conn}  |  {arm}  |  Mode: {mode_str}  |  Power: {self.test_power_pct:.0f}%\n'
                f'  ManualControl: [{mc_str}]\n'
                f'  Telemetry RCOut: AUX[1-6]: {C.YELLOW}[{aux_str}]{C.RESET}  |  MAIN1: {C.CYAN}{main1}µs{C.RESET}')

    # ── Continuous ManualControl publisher (25 Hz timer callback) ────────
    def _publish_manual_control(self):
        msg = ManualControl()
        with self._mc_lock:
            msg.x = float(self._mc_x)   # Surge
            msg.y = float(self._mc_y)   # Sway
            msg.z = float(self._mc_z)   # Heave
            msg.r = float(self._mc_r)   # Yaw

        msg.buttons = 0
        msg.enabled_extensions = 0
        self._mc_pub.publish(msg)

    # ── Set ManualControl values (thread-safe, deadband filtered) ───────────
    def _set_manual_control(self, x=0.0, y=0.0, z=0.0, r=0.0):
        with self._mc_lock:
            self._mc_x = apply_deadband(float(x), neutral=0.0)
            self._mc_y = apply_deadband(float(y), neutral=0.0)
            self._mc_z = apply_deadband(float(z), neutral=0.0)
            self._mc_r = apply_deadband(float(r), neutral=0.0)

    def _reset_manual_control(self):
        self._set_manual_control(0.0, 0.0, 0.0, 0.0)

    # ── Wait for MAVROS connection ───────────────────────────────────────
    def wait_for_connection(self, timeout_sec: float = 15.0) -> bool:
        self.get_logger().info(
            f'Waiting for MAVROS connection (timeout {timeout_sec}s)...'
        )
        start = self.get_clock().now()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.25)
            if self.connected:
                self.get_logger().info('MAVROS connected to FCU.')
                return True
            elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
            if elapsed > timeout_sec:
                self.get_logger().warn('Timed out waiting for MAVROS connection.')
                return False
        return False

    # ── Service helper: Set flight mode to MANUAL ────────────────────────
    def set_mode_manual(self) -> bool:
        if self.mode == 'MANUAL':
            return True
        self.get_logger().info('Setting flight mode -> MANUAL...')
        if not self._mode_client.wait_for_service(timeout_sec=self.SERVICE_TIMEOUT_SEC):
            self.get_logger().error('Service /mavros/set_mode not available.')
            return False
        request = SetMode.Request()
        request.custom_mode = 'MANUAL'
        future = self._mode_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.CALL_TIMEOUT_SEC)
        if future.result() is not None and future.result().mode_sent:
            self.get_logger().info(f'{C.GREEN}Flight mode set to MANUAL.{C.RESET}')
            return True
        self.get_logger().warn(f'{C.YELLOW}Failed to set mode to MANUAL.{C.RESET}')
        return False

    # ── Service helper: Arm / Disarm ─────────────────────────────────────
    def set_arming(self, arm_state: bool) -> bool:
        action_str = 'ARM' if arm_state else 'DISARM'
        self.get_logger().info(f'Sending {action_str} command...')
        if not self._arm_client.wait_for_service(timeout_sec=self.SERVICE_TIMEOUT_SEC):
            self.get_logger().error('Service /mavros/cmd/arming not available.')
            return False
        request = CommandBool.Request()
        request.value = arm_state
        future = self._arm_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.CALL_TIMEOUT_SEC)
        if future.result() is not None and future.result().success:
            self.get_logger().info(f'{C.GREEN}{action_str} accepted by FCU.{C.RESET}')
            return True
        self.get_logger().warn(f'{C.YELLOW}{action_str} rejected by FCU.{C.RESET}')
        return False

    # ── Axis test (Surge / Sway / Heave / Yaw) ──────────────────────────
    def run_axis_test(self, axis: str, value: int, duration: float = TEST_DURATION_SEC):
        axis_names = {
            'x': 'SURGE (Forward/Back)',
            'y': 'SWAY (Left/Right)',
            'z': 'HEAVE (Up/Down)',
            'r': 'YAW (Rotate)',
        }

        # Auto-switch to MANUAL mode before running thruster test
        if self.mode != 'MANUAL':
            self.set_mode_manual()

        if not self.armed:
            print(f'\n  {C.RED}{C.BOLD}⚠ Vehicle is DISARMED!{C.RESET}')
            print(f'  {C.YELLOW}Press [A] to ARM the vehicle first.{C.RESET}')
            return

        name = axis_names.get(axis, axis.upper())
        self._test_active = True
        self._test_name = name

        kwargs = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'r': 0.0}
        kwargs[axis] = float(value)

        pct = abs(value) / 10.0
        direction = '+' if value > 0 else '-'

        print(f'\n  {C.BG_BLUE}{C.WHITE}{C.BOLD} TEST: {name} {C.RESET}')
        print(f'  {C.CYAN}Value: {direction}{pct:.0f}% ({value}/1000) '
              f'for {duration}s{C.RESET}')
        print(f'  {C.DIM}Publishing ManualControl at {PUBLISH_RATE_HZ} Hz...{C.RESET}')

        self._set_manual_control(**kwargs)

        start = time.monotonic()
        while (time.monotonic() - start) < duration and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.04)
            remaining = duration - (time.monotonic() - start)
            if remaining > 0:
                sys.stdout.write(
                    f'\r  {C.YELLOW}⏱ Running... {remaining:.1f}s remaining{C.RESET}  '
                )
                sys.stdout.flush()

        self._reset_manual_control()
        self._test_active = False
        self._test_name = ''

        print(f'\r  {C.GREEN}✓ {name} test complete — returned to neutral.{C.RESET}      ')
        print(self.status_line())

    # ── Gripper servo command ────────────────────────────────────────────
    def send_gripper_command(self, pwm: float, label: str) -> bool:
        self.get_logger().info(f'Sending gripper {label} (PWM={pwm:.0f})...')

        if not self._cmd_client.wait_for_service(timeout_sec=self.SERVICE_TIMEOUT_SEC):
            self.get_logger().error(
                'Service /mavros/cmd/command not available. Is MAVROS running?'
            )
            return False

        request = CommandLong.Request()
        request.broadcast = False
        request.command = MAV_CMD_DO_SET_SERVO    # 183
        request.confirmation = 0
        request.param1 = float(GRIPPER_SERVO_CHANNEL)   # Servo channel = 1 (MAIN 1)
        request.param2 = float(pwm)                     # PWM value
        request.param3 = 0.0
        request.param4 = 0.0
        request.param5 = 0.0
        request.param6 = 0.0
        request.param7 = 0.0

        future = self._cmd_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.CALL_TIMEOUT_SEC)

        if future.result() is None:
            self.get_logger().error(f'Gripper {label} service call timed out.')
            return False

        result = future.result()
        if result.success:
            self.get_logger().info(
                f'{C.GREEN}Gripper {label} command accepted '
                f'(PWM={pwm:.0f}).{C.RESET}'
            )
        else:
            self.get_logger().warn(
                f'{C.YELLOW}Gripper {label} command REJECTED '
                f'(result={result.result}).{C.RESET}'
            )
        return result.success


# ── Terminal I/O helpers ─────────────────────────────────────────────────────
def get_key_nonblocking(timeout: float = 0.04) -> str:
    """Read a single keypress without blocking (Unix only)."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            return sys.stdin.read(1)
        return ''
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def print_menu(power_pct: float):
    """Print the interactive menu."""
    p_int = int(power_pct)
    print(f'\n{C.BOLD}{"═" * 64}{C.RESET}')
    print(f'{C.BOLD}{C.CYAN}      RYUGU ROV — Thruster ({p_int}%) & Gripper Test Console{C.RESET}')
    print(f'{C.BOLD}{"═" * 64}{C.RESET}')
    print(f'  {C.BOLD}{C.BLUE}── Thruster Axis Tests ({p_int}% power, 2.5s duration) ──{C.RESET}')
    print(f'  {C.YELLOW}[1]{C.RESET} Surge Forward  (+{p_int}%)     {C.YELLOW}[2]{C.RESET} Surge Backward (-{p_int}%)')
    print(f'  {C.YELLOW}[3]{C.RESET} Sway Right    (+{p_int}%)     {C.YELLOW}[4]{C.RESET} Sway Left     (-{p_int}%)')
    print(f'  {C.YELLOW}[5]{C.RESET} Heave Up      (+{p_int}%)     {C.YELLOW}[6]{C.RESET} Heave Down    (-{p_int}%)')
    print(f'  {C.YELLOW}[7]{C.RESET} Yaw Clockwise (+{p_int}%)     {C.YELLOW}[8]{C.RESET} Yaw CounterCW (-{p_int}%)')
    print(f'  {C.BOLD}{C.BLUE}── Gripper Servo (MAIN 1) ──{C.RESET}')
    print(f'  {C.YELLOW}[O]{C.RESET} Gripper OPEN  (1300μs)    {C.YELLOW}[C]{C.RESET} Gripper CLOSE (2000μs)')
    print(f'  {C.YELLOW}[V]{C.RESET} Gripper STOP  (1500μs)')
    print(f'  {C.BOLD}{C.BLUE}── Mode & Arming Control ──{C.RESET}')
    print(f'  {C.YELLOW}[A]{C.RESET} Toggle ARM / DISARM        {C.YELLOW}[M]{C.RESET} Set Mode -> MANUAL')
    print(f'  {C.YELLOW}[0]{C.RESET} Emergency STOP (all neutral)')
    print(f'  {C.YELLOW}[Q]{C.RESET} Quit')
    print(f'{C.BOLD}{"─" * 64}{C.RESET}')


# ── Main loop ────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = TestThrustersGripperNode()

    if not node.wait_for_connection(timeout_sec=15.0):
        print(f'\n{C.RED}{C.BOLD}ERROR:{C.RESET} Could not connect to MAVROS.')
        print('Make sure MAVROS is running:')
        print('  ros2 launch ryugu_control mavros_sub.launch.py\n')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return

    print_menu(node.test_power_pct)
    print(node.status_line())
    print(f'\n  {C.DIM}Publishing ManualControl at {PUBLISH_RATE_HZ} Hz '
          f'(neutral). Press a key...{C.RESET}\n')

    p = node.test_throttle

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.04)

            key = get_key_nonblocking(timeout=0.04)
            if not key:
                continue

            key = key.lower()

            if key == '1':
                node.run_axis_test('x', value=+p)

            elif key == '2':
                node.run_axis_test('x', value=-p)

            elif key == '3':
                node.run_axis_test('y', value=+p)

            elif key == '4':
                node.run_axis_test('y', value=-p)

            elif key == '5':
                node.run_axis_test('z', value=+p)

            elif key == '6':
                node.run_axis_test('z', value=-p)

            elif key == '7':
                node.run_axis_test('r', value=+p)

            elif key == '8':
                node.run_axis_test('r', value=-p)

            elif key == 'o':
                print(f'\n  {C.BG_GREEN}{C.WHITE}{C.BOLD} GRIPPER: OPEN {C.RESET}')
                node.send_gripper_command(GRIPPER_PWM_OPEN, 'OPEN')
                for _ in range(10):
                    rclpy.spin_once(node, timeout_sec=0.04)
                print(node.status_line())

            elif key == 'c':
                print(f'\n  {C.BG_RED}{C.WHITE}{C.BOLD} GRIPPER: CLOSE {C.RESET}')
                node.send_gripper_command(GRIPPER_PWM_CLOSE, 'CLOSE')
                for _ in range(10):
                    rclpy.spin_once(node, timeout_sec=0.04)
                print(node.status_line())

            elif key == 'v':
                print(f'\n  {C.BG_YELLOW}{C.WHITE}{C.BOLD} GRIPPER: STOP {C.RESET}')
                node.send_gripper_command(GRIPPER_PWM_STOP, 'STOP')
                for _ in range(10):
                    rclpy.spin_once(node, timeout_sec=0.04)
                print(node.status_line())

            elif key == 'a':
                new_state = not node.armed
                node.set_arming(new_state)
                for _ in range(10):
                    rclpy.spin_once(node, timeout_sec=0.04)
                print(node.status_line())

            elif key == 'm':
                node.set_mode_manual()
                for _ in range(10):
                    rclpy.spin_once(node, timeout_sec=0.04)
                print(node.status_line())

            elif key == '0':
                print(f'\n  {C.RED}{C.BOLD}⚡ EMERGENCY STOP — all neutral{C.RESET}')
                node._reset_manual_control()
                node.send_gripper_command(GRIPPER_PWM_STOP, 'STOP')
                for _ in range(10):
                    rclpy.spin_once(node, timeout_sec=0.04)
                print(node.status_line())

            elif key == 'q':
                node._reset_manual_control()
                print(f'\n{C.YELLOW}All axes set to neutral. Shutting down...{C.RESET}\n')
                break

            else:
                print(f'  {C.DIM}Unknown key "{key}" — press 1-8, O, C, V, A, M, 0, or Q{C.RESET}')

    except KeyboardInterrupt:
        node._reset_manual_control()
        print(f'\n{C.YELLOW}Interrupted. All neutral. Shutting down...{C.RESET}\n')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
