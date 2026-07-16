#!/usr/bin/env python3
"""
test_thrusters_gripper.py — Interactive Thruster & Gripper Test Node for RYUGU ROV

Tests the 6 DSHOT thrusters (AUX 1–6) via ManualControl messages and the
gripper servo (MAIN 1) via MAV_CMD_DO_SET_SERVO.

Safety:
  - Thruster tests use only 10% throttle (value=100 out of ±1000).
  - Each axis test runs for 2 seconds, then returns to neutral.
  - ManualControl is published at 10 Hz to prevent Pixhawk timeout failsafe.

Prerequisites:
  - Vehicle must be ARMED and in MANUAL or STABILIZE mode.
  - Use test_arming_mode node first to arm and set mode.

Subscriptions:
  /mavros/state  — monitors armed status and current flight mode

Publishers:
  /mavros/manual_control/send  (mavros_msgs/msg/ManualControl)  @ 10 Hz

Service Clients:
  /mavros/cmd/command  (mavros_msgs/srv/CommandLong)  — for gripper servo

Usage:
  ros2 run ryugu_control test_thrusters_gripper
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

from mavros_msgs.msg import State, ManualControl
from mavros_msgs.srv import CommandLong


# ── ANSI colour helpers ─────────────────────────────────────────────────────
class C:
    """Terminal colour constants."""
    RESET   = '\033[0m'
    BOLD    = '\033[1m'
    DIM     = '\033[2m'
    RED     = '\033[91m'
    GREEN   = '\033[92m'
    YELLOW  = '\033[93m'
    BLUE    = '\033[94m'
    MAGENTA = '\033[95m'
    CYAN    = '\033[96m'
    WHITE   = '\033[97m'
    BG_RED  = '\033[41m'
    BG_GREEN = '\033[42m'
    BG_BLUE = '\033[44m'
    BG_YELLOW = '\033[43m'


# ── Constants ────────────────────────────────────────────────────────────────
PUBLISH_RATE_HZ = 10        # ManualControl publish rate
TEST_DURATION_SEC = 2.0     # Duration of each axis test
TEST_THROTTLE = 100         # 10% throttle (range is -1000 to +1000)

# MAV_CMD_DO_SET_SERVO command ID
MAV_CMD_DO_SET_SERVO = 183

# Gripper servo channel (MAIN 1 = Servo output 1)
GRIPPER_SERVO_CHANNEL = 1

# Gripper PWM values
GRIPPER_PWM_OPEN  = 1100.0
GRIPPER_PWM_CLOSE = 1900.0
GRIPPER_PWM_STOP  = 1500.0


# ── ROS2 Node ───────────────────────────────────────────────────────────────
class TestThrustersGripperNode(Node):
    """Interactive test node for thrusters (ManualControl) and gripper (servo)."""

    SERVICE_TIMEOUT_SEC = 5.0
    CALL_TIMEOUT_SEC = 10.0

    def __init__(self):
        super().__init__('test_thrusters_gripper')

        # ── Current vehicle state ──
        self._connected = False
        self._armed = False
        self._mode = 'UNKNOWN'
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

        # ── Subscriber: /mavros/state ──
        self._state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self._state_callback,
            state_qos,
        )

        # ── Publisher: /mavros/manual_control/send @ 10 Hz ──
        self._mc_pub = self.create_publisher(
            ManualControl,
            '/mavros/manual_control/send',
            10,
        )

        # ── Timer for continuous ManualControl publishing ──
        self._mc_timer = self.create_timer(
            1.0 / PUBLISH_RATE_HZ,
            self._publish_manual_control,
        )

        # ── Service client: /mavros/cmd/command (CommandLong) ──
        self._cmd_client = self.create_client(
            CommandLong,
            '/mavros/cmd/command',
        )

        self.get_logger().info('TestThrustersGripper node initialised.')
        self.get_logger().info(
            f'ManualControl publishing at {PUBLISH_RATE_HZ} Hz '
            f'(neutral until test is triggered).'
        )

    # ── State subscriber callback ────────────────────────────────────────
    def _state_callback(self, msg: State):
        with self._state_lock:
            self._connected = msg.connected
            self._armed = msg.armed
            self._mode = msg.mode

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

    # ── Status line ──────────────────────────────────────────────────────
    def status_line(self) -> str:
        conn = f'{C.GREEN}CONNECTED{C.RESET}' if self.connected else f'{C.RED}DISCONNECTED{C.RESET}'
        arm = (f'{C.BG_RED}{C.WHITE}{C.BOLD} ARMED {C.RESET}'
               if self.armed else
               f'{C.BG_GREEN}{C.WHITE}{C.BOLD} DISARMED {C.RESET}')
        mode_str = f'{C.CYAN}{C.BOLD}{self.mode}{C.RESET}'
        with self._mc_lock:
            mc_str = (f'x={self._mc_x:+.0f}  y={self._mc_y:+.0f}  '
                      f'z={self._mc_z:+.0f}  r={self._mc_r:+.0f}')
        return (f'  FCU: {conn}  |  {arm}  |  Mode: {mode_str}\n'
                f'  ManualControl: [{mc_str}]')

    # ── Continuous ManualControl publisher (10 Hz timer callback) ────────
    def _publish_manual_control(self):
        """
        Publishes ManualControl at 10 Hz.

        This MUST run continuously even when all axes are neutral (0),
        because the Pixhawk's manual control timeout failsafe will
        trigger if it stops receiving messages.
        """
        msg = ManualControl()

        with self._mc_lock:
            msg.x = self._mc_x   # Surge
            msg.y = self._mc_y   # Sway
            msg.z = self._mc_z   # Heave
            msg.r = self._mc_r   # Yaw

        # Buttons field — set to 0 (no button presses)
        msg.buttons = 0
        # Enabled extensions bitmask — not used for basic control
        msg.enabled_extensions = 0

        self._mc_pub.publish(msg)

    # ── Set ManualControl values (thread-safe) ───────────────────────────
    def _set_manual_control(self, x=0.0, y=0.0, z=0.0, r=0.0):
        with self._mc_lock:
            self._mc_x = float(x)
            self._mc_y = float(y)
            self._mc_z = float(z)
            self._mc_r = float(r)

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

    # ── Axis test (Surge / Sway / Heave / Yaw) ──────────────────────────
    def run_axis_test(self, axis: str, value: int = TEST_THROTTLE,
                      duration: float = TEST_DURATION_SEC):
        """
        Run a timed test on one axis.

        Args:
            axis: One of 'x' (Surge), 'y' (Sway), 'z' (Heave), 'r' (Yaw).
            value: Throttle value (-1000 to +1000). Default: +100 (10%).
            duration: Test duration in seconds. Default: 2.0s.
        """
        axis_names = {
            'x': 'SURGE (Forward/Back)',
            'y': 'SWAY (Left/Right)',
            'z': 'HEAVE (Up/Down)',
            'r': 'YAW (Rotate)',
        }

        if not self.armed:
            print(f'\n  {C.RED}{C.BOLD}⚠ Vehicle is DISARMED!{C.RESET}')
            print(f'  {C.YELLOW}ARM the vehicle first using test_arming_mode node.{C.RESET}')
            return

        mode = self.mode
        if mode not in ('MANUAL', 'STABILIZE'):
            print(f'\n  {C.RED}{C.BOLD}⚠ Current mode: {mode}{C.RESET}')
            print(f'  {C.YELLOW}ManualControl requires MANUAL or STABILIZE mode.{C.RESET}')
            return

        name = axis_names.get(axis, axis.upper())
        self._test_active = True
        self._test_name = name

        # Set the target axis
        kwargs = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'r': 0.0}
        kwargs[axis] = float(value)

        pct = abs(value) / 10.0
        direction = '+' if value > 0 else '-'

        print(f'\n  {C.BG_BLUE}{C.WHITE}{C.BOLD} TEST: {name} {C.RESET}')
        print(f'  {C.CYAN}Value: {direction}{pct:.0f}% ({value}/1000) '
              f'for {duration}s{C.RESET}')
        print(f'  {C.DIM}Publishing ManualControl at {PUBLISH_RATE_HZ} Hz...{C.RESET}')

        self._set_manual_control(**kwargs)

        # Run for the specified duration while spinning (so timer callback fires)
        start = time.monotonic()
        while (time.monotonic() - start) < duration and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            remaining = duration - (time.monotonic() - start)
            if remaining > 0:
                # Print countdown (overwrite line)
                sys.stdout.write(
                    f'\r  {C.YELLOW}⏱ Running... {remaining:.1f}s remaining{C.RESET}  '
                )
                sys.stdout.flush()

        # Return to neutral
        self._reset_manual_control()
        self._test_active = False
        self._test_name = ''

        print(f'\r  {C.GREEN}✓ {name} test complete — returned to neutral.{C.RESET}      ')
        print(self.status_line())

    # ── Gripper servo command ────────────────────────────────────────────
    def send_gripper_command(self, pwm: float, label: str) -> bool:
        """
        Send MAV_CMD_DO_SET_SERVO to control the gripper on MAIN 1.

        Args:
            pwm: PWM value (1100=open, 1500=stop, 1900=close).
            label: Human-readable label for logging.

        CommandLong fields for MAV_CMD_DO_SET_SERVO (183):
            param1 = servo channel number (1 = MAIN 1)
            param2 = PWM value (microseconds)
            param3–param7 = unused (0)
        """
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
def get_key_nonblocking(timeout: float = 0.1) -> str:
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


def print_menu():
    """Print the interactive menu."""
    print(f'\n{C.BOLD}{"═" * 62}{C.RESET}')
    print(f'{C.BOLD}{C.CYAN}      RYUGU ROV — Thruster & Gripper Test Console{C.RESET}')
    print(f'{C.BOLD}{"═" * 62}{C.RESET}')
    print(f'  {C.BOLD}{C.BLUE}── Axis Tests (10% throttle, 2s each) ──{C.RESET}')
    print(f'  {C.YELLOW}[1]{C.RESET} Surge  (+x forward)    '
          f'{C.YELLOW}[2]{C.RESET} Sway   (+y right)')
    print(f'  {C.YELLOW}[3]{C.RESET} Heave  (+z ascend)     '
          f'{C.YELLOW}[4]{C.RESET} Yaw    (+r clockwise)')
    print(f'  {C.BOLD}{C.BLUE}── Gripper Servo (MAIN 1) ──{C.RESET}')
    print(f'  {C.YELLOW}[5]{C.RESET} Gripper OPEN  (1100μs) '
          f'{C.YELLOW}[6]{C.RESET} Gripper CLOSE (1900μs)')
    print(f'  {C.YELLOW}[7]{C.RESET} Gripper STOP  (1500μs)')
    print(f'  {C.BOLD}{C.BLUE}── Control ──{C.RESET}')
    print(f'  {C.YELLOW}[0]{C.RESET} Emergency STOP (all neutral)')
    print(f'  {C.YELLOW}[Q]{C.RESET} Quit')
    print(f'{C.BOLD}{"─" * 62}{C.RESET}')


# ── Main loop ────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = TestThrustersGripperNode()

    # Wait for MAVROS connection
    if not node.wait_for_connection(timeout_sec=15.0):
        print(f'\n{C.RED}{C.BOLD}ERROR:{C.RESET} Could not connect to MAVROS.')
        print('Make sure MAVROS is running:')
        print('  ros2 launch ryugu_control mavros_sub.launch.py\n')
        node.destroy_node()
        rclpy.shutdown()
        return

    print_menu()
    print(node.status_line())
    print(f'\n  {C.DIM}Publishing ManualControl at {PUBLISH_RATE_HZ} Hz '
          f'(neutral). Press a key...{C.RESET}\n')

    try:
        while rclpy.ok():
            # Spin for timer callback (ManualControl publisher) and state updates
            rclpy.spin_once(node, timeout_sec=0.05)

            key = get_key_nonblocking(timeout=0.05)
            if not key:
                continue

            key = key.lower()

            if key == '1':
                node.run_axis_test('x', value=+TEST_THROTTLE)

            elif key == '2':
                node.run_axis_test('y', value=+TEST_THROTTLE)

            elif key == '3':
                node.run_axis_test('z', value=+TEST_THROTTLE)

            elif key == '4':
                node.run_axis_test('r', value=+TEST_THROTTLE)

            elif key == '5':
                print(f'\n  {C.BG_GREEN}{C.WHITE}{C.BOLD} GRIPPER: OPEN {C.RESET}')
                node.send_gripper_command(GRIPPER_PWM_OPEN, 'OPEN')
                for _ in range(10):
                    rclpy.spin_once(node, timeout_sec=0.05)
                print(node.status_line())

            elif key == '6':
                print(f'\n  {C.BG_RED}{C.WHITE}{C.BOLD} GRIPPER: CLOSE {C.RESET}')
                node.send_gripper_command(GRIPPER_PWM_CLOSE, 'CLOSE')
                for _ in range(10):
                    rclpy.spin_once(node, timeout_sec=0.05)
                print(node.status_line())

            elif key == '7':
                print(f'\n  {C.BG_YELLOW}{C.WHITE}{C.BOLD} GRIPPER: STOP {C.RESET}')
                node.send_gripper_command(GRIPPER_PWM_STOP, 'STOP')
                for _ in range(10):
                    rclpy.spin_once(node, timeout_sec=0.05)
                print(node.status_line())

            elif key == '0':
                print(f'\n  {C.RED}{C.BOLD}⚡ EMERGENCY STOP — all neutral{C.RESET}')
                node._reset_manual_control()
                node.send_gripper_command(GRIPPER_PWM_STOP, 'STOP')
                for _ in range(10):
                    rclpy.spin_once(node, timeout_sec=0.05)
                print(node.status_line())

            elif key == 'q':
                # Ensure everything is neutral before quitting
                node._reset_manual_control()
                print(f'\n{C.YELLOW}All axes set to neutral. Shutting down...{C.RESET}\n')
                break

            else:
                print(f'  {C.DIM}Unknown key "{key}" — press 1-7, 0, or Q{C.RESET}')

    except KeyboardInterrupt:
        node._reset_manual_control()
        print(f'\n{C.YELLOW}Interrupted. All neutral. Shutting down...{C.RESET}\n')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
