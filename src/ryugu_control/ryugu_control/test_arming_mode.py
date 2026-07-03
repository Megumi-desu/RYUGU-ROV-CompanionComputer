#!/usr/bin/env python3
"""
test_arming_mode.py — Interactive Arming & Flight-Mode Test Node for RYUGU ROV

Provides a terminal menu to safely command the Pixhawk 2.4.8 running
ArduSub v4.5.7 via MAVROS ROS2 services.

Menu:
  [1] ARM        [2] DISARM
  [3] MANUAL     [4] STABILIZE     [5] ALT_HOLD (Depth Hold)
  [Q] Quit

Subscriptions:
  /mavros/state  — monitors armed status and current flight mode

Service Clients:
  /mavros/cmd/arming  (mavros_msgs/srv/CommandBool)
  /mavros/set_mode    (mavros_msgs/srv/SetMode)

Usage:
  ros2 run ryugu_control test_arming_mode
"""

import sys
import select
import termios
import tty
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode


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


# ── ROS2 Node ───────────────────────────────────────────────────────────────
class TestArmingModeNode(Node):
    """Interactive terminal node for arming/disarming and mode switching."""

    SERVICE_TIMEOUT_SEC = 5.0
    CALL_TIMEOUT_SEC = 10.0

    def __init__(self):
        super().__init__('test_arming_mode')

        # ── Current vehicle state ──
        self._connected = False
        self._armed = False
        self._mode = 'UNKNOWN'
        self._system_status = 0
        self._state_lock = threading.Lock()

        # ── QoS profile matching MAVROS state publisher ──
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

        # ── Service clients ──
        self._arming_client = self.create_client(
            CommandBool,
            '/mavros/cmd/arming',
        )
        self._set_mode_client = self.create_client(
            SetMode,
            '/mavros/set_mode',
        )

        self.get_logger().info('TestArmingMode node initialised.')

    # ── State subscriber callback ────────────────────────────────────────
    def _state_callback(self, msg: State):
        with self._state_lock:
            self._connected = msg.connected
            self._armed = msg.armed
            self._mode = msg.mode
            self._system_status = msg.system_status

    # ── Public accessors (thread-safe) ───────────────────────────────────
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

    @property
    def system_status(self) -> int:
        with self._state_lock:
            return self._system_status

    # ── Formatted status string ──────────────────────────────────────────
    def status_line(self) -> str:
        conn = f'{C.GREEN}CONNECTED{C.RESET}' if self.connected else f'{C.RED}DISCONNECTED{C.RESET}'
        arm = f'{C.BG_RED}{C.WHITE}{C.BOLD} ARMED {C.RESET}' if self.armed else f'{C.BG_GREEN}{C.WHITE}{C.BOLD} DISARMED {C.RESET}'
        mode_str = f'{C.CYAN}{C.BOLD}{self.mode}{C.RESET}'
        return f'  FCU: {conn}  |  {arm}  |  Mode: {mode_str}'

    # ── Wait for MAVROS connection ───────────────────────────────────────
    def wait_for_connection(self, timeout_sec: float = 15.0) -> bool:
        """Block until /mavros/state reports connected=True."""
        self.get_logger().info(
            f'Waiting for MAVROS connection (timeout {timeout_sec}s)...'
        )
        start = self.get_clock().now()
        rate = self.create_rate(4)  # 4 Hz polling
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.25)
            if self.connected:
                self.get_logger().info('MAVROS connected to FCU.')
                return True
            elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
            if elapsed > timeout_sec:
                self.get_logger().warn(
                    'Timed out waiting for MAVROS connection.'
                )
                return False
        return False

    # ── Service helpers ──────────────────────────────────────────────────
    def _wait_for_service(self, client, name: str) -> bool:
        if not client.wait_for_service(timeout_sec=self.SERVICE_TIMEOUT_SEC):
            self.get_logger().error(
                f'Service {name} not available after '
                f'{self.SERVICE_TIMEOUT_SEC}s. Is MAVROS running?'
            )
            return False
        return True

    def send_arm(self, arm: bool) -> bool:
        """Send arm (True) or disarm (False) command. Returns success."""
        label = 'ARM' if arm else 'DISARM'
        self.get_logger().info(f'Sending {label} command...')

        if not self._wait_for_service(self._arming_client, '/mavros/cmd/arming'):
            return False

        request = CommandBool.Request()
        request.value = arm

        future = self._arming_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.CALL_TIMEOUT_SEC)

        if future.result() is None:
            self.get_logger().error(f'{label} service call timed out.')
            return False

        result = future.result()
        if result.success:
            self.get_logger().info(f'{C.GREEN}{label} command accepted by FCU.{C.RESET}')
        else:
            self.get_logger().warn(
                f'{C.YELLOW}{label} command REJECTED by FCU (result={result.result}).{C.RESET}'
            )
        return result.success

    def send_set_mode(self, mode: str) -> bool:
        """Send set_mode command. Returns success."""
        self.get_logger().info(f'Setting flight mode → {mode}...')

        if not self._wait_for_service(self._set_mode_client, '/mavros/set_mode'):
            return False

        request = SetMode.Request()
        request.custom_mode = mode

        future = self._set_mode_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.CALL_TIMEOUT_SEC)

        if future.result() is None:
            self.get_logger().error(f'set_mode({mode}) service call timed out.')
            return False

        result = future.result()
        if result.mode_sent:
            self.get_logger().info(
                f'{C.GREEN}Mode change → {mode} accepted by FCU.{C.RESET}'
            )
        else:
            self.get_logger().warn(
                f'{C.YELLOW}Mode change → {mode} REJECTED by FCU.{C.RESET}'
            )
        return result.mode_sent


# ── Terminal I/O helpers ─────────────────────────────────────────────────────
def get_key_nonblocking(timeout: float = 0.1) -> str:
    """Read a single keypress without blocking (Unix only)."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
            return key
        return ''
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def print_menu():
    """Print the interactive menu."""
    print(f'\n{C.BOLD}{"═" * 60}{C.RESET}')
    print(f'{C.BOLD}{C.CYAN}        RYUGU ROV — Arming & Mode Test Console{C.RESET}')
    print(f'{C.BOLD}{"═" * 60}{C.RESET}')
    print(f'  {C.YELLOW}[1]{C.RESET} ARM           {C.YELLOW}[2]{C.RESET} DISARM')
    print(f'  {C.YELLOW}[3]{C.RESET} MANUAL        {C.YELLOW}[4]{C.RESET} STABILIZE')
    print(f'  {C.YELLOW}[5]{C.RESET} ALT_HOLD      {C.YELLOW}[Q]{C.RESET} Quit')
    print(f'{C.BOLD}{"─" * 60}{C.RESET}')


# ── Main loop ────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = TestArmingModeNode()

    # Wait for MAVROS to connect before showing the menu
    if not node.wait_for_connection(timeout_sec=15.0):
        print(f'\n{C.RED}{C.BOLD}ERROR:{C.RESET} Could not connect to MAVROS.')
        print('Make sure MAVROS is running:')
        print('  ros2 launch ryugu_control mavros_sub.launch.py\n')
        node.destroy_node()
        rclpy.shutdown()
        return

    print_menu()
    print(node.status_line())
    print(f'\n  {C.DIM}Press a key to send a command...{C.RESET}\n')

    try:
        while rclpy.ok():
            # Spin to process /mavros/state callbacks
            rclpy.spin_once(node, timeout_sec=0.05)

            key = get_key_nonblocking(timeout=0.1)
            if not key:
                continue

            key = key.lower()

            if key == '1':
                print(f'\n{C.MAGENTA}▶ ARM{C.RESET}')
                node.send_arm(True)
                # Spin a few times to let state update arrive
                for _ in range(20):
                    rclpy.spin_once(node, timeout_sec=0.05)
                print(node.status_line())

            elif key == '2':
                print(f'\n{C.MAGENTA}▶ DISARM{C.RESET}')
                node.send_arm(False)
                for _ in range(20):
                    rclpy.spin_once(node, timeout_sec=0.05)
                print(node.status_line())

            elif key == '3':
                print(f'\n{C.MAGENTA}▶ MANUAL mode{C.RESET}')
                node.send_set_mode('MANUAL')
                for _ in range(20):
                    rclpy.spin_once(node, timeout_sec=0.05)
                print(node.status_line())

            elif key == '4':
                print(f'\n{C.MAGENTA}▶ STABILIZE mode{C.RESET}')
                node.send_set_mode('STABILIZE')
                for _ in range(20):
                    rclpy.spin_once(node, timeout_sec=0.05)
                print(node.status_line())

            elif key == '5':
                print(f'\n{C.MAGENTA}▶ ALT_HOLD (Depth Hold) mode{C.RESET}')
                node.send_set_mode('ALT_HOLD')
                for _ in range(20):
                    rclpy.spin_once(node, timeout_sec=0.05)
                print(node.status_line())

            elif key == 'q':
                print(f'\n{C.YELLOW}Shutting down...{C.RESET}\n')
                break

            else:
                print(f'  {C.DIM}Unknown key "{key}" — press 1-5 or Q{C.RESET}')

    except KeyboardInterrupt:
        print(f'\n{C.YELLOW}Interrupted. Shutting down...{C.RESET}\n')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
