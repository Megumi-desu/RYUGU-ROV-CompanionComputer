#!/usr/bin/env python3
"""
test_sensor_reader.py — Real-Time Sensor Display for RYUGU ROV

Subscribes to MAVROS sensor topics and displays clean, human-readable
data that refreshes in-place every 100ms.

Subscriptions:
  /mavros/imu/data               (sensor_msgs/msg/Imu)
  /mavros/local_position/pose    (geometry_msgs/msg/PoseStamped)
  /mavros/global_position/rel_alt (std_msgs/msg/Float64)
  /mavros/battery/battery        (sensor_msgs/msg/BatteryState)
  /mavros/state                  (mavros_msgs/msg/State)

Displayed Data:
  - Roll / Pitch / Yaw (degrees) from EKF-fused orientation
  - Depth (metres) from relative altitude (Bar30 pressure sensor)
  - Battery voltage (V) and current (A)
  - FCU connection and arming state

Usage:
  ros2 run ryugu_control test_sensor_reader
"""

import sys
import math
import threading
import signal

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import Imu, BatteryState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
from mavros_msgs.msg import State


# ── ANSI escape helpers ─────────────────────────────────────────────────────
class C:
    """Terminal colour and cursor constants."""
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

    # Cursor control
    CLEAR_SCREEN = '\033[2J'
    HOME         = '\033[H'
    HIDE_CURSOR  = '\033[?25l'
    SHOW_CURSOR  = '\033[?25h'
    CLEAR_LINE   = '\033[2K'


# ── Quaternion → Euler conversion ────────────────────────────────────────────
def quaternion_to_euler_deg(x: float, y: float, z: float, w: float):
    """
    Convert a quaternion (x, y, z, w) to Euler angles (roll, pitch, yaw)
    in degrees, using the ZYX (aerospace) convention.

    Returns:
        Tuple[float, float, float]: (roll, pitch, yaw) in degrees.
    """
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation) — clamped to avoid NaN at gimbal lock
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))


# ── Sensor data container ───────────────────────────────────────────────────
class SensorData:
    """Thread-safe container for all sensor readings."""

    def __init__(self):
        self._lock = threading.Lock()

        # IMU (from /mavros/imu/data)
        self.imu_roll: float | None = None
        self.imu_pitch: float | None = None
        self.imu_yaw: float | None = None
        self.imu_ax: float | None = None   # linear acceleration
        self.imu_ay: float | None = None
        self.imu_az: float | None = None
        self.imu_gx: float | None = None   # angular velocity
        self.imu_gy: float | None = None
        self.imu_gz: float | None = None
        self.imu_stamp = None

        # EKF local position (from /mavros/local_position/pose)
        self.ekf_roll: float | None = None
        self.ekf_pitch: float | None = None
        self.ekf_yaw: float | None = None
        self.ekf_x: float | None = None
        self.ekf_y: float | None = None
        self.ekf_z: float | None = None
        self.ekf_stamp = None

        # Depth / altitude (from /mavros/global_position/rel_alt)
        self.depth: float | None = None
        self.depth_stamp = None

        # Battery (from /mavros/battery/battery)
        self.batt_voltage: float | None = None
        self.batt_current: float | None = None
        self.batt_percentage: float | None = None
        self.batt_stamp = None

        # FCU state (from /mavros/state)
        self.fcu_connected = False
        self.fcu_armed = False
        self.fcu_mode = 'UNKNOWN'
        self.state_stamp = None

    def lock(self):
        return self._lock


# ── ROS2 Node ───────────────────────────────────────────────────────────────
class TestSensorReaderNode(Node):
    """Subscribes to MAVROS sensor topics and stores latest readings."""

    def __init__(self):
        super().__init__('test_sensor_reader')
        self.data = SensorData()

        # ── QoS profiles ──
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        # ── /mavros/imu/data (sensor_msgs/msg/Imu) ──
        self.create_subscription(
            Imu, '/mavros/imu/data',
            self._imu_callback, best_effort_qos,
        )

        # ── /mavros/local_position/pose (geometry_msgs/msg/PoseStamped) ──
        self.create_subscription(
            PoseStamped, '/mavros/local_position/pose',
            self._local_pos_callback, best_effort_qos,
        )

        # ── /mavros/global_position/rel_alt (std_msgs/msg/Float64) ──
        self.create_subscription(
            Float64, '/mavros/global_position/rel_alt',
            self._rel_alt_callback, best_effort_qos,
        )

        # ── /mavros/battery/battery (sensor_msgs/msg/BatteryState) ──
        # Try both RELIABLE and BEST_EFFORT since MAVROS version may vary
        self.create_subscription(
            BatteryState, '/mavros/battery/battery',
            self._battery_callback, best_effort_qos,
        )

        # ── /mavros/state (mavros_msgs/msg/State) ──
        self.create_subscription(
            State, '/mavros/state',
            self._state_callback, reliable_qos,
        )

        # ── Display refresh timer (100ms = 10 Hz) ──
        self._display_timer = self.create_timer(0.1, self._display_callback)

        # ── Staleness timeout (seconds) ──
        self._stale_timeout = 3.0

        self.get_logger().info('TestSensorReader node initialised.')

    # ── Callbacks ────────────────────────────────────────────────────────
    def _imu_callback(self, msg: Imu):
        o = msg.orientation
        roll, pitch, yaw = quaternion_to_euler_deg(o.x, o.y, o.z, o.w)
        with self.data.lock():
            self.data.imu_roll = roll
            self.data.imu_pitch = pitch
            self.data.imu_yaw = yaw
            self.data.imu_ax = msg.linear_acceleration.x
            self.data.imu_ay = msg.linear_acceleration.y
            self.data.imu_az = msg.linear_acceleration.z
            self.data.imu_gx = msg.angular_velocity.x
            self.data.imu_gy = msg.angular_velocity.y
            self.data.imu_gz = msg.angular_velocity.z
            self.data.imu_stamp = self.get_clock().now()

    def _local_pos_callback(self, msg: PoseStamped):
        o = msg.pose.orientation
        p = msg.pose.position
        roll, pitch, yaw = quaternion_to_euler_deg(o.x, o.y, o.z, o.w)
        with self.data.lock():
            self.data.ekf_roll = roll
            self.data.ekf_pitch = pitch
            self.data.ekf_yaw = yaw
            self.data.ekf_x = p.x
            self.data.ekf_y = p.y
            self.data.ekf_z = p.z
            self.data.ekf_stamp = self.get_clock().now()

    def _rel_alt_callback(self, msg: Float64):
        with self.data.lock():
            self.data.depth = msg.data
            self.data.depth_stamp = self.get_clock().now()

    def _battery_callback(self, msg: BatteryState):
        with self.data.lock():
            self.data.batt_voltage = msg.voltage
            self.data.batt_current = msg.current
            self.data.batt_percentage = msg.percentage
            self.data.batt_stamp = self.get_clock().now()

    def _state_callback(self, msg: State):
        with self.data.lock():
            self.data.fcu_connected = msg.connected
            self.data.fcu_armed = msg.armed
            self.data.fcu_mode = msg.mode
            self.data.state_stamp = self.get_clock().now()

    # ── Staleness check ──────────────────────────────────────────────────
    def _is_stale(self, stamp) -> bool:
        """Returns True if the stamp is None or older than _stale_timeout."""
        if stamp is None:
            return True
        elapsed = (self.get_clock().now() - stamp).nanoseconds / 1e9
        return elapsed > self._stale_timeout

    # ── Formatting helpers ───────────────────────────────────────────────
    def _fmt_angle(self, val, stamp) -> str:
        if self._is_stale(stamp) or val is None:
            return f'{C.RED}{"N/A":>8s}{C.RESET}'
        return f'{C.GREEN}{val:+8.2f}°{C.RESET}'

    def _fmt_float(self, val, stamp, unit: str, fmt: str = '+8.3f') -> str:
        if self._is_stale(stamp) or val is None:
            return f'{C.RED}{"N/A":>8s}{C.RESET} {unit}'
        return f'{C.GREEN}{val:{fmt}}{C.RESET} {unit}'

    def _fmt_status(self, label: str, ok: bool, stamp) -> str:
        if self._is_stale(stamp):
            return f'{C.RED}DISCONNECTED{C.RESET}'
        return f'{C.GREEN}{label}{C.RESET}' if ok else f'{C.RED}{label}{C.RESET}'

    # ── Display refresh (10 Hz) ──────────────────────────────────────────
    def _display_callback(self):
        d = self.data
        now = self.get_clock().now()

        with d.lock():
            # ── FCU Status ──
            if self._is_stale(d.state_stamp):
                fcu_line = f'{C.RED}DISCONNECTED{C.RESET}'
                armed_line = f'{C.RED}N/A{C.RESET}'
                mode_line = f'{C.RED}N/A{C.RESET}'
            else:
                fcu_line = (f'{C.GREEN}CONNECTED{C.RESET}'
                            if d.fcu_connected else
                            f'{C.RED}DISCONNECTED{C.RESET}')
                armed_line = (f'{C.BG_RED}{C.WHITE}{C.BOLD} ARMED {C.RESET}'
                              if d.fcu_armed else
                              f'{C.BG_GREEN}{C.WHITE}{C.BOLD} DISARMED {C.RESET}')
                mode_line = f'{C.CYAN}{C.BOLD}{d.fcu_mode}{C.RESET}'

            # ── IMU ──
            imu_stale = self._is_stale(d.imu_stamp)
            imu_roll  = self._fmt_angle(d.imu_roll, d.imu_stamp)
            imu_pitch = self._fmt_angle(d.imu_pitch, d.imu_stamp)
            imu_yaw   = self._fmt_angle(d.imu_yaw, d.imu_stamp)

            if imu_stale or d.imu_az is None:
                imu_accel = f'{C.RED}N/A{C.RESET}'
                imu_gyro  = f'{C.RED}N/A{C.RESET}'
            else:
                imu_accel = (f'{C.GREEN}{d.imu_ax:+7.3f}  '
                             f'{d.imu_ay:+7.3f}  {d.imu_az:+7.3f}{C.RESET} m/s²')
                imu_gyro  = (f'{C.GREEN}{d.imu_gx:+7.4f}  '
                             f'{d.imu_gy:+7.4f}  {d.imu_gz:+7.4f}{C.RESET} rad/s')

            # ── EKF Local Position ──
            ekf_stale = self._is_stale(d.ekf_stamp)
            ekf_roll  = self._fmt_angle(d.ekf_roll, d.ekf_stamp)
            ekf_pitch = self._fmt_angle(d.ekf_pitch, d.ekf_stamp)
            ekf_yaw   = self._fmt_angle(d.ekf_yaw, d.ekf_stamp)

            if ekf_stale or d.ekf_x is None:
                ekf_pos = f'{C.RED}N/A{C.RESET}'
            else:
                ekf_pos = (f'{C.GREEN}x={d.ekf_x:+7.3f}  '
                           f'y={d.ekf_y:+7.3f}  z={d.ekf_z:+7.3f}{C.RESET} m')

            # ── Depth (rel_alt) ──
            depth_stale = self._is_stale(d.depth_stamp)
            if depth_stale or d.depth is None:
                depth_str = f'{C.RED}{"N/A":>8s}{C.RESET} m  {C.DIM}(Bar30 DISCONNECTED){C.RESET}'
            else:
                depth_str = f'{C.GREEN}{d.depth:+8.3f}{C.RESET} m  {C.DIM}(Bar30){C.RESET}'

            # ── Battery ──
            batt_stale = self._is_stale(d.batt_stamp)
            if batt_stale or d.batt_voltage is None:
                batt_v   = f'{C.RED}{"N/A":>7s}{C.RESET} V'
                batt_a   = f'{C.RED}{"N/A":>7s}{C.RESET} A'
                batt_pct = f'{C.RED}N/A{C.RESET}'
            else:
                batt_v = f'{C.GREEN}{d.batt_voltage:7.2f}{C.RESET} V'
                batt_a_val = d.batt_current if d.batt_current is not None else 0.0
                batt_a = f'{C.GREEN}{batt_a_val:7.2f}{C.RESET} A'
                if d.batt_percentage is not None and d.batt_percentage >= 0:
                    pct = d.batt_percentage * 100.0
                    pct_color = C.GREEN if pct > 30 else (C.YELLOW if pct > 15 else C.RED)
                    batt_pct = f'{pct_color}{pct:5.1f}%{C.RESET}'
                else:
                    batt_pct = f'{C.DIM}N/A{C.RESET}'

            # ── Topic status indicators ──
            def topic_status(stamp, name):
                if self._is_stale(stamp):
                    return f'{C.RED}✗{C.RESET} {C.DIM}{name}{C.RESET}'
                return f'{C.GREEN}✓{C.RESET} {name}'

            ts_imu   = topic_status(d.imu_stamp, '/mavros/imu/data')
            ts_ekf   = topic_status(d.ekf_stamp, '/mavros/local_position/pose')
            ts_depth = topic_status(d.depth_stamp, '/mavros/global_position/rel_alt')
            ts_batt  = topic_status(d.batt_stamp, '/mavros/battery/battery')
            ts_state = topic_status(d.state_stamp, '/mavros/state')

        # ── Render display ───────────────────────────────────────────────
        lines = []
        w = 64  # display width

        lines.append(f'{C.HOME}')  # Move cursor to top-left
        lines.append(f'{C.BOLD}{"═" * w}{C.RESET}')
        lines.append(f'{C.BOLD}{C.CYAN}{"RYUGU ROV — SENSOR MONITOR":^{w}}{C.RESET}')
        lines.append(f'{C.BOLD}{"═" * w}{C.RESET}')

        # FCU status
        lines.append(f'  {C.BOLD}FCU:{C.RESET}   {fcu_line}  |  {armed_line}  |  Mode: {mode_line}')
        lines.append(f'{C.BOLD}{"─" * w}{C.RESET}')

        # IMU section
        lines.append(f'  {C.BOLD}{C.BLUE}▸ IMU (Raw Orientation){C.RESET}')
        lines.append(f'    Roll: {imu_roll}   Pitch: {imu_pitch}   Yaw: {imu_yaw}')
        lines.append(f'    Accel [x y z]: {imu_accel}')
        lines.append(f'    Gyro  [x y z]: {imu_gyro}')
        lines.append(f'{C.BOLD}{"─" * w}{C.RESET}')

        # EKF section
        lines.append(f'  {C.BOLD}{C.BLUE}▸ EKF Local Position (Fused){C.RESET}')
        lines.append(f'    Roll: {ekf_roll}   Pitch: {ekf_pitch}   Yaw: {ekf_yaw}')
        lines.append(f'    Position:      {ekf_pos}')
        lines.append(f'{C.BOLD}{"─" * w}{C.RESET}')

        # Depth section
        lines.append(f'  {C.BOLD}{C.BLUE}▸ Depth / Altitude{C.RESET}')
        lines.append(f'    Relative Alt:  {depth_str}')
        lines.append(f'{C.BOLD}{"─" * w}{C.RESET}')

        # Battery section
        lines.append(f'  {C.BOLD}{C.BLUE}▸ Battery{C.RESET}')
        lines.append(f'    Voltage: {batt_v}   |   Current: {batt_a}   |   {batt_pct}')
        lines.append(f'{C.BOLD}{"─" * w}{C.RESET}')

        # Topic status
        lines.append(f'  {C.BOLD}{C.DIM}Topic Status:{C.RESET}')
        lines.append(f'    {ts_state}')
        lines.append(f'    {ts_imu}')
        lines.append(f'    {ts_ekf}')
        lines.append(f'    {ts_depth}')
        lines.append(f'    {ts_batt}')
        lines.append(f'{C.BOLD}{"═" * w}{C.RESET}')
        lines.append(f'  {C.DIM}Refreshing at 10 Hz  |  Press Ctrl+C to quit{C.RESET}')

        # Clear any leftover lines below
        lines.append(f'{C.CLEAR_LINE}')
        lines.append(f'{C.CLEAR_LINE}')

        sys.stdout.write('\n'.join(lines))
        sys.stdout.flush()


# ── Main ─────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = TestSensorReaderNode()

    # Hide cursor and clear screen
    sys.stdout.write(C.HIDE_CURSOR + C.CLEAR_SCREEN)
    sys.stdout.flush()

    # Restore cursor on exit
    def _restore_terminal(signum=None, frame=None):
        sys.stdout.write(C.SHOW_CURSOR + '\n')
        sys.stdout.flush()

    signal.signal(signal.SIGINT, lambda s, f: None)  # Let KeyboardInterrupt propagate

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        _restore_terminal()
        node.destroy_node()
        rclpy.shutdown()
        print(f'\n{C.YELLOW}Sensor reader stopped.{C.RESET}\n')


if __name__ == '__main__':
    main()
