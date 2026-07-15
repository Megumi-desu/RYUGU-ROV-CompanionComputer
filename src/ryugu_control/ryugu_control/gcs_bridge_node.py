#!/usr/bin/env python3
"""
gcs_bridge_node.py — Unified GCS ↔ Jetson ↔ Pixhawk Communication Bridge

This ROS2 node manages the bidirectional UDP telemetry link between the
Operator GCS Laptop and the Pixhawk flight controller via MAVROS.

Architecture:
  GCS Laptop (192.168.1.100)  ←─ UDP :5001/:5002 ─→  Jetson Orin Nano (192.168.1.10)
                                                              │
                                                        MAVROS (USB serial)
                                                              │
                                                      Pixhawk 2.4.8 (ArduSub v4.5.7)

Uplink (GCS → Pixhawk):
  - CMD_MOTION  (0x81) → /mavros/manual_control/send   @ 10 Hz
  - CMD_MODE    (0x82) → /mavros/set_mode              (ACK)
  - CMD_GRIPPER (0x83) → /mavros/cmd/command           (MAV_CMD_DO_SET_SERVO)
  - CMD_BALLAST (0x84) → parsed, no hardware action
  - CMD_ARM     (0x85) → /mavros/cmd/arming            (ACK)
  - CMD_ESTOP   (0x86) → disarm immediately            (ACK)

Downlink (Pixhawk → GCS):
  - TELEM_IMU    (0x01): pitch, roll, yaw (°)           @ 20 Hz
  - TELEM_DEPTH  (0x02): depth_m, altitude_m            @ 20 Hz
  - TELEM_STATUS (0x03): battery, arm, mode, thrusters  @ 20 Hz

Packet Format:
  [SYNC: 0xAA55 LE (2B)] [ID (1B)] [LEN (2B LE)] [PAYLOAD (0..1024B)] [CRC-16 (2B)]
  CRC-16/CCITT-FALSE: poly=0x1021, init=0xFFFF, no reflection

Usage:
  ros2 run ryugu_control gcs_bridge_node
  ros2 run ryugu_control gcs_bridge_node --ros-args -p jetson_ip:=192.168.1.10
"""

import math
import socket
import struct
import subprocess
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

# ── MAVROS message & service types ──────────────────────────────────────────
from mavros_msgs.msg import ManualControl, RCOut, State, Altitude
from mavros_msgs.srv import CommandBool, CommandLong, SetMode
from sensor_msgs.msg import BatteryState, Imu
from geometry_msgs.msg import PoseStamped

# ═══════════════════════════════════════════════════════════════════════════════
#  CRC-16/CCITT-FALSE  (poly=0x1021, init=0xFFFF, no reflection)
# ═══════════════════════════════════════════════════════════════════════════════
_CRC_POLY = 0x1021
_crc_table: list[int] = []
for _i in range(256):
    _crc = _i << 8
    for _ in range(8):
        if _crc & 0x8000:
            _crc = (_crc << 1) ^ _CRC_POLY
        else:
            _crc <<= 1
        _crc &= 0xFFFF
    _crc_table.append(_crc)


def crc16_ccitt(data: bytes) -> int:
    """Compute CRC-16/CCITT-FALSE over *data*."""
    crc = 0xFFFF
    for byte in data:
        idx = ((crc >> 8) ^ byte) & 0xFF
        crc = ((crc << 8) ^ _crc_table[idx]) & 0xFFFF
    return crc


# ═══════════════════════════════════════════════════════════════════════════════
#  Protocol constants
# ═══════════════════════════════════════════════════════════════════════════════
SYNC_WORD   = 0xAA55
SYNC_BYTES  = struct.pack('<H', SYNC_WORD)   # b'\x55\xAA' on wire
HEADER_SIZE = 5    # SYNC(2) + ID(1) + LEN(2)
CRC_SIZE    = 2
MAX_PAYLOAD = 1024

# ── GCS → Jetson command IDs ──
CMD_MOTION   = 0x81
CMD_MODE     = 0x82
CMD_GRIPPER  = 0x83
CMD_BALLAST  = 0x84
CMD_ARM      = 0x85
CMD_ESTOP    = 0x86

# ── Jetson → GCS telemetry IDs ──
TELEM_IMU    = 0x01
TELEM_DEPTH  = 0x02
TELEM_STATUS = 0x03
ACK          = 0xF0

# ── Mode mapping: protocol ID → ArduSub custom_mode string ──
MODE_MAP = {
    0: 'MANUAL',
    1: 'STABILIZE',
    2: 'ALT_HOLD',
    3: 'AUTO',
}

# ── Gripper constants ──
MAV_CMD_DO_SET_SERVO = 183
GRIPPER_SERVO_PIN    = 1       # MAIN 1 on Pixhawk
GRIPPER_PWM_STOP     = 1500
GRIPPER_PWM_OPEN     = 1900
GRIPPER_PWM_CLOSE    = 1100

# ── Thruster mapping: AUX 1–6 → mavros RC out channel indices (0-based) ──
#  On Pixhawk with ArduSub, AUX 1–6 are typically RC channels 9–14
#  (indices 8–13 in the 16-element channels array).
THRUSTER_START_IDX = 8   # AUX 1 = channel 9 = index 8
THRUSTER_COUNT     = 6

# ═══════════════════════════════════════════════════════════════════════════════
#  Packet builder / parser
# ═══════════════════════════════════════════════════════════════════════════════
def build_packet(pkt_id: int, payload: bytes = b'') -> bytes:
    """
    Build a complete wire packet:
      SYNC(2) + ID(1) + LEN(2) + PAYLOAD(N) + CRC16(2)

    CRC is computed over ID + LEN + PAYLOAD.
    """
    length = len(payload)
    header = struct.pack('<HBH', SYNC_WORD, pkt_id, length)
    crc_data = struct.pack('<BH', pkt_id, length) + payload
    crc = crc16_ccitt(crc_data)
    return header + payload + struct.pack('<H', crc)


def parse_packet(buf: bytearray) -> Optional[tuple[int, bytes, int]]:
    """
    Attempt to extract one valid packet from *buf*.

    Returns:
        (pkt_id, payload, bytes_consumed)  on success
        None                               if no complete packet yet

    On sync errors or CRC mismatch the buffer is advanced past bad bytes.
    """
    while True:
        if len(buf) < HEADER_SIZE + CRC_SIZE:
            return None

        # Locate SYNC word
        sync_pos = buf.find(SYNC_BYTES)
        if sync_pos < 0:
            # Keep last byte which could be partial sync start
            if len(buf) > 1:
                del buf[:len(buf) - 1]
            return None
        if sync_pos > 0:
            del buf[:sync_pos]

        if len(buf) < HEADER_SIZE:
            return None

        _, pkt_id, length = struct.unpack_from('<HBH', buf, 0)

        if length > MAX_PAYLOAD:
            del buf[:2]       # skip bad sync, keep searching
            continue

        total = HEADER_SIZE + length + CRC_SIZE
        if len(buf) < total:
            return None       # incomplete packet

        payload = bytes(buf[HEADER_SIZE:HEADER_SIZE + length])

        # Validate CRC over ID + LEN + PAYLOAD
        crc_data = struct.pack('<BH', pkt_id, length) + payload
        crc_expected = crc16_ccitt(crc_data)
        crc_received = struct.unpack_from('<H', buf, HEADER_SIZE + length)[0]

        if crc_received != crc_expected:
            del buf[:2]       # CRC mismatch → skip sync
            continue

        del buf[:total]
        return (pkt_id, payload, total)


# ═══════════════════════════════════════════════════════════════════════════════
#  Quaternion → Euler angles  (intrinsic ZYX / Tait-Bryan)
# ═══════════════════════════════════════════════════════════════════════════════
def quaternion_to_euler_deg(qx: float, qy: float, qz: float, qw: float):
    """
    Convert a quaternion to Tait-Bryan Euler angles (roll, pitch, yaw) in degrees.

    Uses the intrinsic ZYX convention consistent with ROS/MAVROS.
    Returns (roll_deg, pitch_deg, yaw_deg).
    """
    # Roll  (x-axis rotation)
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw   (z-axis rotation)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))


# ═══════════════════════════════════════════════════════════════════════════════
#  GCSBridgeNode
# ═══════════════════════════════════════════════════════════════════════════════
class GCSBridgeNode(Node):
    """
    Unified ROS2 bridge node for the RYUGU ROV.

    Uplink:   Receives binary UDP packets from the GCS, validates CRC-16,
              and translates them into MAVROS topic publications and service calls.

    Downlink: Subscribes to MAVROS telemetry topics, packs them into binary
              UDP packets, and transmits them to the GCS at 20 Hz.
    """

    def __init__(self):
        super().__init__('gcs_bridge_node')

        # ── Parameters ─────────────────────────────────────────────────
        self.declare_parameter('jetson_ip', '192.168.1.10')
        self.declare_parameter('gcs_ip', '192.168.1.100')
        self.declare_parameter('cmd_port', 5001)
        self.declare_parameter('telem_port', 5002)
        self.declare_parameter('manual_control_rate', 10.0)
        self.declare_parameter('telemetry_rate', 20.0)

        self._jetson_ip = self.get_parameter('jetson_ip').get_parameter_value().string_value
        self._gcs_ip    = self.get_parameter('gcs_ip').get_parameter_value().string_value
        self._cmd_port  = self.get_parameter('cmd_port').get_parameter_value().integer_value
        self._telem_port = self.get_parameter('telem_port').get_parameter_value().integer_value
        self._manual_rate = self.get_parameter('manual_control_rate').get_parameter_value().double_value
        self._telem_rate  = self.get_parameter('telemetry_rate').get_parameter_value().double_value

        # ── Thread-safe shared state ───────────────────────────────────
        self._state_lock = threading.Lock()

        # Latest received motion command (surge, sway, heave, yaw, pitch, roll)
        self._latest_motion: tuple[int, ...] = (0, 0, 0, 0, 0, 0)

        # Latest MAVROS telemetry values
        self._roll: float  = 0.0
        self._pitch: float = 0.0
        self._yaw: float   = 0.0
        self._depth_m: float   = 0.0
        self._altitude_m: float = 0.0
        self._battery_v: float  = 0.0
        self._pose_received: bool = False  # True once EKF pose arrives (requires GPS)
        self._arm_state: bool   = False
        self._mode_id: int      = 0
        self._rc_channels: list[int] = [0] * 16

        # ── UDP socket ─────────────────────────────────────────────────
        self._sock: Optional[socket.socket] = None
        self._rx_buf = bytearray()
        self._receiver_thread: Optional[threading.Thread] = None
        self._running = False

        # Statistics
        self._rx_packet_count = 0
        self._tx_packet_count = 0
        self._crc_error_count = 0

        # Motion logging throttle (avoid CLI flood at 10–20 Hz input rate)
        self._last_motion_log_time: float = 0.0

        # ── Callback group (reentrant for multi-threaded executor) ─────
        self._cb_group = ReentrantCallbackGroup()

        # ── Publishers ─────────────────────────────────────────────────
        self._manual_pub = self.create_publisher(
            ManualControl, '/mavros/manual_control/send', 10)

        # ── Service clients ────────────────────────────────────────────
        self._arming_cli  = self.create_client(CommandBool, '/mavros/cmd/arming')
        self._setmode_cli = self.create_client(SetMode, '/mavros/set_mode')
        self._command_cli = self.create_client(CommandLong, '/mavros/cmd/command')

        # ── Subscribers ────────────────────────────────────────────────
        # Pose (EKF-fused orientation for TELEM_IMU)
        self._pose_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose',
            self._pose_callback, qos_profile_sensor_data,
            callback_group=self._cb_group)

        # IMU data (fallback for orientation)
        self._imu_sub = self.create_subscription(
            Imu, '/mavros/imu/data',
            self._imu_callback, qos_profile_sensor_data, callback_group=self._cb_group)

        # Altitude / depth
        self._alt_sub = self.create_subscription(
            Altitude, '/mavros/altitude',
            self._altitude_callback, qos_profile_sensor_data,
            callback_group=self._cb_group)

        # Battery
        self._batt_sub = self.create_subscription(
            BatteryState, '/mavros/battery',
            self._battery_callback, qos_profile_sensor_data,
            callback_group=self._cb_group)

        # Vehicle state (arm, mode)
        self._state_sub = self.create_subscription(
            State, '/mavros/state',
            self._state_callback, 10, callback_group=self._cb_group)

        # RC outputs (thruster PWM channels)
        self._rcout_sub = self.create_subscription(
            RCOut, '/mavros/rc/out',
            self._rcout_callback, 10, callback_group=self._cb_group)

        # ── Timers ─────────────────────────────────────────────────────
        self._manual_timer = self.create_timer(
            1.0 / self._manual_rate, self._publish_manual_control,
            callback_group=self._cb_group)

        self._telem_timer = self.create_timer(
            1.0 / self._telem_rate, self._publish_telemetry,
            callback_group=self._cb_group)

        # ── Initialise socket & start receiver thread ──────────────────
        self._init_socket()
        self._start_receiver()

        self.get_logger().info(
            f'GCSBridgeNode started — '
            f'listening on {self._jetson_ip}:{self._cmd_port}, '
            f'sending to {self._gcs_ip}:{self._telem_port}')

    # ═══════════════════════════════════════════════════════════════════════
    #  Socket setup
    # ═══════════════════════════════════════════════════════════════════════
    def _init_socket(self):
        """Create and bind the UDP command socket."""
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._sock.bind((self._jetson_ip, self._cmd_port))
            self._sock.setblocking(False)
            self._sock.settimeout(0.05)  # 50 ms recv timeout
            self.get_logger().info(
                f'UDP socket bound to {self._jetson_ip}:{self._cmd_port}')
        except OSError as e:
            self.get_logger().error(
                f'Failed to bind UDP socket to '
                f'{self._jetson_ip}:{self._cmd_port} — {e}')
            self._sock = None

    # ═══════════════════════════════════════════════════════════════════════
    #  Transmit helper
    # ═══════════════════════════════════════════════════════════════════════
    def _send_packet(self, pkt_id: int, payload: bytes = b''):
        """Build and send a packet to the GCS via UDP."""
        if self._sock is None:
            return
        pkt = build_packet(pkt_id, payload)
        try:
            self._sock.sendto(pkt, (self._gcs_ip, self._telem_port))
            self._tx_packet_count += 1
        except OSError:
            pass  # GCS may not be listening yet

    def _send_ack(self, acked_cmd_id: int):
        """Send an ACK packet acknowledging a command."""
        payload = struct.pack('<B', acked_cmd_id)
        self._send_packet(ACK, payload)
        self.get_logger().debug(f'ACK sent for cmd 0x{acked_cmd_id:02X}')

    # ═══════════════════════════════════════════════════════════════════════
    #  Receiver thread  (Uplink: GCS → MAVROS)
    # ═══════════════════════════════════════════════════════════════════════
    def _start_receiver(self):
        """Launch the background UDP receiver thread."""
        if self._sock is None:
            self.get_logger().warn('Receiver thread not started — no socket')
            return
        self._running = True
        self._receiver_thread = threading.Thread(
            target=self._receiver_loop, name='gcs-udp-rx', daemon=True)
        self._receiver_thread.start()
        self.get_logger().info('UDP receiver thread started')

    def _receiver_loop(self):
        """Continuously read from the UDP socket and dispatch packets."""
        while self._running and rclpy.ok():
            try:
                data, _addr = self._sock.recvfrom(4096)
                self._rx_buf.extend(data)
            except (socket.timeout, BlockingIOError):
                pass

            # Parse all complete packets in buffer
            while True:
                result = parse_packet(self._rx_buf)
                if result is None:
                    break
                pkt_id, payload, _consumed = result
                self._rx_packet_count += 1
                self._dispatch_command(pkt_id, payload)

            # Avoid busy-waiting when no data
            if not self._rx_buf:
                time.sleep(0.001)

    def _dispatch_command(self, pkt_id: int, payload: bytes):
        """Route a received packet to its handler."""
        if pkt_id == CMD_MOTION:
            self._handle_motion(payload)
        elif pkt_id == CMD_MODE:
            self._handle_mode(payload)
        elif pkt_id == CMD_GRIPPER:
            self._handle_gripper(payload)
        elif pkt_id == CMD_BALLAST:
            self._handle_ballast(payload)
        elif pkt_id == CMD_ARM:
            self._handle_arm(payload)
        elif pkt_id == CMD_ESTOP:
            self._handle_estop()
        else:
            self.get_logger().debug(
                f'Unknown packet ID: 0x{pkt_id:02X} (len={len(payload)})')

    # ── Motion handler ───────────────────────────────────────────────────
    def _handle_motion(self, payload: bytes):
        """CMD_MOTION (0x81): payload = <6h (surge, sway, heave, yaw, pitch, roll)."""
        if len(payload) != 12:
            self.get_logger().warn(
                f'Invalid CMD_MOTION payload length: {len(payload)} (expected 12)')
            return
        values = struct.unpack('<6h', payload)
        with self._state_lock:
            self._latest_motion = values

        # ── Throttled motion logging ──────────────────────────────────
        # Only log when the pilot is actively commanding (at least one
        # axis non-zero), and at most once every 0.5 s to avoid flooding
        # the terminal at 10–20 Hz input rates.
        surge, sway, heave, yaw, pitch, roll = values
        if any(v != 0 for v in values):
            now = time.monotonic()
            if now - self._last_motion_log_time >= 0.5:
                self._last_motion_log_time = now
                self.get_logger().info(
                    f'🎮 Motion: '
                    f'surge={surge:+5d}  sway={sway:+5d}  heave={heave:+5d}  '
                    f'yaw={yaw:+5d}  pitch={pitch:+5d}  roll={roll:+5d}'
                )

    # ── Mode handler ─────────────────────────────────────────────────────
    def _handle_mode(self, payload: bytes):
        """CMD_MODE (0x82): payload = <B (mode_id).  Calls /mavros/set_mode."""
        if len(payload) != 1:
            self.get_logger().warn(
                f'Invalid CMD_MODE payload length: {len(payload)} (expected 1)')
            return
        mode_id = struct.unpack('<B', payload)[0]
        mode_str = MODE_MAP.get(mode_id)
        if mode_str is None:
            self.get_logger().warn(f'Unknown mode ID: {mode_id}')
            return

        self.get_logger().info(f'Mode change requested: {mode_id} → "{mode_str}"')

        if not self._setmode_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('SetMode service not available')
            return

        req = SetMode.Request()
        req.custom_mode = mode_str
        # base_mode = 0 means we use custom_mode (ArduSub convention)
        req.base_mode = 0

        future = self._setmode_cli.call_async(req)
        # Wait synchronously in the receiver thread
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)

        if future.result() is not None and future.result().mode_sent:
            self.get_logger().info(f'Mode set to "{mode_str}" — sending ACK')
            self._send_ack(CMD_MODE)
        else:
            self.get_logger().error(f'Failed to set mode "{mode_str}"')

    # ── Gripper handler ──────────────────────────────────────────────────
    def _handle_gripper(self, payload: bytes):
        """CMD_GRIPPER (0x83): payload = <B (0=stop, 1=open, 2=close).
        Calls /mavros/cmd/command with MAV_CMD_DO_SET_SERVO."""
        if len(payload) != 1:
            self.get_logger().warn(
                f'Invalid CMD_GRIPPER payload length: {len(payload)} (expected 1)')
            return
        state = struct.unpack('<B', payload)[0]

        pwm_map = {
            0: GRIPPER_PWM_STOP,
            1: GRIPPER_PWM_OPEN,
            2: GRIPPER_PWM_CLOSE,
        }
        pwm = pwm_map.get(state)
        if pwm is None:
            self.get_logger().warn(f'Unknown gripper state: {state}')
            return

        state_names = {0: 'STOP', 1: 'OPEN', 2: 'CLOSE'}
        self.get_logger().info(
            f'Gripper: {state_names.get(state, "?")} → PWM {pwm} µs on servo {GRIPPER_SERVO_PIN}')

        if not self._command_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('CommandLong service not available')
            return

        req = CommandLong.Request()
        req.broadcast = False
        req.command   = MAV_CMD_DO_SET_SERVO
        req.confirmation = 0
        req.param1 = float(GRIPPER_SERVO_PIN)   # servo number
        req.param2 = float(pwm)                  # PWM in microseconds
        req.param3 = 0.0
        req.param4 = 0.0
        req.param5 = 0.0
        req.param6 = 0.0
        req.param7 = 0.0

        future = self._command_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)

        if future.result() is not None and future.result().success:
            self.get_logger().info('Gripper command accepted')
        else:
            self.get_logger().error('Gripper command failed')

    # ── Ballast handler ──────────────────────────────────────────────────
    def _handle_ballast(self, payload: bytes):
        """CMD_BALLAST (0x84): payload = <B.  Parsed but ignored (no hardware)."""
        if len(payload) >= 1:
            state = struct.unpack('<B', payload[:1])[0]
            self.get_logger().debug(
                f'Ballast command received (state={state}) — no hardware, ignored')
        else:
            self.get_logger().debug('Ballast command received — ignored (no hardware)')

    # ── Arm handler ──────────────────────────────────────────────────────
    def _handle_arm(self, payload: bytes):
        """CMD_ARM (0x85): payload = <B (1=ARM, 0=DISARM).  Calls /mavros/cmd/arming."""
        if len(payload) != 1:
            self.get_logger().warn(
                f'Invalid CMD_ARM payload length: {len(payload)} (expected 1)')
            return
        arm_val = struct.unpack('<B', payload)[0]
        arm = bool(arm_val)

        label = 'ARM' if arm else 'DISARM'
        self.get_logger().info(f'{label} requested')

        if not self._arming_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('Arming service not available')
            return

        req = CommandBool.Request()
        req.value = arm

        future = self._arming_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)

        if future.result() is not None and future.result().success:
            # ── Prominent ARM / DISARM confirmation ──────────────────
            if arm:
                self.get_logger().info(
                    '\n'
                    '  \033[1;32m╔══════════════════════════════════════════════╗\033[0m\n'
                    '  \033[1;32m║   █████╗ ██████╗ ███╗   ███╗███████╗██████╗  ║\033[0m\n'
                    '  \033[1;32m║  ██╔══██╗██╔══██╗████╗ ████║██╔════╝██╔══██╗ ║\033[0m\n'
                    '  \033[1;32m║  ███████║██████╔╝██╔████╔██║█████╗  ██║  ██║ ║\033[0m\n'
                    '  \033[1;32m║  ██╔══██║██╔══██╗██║╚██╔╝██║██╔══╝  ██║  ██║ ║\033[0m\n'
                    '  \033[1;32m║  ██║  ██║██║  ██║██║ ╚═╝ ██║███████╗██████╔╝ ║\033[0m\n'
                    '  \033[1;32m║  ╚═╝  ╚═╝╚═╝  ╚═╝╚═╝     ╚═╝╚══════╝╚═════╝  ║\033[0m\n'
                    '  \033[1;32m║         ROV IS ARMED — THRUSTERS LIVE        ║\033[0m\n'
                    '  \033[1;32m╚══════════════════════════════════════════════╝\033[0m'
                )
            else:
                self.get_logger().info(
                    '\n'
                    '  \033[1;33m╔══════════════════════════════════════════════════════╗\033[0m\n'
                    '  \033[1;33m║     ██████╗ ██╗███████╗ █████╗ ██████╗ ███╗   ███╗   ║\033[0m\n'
                    '  \033[1;33m║     ██╔══██╗██║██╔════╝██╔══██╗██╔══██╗████╗ ████║   ║\033[0m\n'
                    '  \033[1;33m║     ██║  ██║██║███████╗███████║██████╔╝██╔████╔██║   ║\033[0m\n'
                    '  \033[1;33m║     ██║  ██║██║╚════██║██╔══██║██╔══██╗██║╚██╔╝██║   ║\033[0m\n'
                    '  \033[1;33m║     ██████╔╝██║███████║██║  ██║██║  ██║██║ ╚═╝ ██║   ║\033[0m\n'
                    '  \033[1;33m║     ╚═════╝ ╚═╝╚══════╝╚═╝  ╚═╝╚═╝  ╚═╝╚═╝     ╚═╝   ║\033[0m\n'
                    '  \033[1;33m║           ROV DISARMED — THRUSTERS SAFE              ║\033[0m\n'
                    '  \033[1;33m╚══════════════════════════════════════════════════════╝\033[0m'
                )
            self._send_ack(CMD_ARM)
        else:
            self.get_logger().error(f'{label} failed — '
                                    f'result: {future.result()}')

    # ── E-STOP handler ───────────────────────────────────────────────────
    def _handle_estop(self):
        """
        CMD_ESTOP (0x86): empty payload.

        Emergency stop sequence:
          1. Disarm the Pixhawk immediately via the arming service.
          2. Send an ACK back to the GCS so the operator knows the command
             was processed.
          3. Sleep briefly to allow the UDP ACK packet to flush through the
             network stack before the interface goes down.
          4. Power off the Jetson Orin Nano via ``sudo poweroff``.

        .. important::
           The Linux user running this ROS2 node **must** have passwordless
           sudo permissions for ``/usr/sbin/poweroff``.  Add the following
           line to ``/etc/sudoers`` (via ``visudo``)::

               <username> ALL=(ALL) NOPASSWD: /usr/sbin/poweroff

           Replace ``<username>`` with the actual user account (e.g.
           ``icad`` or ``jetson``).
        """
        self.get_logger().warn('⚠ E-STOP TRIGGERED — disarming immediately!')

        # Step 1: Disarm the Pixhawk -------------------------------------------
        if not self._arming_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('E-STOP: arming service not available!')
        else:
            req = CommandBool.Request()
            req.value = False

            future = self._arming_cli.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)

            if future.result() is not None and future.result().success:
                self.get_logger().info('E-STOP: disarm successful')
            else:
                self.get_logger().error(
                    'E-STOP: disarm FAILED — check flight controller connection!')

        # Step 2: Acknowledge the E-STOP command to the GCS --------------------
        self._send_ack(CMD_ESTOP)
        self.get_logger().info('E-STOP: ACK sent to GCS')

        # Step 3: Flush the UDP packet before the network goes down ------------
        time.sleep(0.5)

        # Step 4: Power off the Jetson Orin Nano ------------------------------
        self.get_logger().warn('E-STOP: shutting down Jetson NOW!')
        try:
            subprocess.run(
                ['sudo', 'poweroff'],
                check=False,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except FileNotFoundError:
            self.get_logger().fatal(
                'E-STOP: "poweroff" not found — '
                'Jetson shutdown FAILED!  Power off manually.')

    # ═══════════════════════════════════════════════════════════════════════
    #  MAVROS subscriber callbacks
    # ═══════════════════════════════════════════════════════════════════════
    def _pose_callback(self, msg: PoseStamped):
        """Store EKF-fused orientation for TELEM_IMU."""
        q = msg.pose.orientation
        roll, pitch, yaw = quaternion_to_euler_deg(q.x, q.y, q.z, q.w)
        with self._state_lock:
            self._pose_received = True
            self._roll  = roll
            self._pitch = pitch
            self._yaw   = yaw

    def _imu_callback(self, msg: Imu):
        """
        Fallback IMU orientation — used only when EKF pose is unavailable
        (e.g. indoors without GPS).  Once _pose_callback has received a
        valid EKF estimate it sets _pose_received and IMU updates stop.
        """
        q = msg.orientation
        roll, pitch, yaw = quaternion_to_euler_deg(q.x, q.y, q.z, q.w)
        with self._state_lock:
            if not self._pose_received:
                self._roll  = roll
                self._pitch = pitch
                self._yaw   = yaw

    def _altitude_callback(self, msg: Altitude):
        """
        Store altitude/depth for TELEM_DEPTH.

        ArduSub reports:
          - altitude.relative (m) = negative depth (i.e. -depth)
          - We treat relative altitude as:  depth_m = -altitude.relative
          - For display: altitude_m = 1.5 - depth_m  (above seabed assumption)
        """
        # In ArduSub, relative altitude is negative when submerged
        rel_alt = msg.relative
        depth = -rel_alt
        altitude = max(0.0, 1.5 - depth)
        with self._state_lock:
            self._depth_m = depth
            self._altitude_m = altitude

    def _battery_callback(self, msg: BatteryState):
        """Store battery voltage for TELEM_STATUS."""
        with self._state_lock:
            self._battery_v = msg.voltage

    def _state_callback(self, msg: State):
        """Store arm state and mode for TELEM_STATUS."""
        # Map ArduSub mode string back to protocol mode ID
        mode_str = msg.mode.upper()
        mode_id = 0  # default MANUAL
        for mid, mstr in MODE_MAP.items():
            if mstr == mode_str:
                mode_id = mid
                break

        with self._state_lock:
            self._arm_state = msg.armed
            self._mode_id = mode_id

    def _rcout_callback(self, msg: RCOut):
        """Store RC output channel values for thruster feedback."""
        with self._state_lock:
            self._rc_channels = list(msg.channels)

    # ═══════════════════════════════════════════════════════════════════════
    #  Timer callbacks  (publish to MAVROS / send to GCS)
    # ═══════════════════════════════════════════════════════════════════════
    def _publish_manual_control(self):
        """
        Publish the latest motion command to /mavros/manual_control/send at
        the configured rate (default 10 Hz).

        ManualControl fields (ArduSub convention):
          x → surge   (forward/back)
          y → sway    (left/right)
          z → heave   (up/down, positive = up in ArduSub)
          r → yaw
          Aux buttons are unused here — yaw/pitch/roll mapped to axes.
        """
        with self._state_lock:
            surge, sway, heave, yaw, pitch, roll = self._latest_motion

        msg = ManualControl()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.x   = float(surge)   # surge
        msg.y   = float(sway)    # sway
        msg.z   = float(heave)   # heave (ArduSub: positive = up)
        msg.r   = float(yaw)     # yaw
        # Pitch and roll are not directly supported by ManualControl's standard
        # axes; ArduSub reads buttons for extra axes.  We pack pitch/roll into
        # the button field as a bitmask (ArduSub-specific extension).
        #   buttons = (pitch_clamped << 16) | (roll_clamped & 0xFFFF)
        pitch_clamped = max(-1000, min(1000, pitch))
        roll_clamped  = max(-1000, min(1000, roll))
        msg.buttons = ((pitch_clamped & 0xFFFF) << 16) | (roll_clamped & 0xFFFF)

        self._manual_pub.publish(msg)

    def _publish_telemetry(self):
        """
        Assemble and transmit TELEM_IMU, TELEM_DEPTH, and TELEM_STATUS packets
        to the GCS at the configured rate (default 20 Hz).
        """
        if self._sock is None:
            return

        with self._state_lock:
            roll       = self._roll
            pitch      = self._pitch
            yaw        = self._yaw
            depth_m    = self._depth_m
            altitude_m = self._altitude_m
            battery_v  = self._battery_v
            arm_state  = self._arm_state
            mode_id    = self._mode_id
            rc_channels = list(self._rc_channels)

        # ── TELEM_IMU (0x01): pitch, roll, yaw (°) as 3× float32 ──
        imu_payload = struct.pack('<3f', pitch, roll, yaw)
        self._send_packet(TELEM_IMU, imu_payload)

        # ── TELEM_DEPTH (0x02): depth_m, altitude_m as 2× float32 ──
        depth_payload = struct.pack('<2f', depth_m, altitude_m)
        self._send_packet(TELEM_DEPTH, depth_payload)

        # ── TELEM_STATUS (0x03): battery_v(f), arm_state(B), mode_id(B), 8× thruster(B) ──
        # Extract AUX 1–6 from RC channels (indices 8–13), scale to 0–255
        thruster_bytes = []
        for i in range(THRUSTER_COUNT):
            idx = THRUSTER_START_IDX + i
            if idx < len(rc_channels):
                # ArduSub RC output is typically 1100–1900 µs; scale to 0–255
                raw = rc_channels[idx]
                scaled = max(0, min(255, int((raw - 1100) / 800.0 * 255.0)))
            else:
                scaled = 0
            thruster_bytes.append(scaled)

        # Pad to 8 bytes (we only have 6 thrusters; last 2 are reserved)
        while len(thruster_bytes) < 8:
            thruster_bytes.append(0)

        arm_byte = 1 if arm_state else 0
        status_payload = struct.pack(
            '<fBB8B',
            battery_v,
            arm_byte,
            mode_id,
            *thruster_bytes,
        )
        self._send_packet(TELEM_STATUS, status_payload)

    # ═══════════════════════════════════════════════════════════════════════
    #  Lifecycle
    # ═══════════════════════════════════════════════════════════════════════
    def destroy_node(self):
        """Clean shutdown: stop receiver thread, close socket."""
        self._running = False
        if self._receiver_thread is not None:
            self._receiver_thread.join(timeout=3.0)
        if self._sock is not None:
            self._sock.close()
            self._sock = None
        self.get_logger().info(
            f'GCSBridgeNode stopped — '
            f'RX packets: {self._rx_packet_count}, '
            f'TX packets: {self._tx_packet_count}')
        super().destroy_node()


# ═══════════════════════════════════════════════════════════════════════════════
#  Entry point
# ═══════════════════════════════════════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)
    node = GCSBridgeNode()

    # Use a MultiThreadedExecutor so that timers, subscribers, and the
    # receiver thread can all run concurrently without blocking each other.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
