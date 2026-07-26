#!/usr/bin/env python3
"""
gcs_bridge_node.py — Telemetry-Only GCS ↔ Jetson ↔ Pixhawk Communication Bridge

This ROS2 node manages the DOWNLINK-ONLY UDP telemetry link between the
Operator GCS Laptop and the Pixhawk flight controller via MAVROS.

**HYBRID CONTROL ARCHITECTURE (Refactored 2026-07-27):**

  ┌─────────────────────────────────────────────────────────────────────┐
  │  CONTROL PATH (Uplink):  Handled 100% by QGroundControl v5.0+       │
  │                                                                     │
  │  GCS Laptop (QGC) ──UDP :14550──▶ mavlink-router ──USB Serial──▶ Pixhawk │
  │                                                                     │
  │  • Joystick MANUAL_CONTROL messages                                 │
  │  • Arming / Disarming                                               │
  │  • Flight Mode switching                                            │
  │  • Parameter tuning                                                 │
  │  • All MAVLink command packets (sysid = GCS)                        │
  └─────────────────────────────────────────────────────────────────────┘

  ┌─────────────────────────────────────────────────────────────────────┐
  │  TELEMETRY PATH (Downlink):  ROS 2 gcs_bridge_node (THIS NODE)      │
  │                                                                     │
  │  Pixhawk ──USB Serial──▶ MAVROS ──ROS2 topics──▶ gcs_bridge_node    │
  │                                                     │               │
  │                                          Binary UDP :5002           │
  │                                                     │               │
  │                                               PyQt5 GCS             │
  │                                                                     │
  │  • TELEM_IMU    (0x01): pitch, roll, yaw (°)          @ 20 Hz      │
  │  • TELEM_DEPTH  (0x02): depth_m, altitude_m           @ 20 Hz      │
  │  • TELEM_STATUS (0x03): battery, arm, mode, thrusters @ 20 Hz      │
  │  • TELEM_QR     (0x04): QR code string (event-driven)               │
  └─────────────────────────────────────────────────────────────────────┘

  ┌─────────────────────────────────────────────────────────────────────┐
  │  VIDEO PATH (Independent): webcam_streamer node                     │
  │                                                                     │
  │  USB Webcams ──▶ webcam_streamer ──HTTP MJPEG :8554/:8555──▶ GCS   │
  └─────────────────────────────────────────────────────────────────────┘

**CRITICAL DESIGN CONSTRAINT:**
  This node MUST NOT publish any ManualControl, OverrideRCIn, or invoke any
  MAVROS service calls (arming, set_mode, command).  Doing so would cause
  MAVLink packet collisions (sysid mismatch / lockout) with QGroundControl
  on the shared MAVLink bus.  All command execution is disabled by design.

Downlink (Pixhawk → GCS):
  - TELEM_IMU    (0x01): pitch, roll, yaw (°)               @ 20 Hz
  - TELEM_DEPTH  (0x02): depth_m, altitude_m                @ 20 Hz
  - TELEM_STATUS (0x03): battery, arm, mode, thrusters      @ 20 Hz
  - TELEM_QR     (0x04): QR code string (on-detection)      event-driven, throttled to 1 Hz

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
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

# ── MAVROS message types (telemetry-only subscriptions) ──────────────────
# NOTE: ManualControl, OverrideRCIn, CommandBool, CommandLong, and SetMode
#       imports REMOVED — this node operates in TELEMETRY-ONLY mode.
#       All MAVLink command packets are handled exclusively by QGroundControl.
from mavros_msgs.msg import RCOut, State, Altitude
from sensor_msgs.msg import BatteryState, Imu
from std_msgs.msg import String

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

# ── GCS → Jetson command IDs (received but NOT executed in telemetry-only mode) ──
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
TELEM_QR     = 0x04
ACK          = 0xF0

# ── Mode mapping: protocol ID → ArduSub custom_mode string (telemetry use only) ──
MODE_MAP = {
    0: '0',  # MANUAL
    1: '1',  # STABILIZE
    2: '2',  # ALT_HOLD
    3: '3',  # AUTO
}

# ── Gripper constants (kept for reference; not executed in telemetry-only mode) ──
# MAV_CMD_DO_SET_SERVO = 183
# GRIPPER_SERVO_PIN    = 1       # MAIN 1 on Pixhawk
# GRIPPER_PWM_STOP     = 1500
# GRIPPER_PWM_OPEN     = 1900
# GRIPPER_PWM_CLOSE    = 1100

# ── Thruster mapping: AUX 1–6 → mavros RC out channel indices (0-based) ──
#  On Pixhawk with ArduSub, AUX 1–6 are typically RC channels 9–14
#  (indices 8–13 in the 16-element channels array).
THRUSTER_START_IDX = 8   # AUX 1 = channel 9 = index 8
THRUSTER_COUNT     = 6

# ── Command name mapping for log messages ────────────────────────────────
_CMD_NAMES = {
    0x81: 'CMD_MOTION',
    0x82: 'CMD_MODE',
    0x83: 'CMD_GRIPPER',
    0x84: 'CMD_BALLAST',
    0x85: 'CMD_ARM',
    0x86: 'CMD_ESTOP',
}

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
#  GCSBridgeNode  (Telemetry-Only Mode)
# ═══════════════════════════════════════════════════════════════════════════════
class GCSBridgeNode(Node):
    """
    Telemetry-only ROS2 bridge node for the RYUGU ROV.

    **Uplink: DISABLED.**
       Incoming binary UDP packets from the GCS are received and CRC-validated,
       but their commands are NOT executed.  A warning is logged for each
       received command indicating that the control path is handled exclusively
       by QGroundControl on the MAVLink bus.

       This prevents sysid mismatch / lockout collisions between this node
       and QGroundControl when both would otherwise attempt to publish
       MANUAL_CONTROL or invoke arming/set_mode service calls.

    **Downlink: ACTIVE.**
       Subscribes to MAVROS telemetry topics, packs them into binary UDP
       packets (TELEM_IMU, TELEM_DEPTH, TELEM_STATUS, TELEM_QR), and
       transmits them to the custom PyQt5 GCS at the configured rate (20 Hz).
    """

    def __init__(self):
        super().__init__('gcs_bridge_node')

        # ── Parameters ─────────────────────────────────────────────────
        self.declare_parameter('jetson_ip', '192.168.1.10')
        self.declare_parameter('gcs_ip', '192.168.1.100')
        self.declare_parameter('cmd_port', 5001)
        self.declare_parameter('telem_port', 5002)
        self.declare_parameter('telemetry_rate', 20.0)

        self._jetson_ip = self.get_parameter('jetson_ip').get_parameter_value().string_value
        self._gcs_ip    = self.get_parameter('gcs_ip').get_parameter_value().string_value
        self._cmd_port  = self.get_parameter('cmd_port').get_parameter_value().integer_value
        self._telem_port = self.get_parameter('telem_port').get_parameter_value().integer_value
        self._telem_rate  = self.get_parameter('telemetry_rate').get_parameter_value().double_value

        # ── Thread-safe shared state ───────────────────────────────────
        self._state_lock = threading.Lock()

        # Latest MAVROS telemetry values
        self._roll: float  = 0.0
        self._pitch: float = 0.0
        self._yaw: float   = 0.0
        self._depth_m: float   = 0.0
        self._altitude_m: float = 0.0
        self._battery_v: float  = 0.0
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

        # QR telemetry throttling — avoid flooding GCS with duplicate data
        self._last_qr_sent_data: str = ''
        self._last_qr_send_time: float = 0.0

        # ── Callback group (reentrant for multi-threaded executor) ─────
        self._cb_group = ReentrantCallbackGroup()

        # ═══════════════════════════════════════════════════════════════════
        #  PUBLISHERS — DISABLED (Control Path Shutdown)
        # ═══════════════════════════════════════════════════════════════════
        #
        # ManualControl and OverrideRCIn publishers are intentionally
        # removed.  All MAVLink command packets (MANUAL_CONTROL, arming,
        # mode switching) are handled exclusively by QGroundControl.
        #
        # Publishing from this node would cause a sysid mismatch on the
        # MAVLink bus, leading to lockout where the Pixhawk alternates
        # between two control sources and becomes unresponsive.
        #
        # self._manual_pub = self.create_publisher(
        #     ManualControl, '/mavros/manual_control/send', 10)
        # self._rc_override_pub = self.create_publisher(
        #     OverrideRCIn, '/mavros/rc/override', 10)

        # ═══════════════════════════════════════════════════════════════════
        #  SERVICE CLIENTS — DISABLED (Control Path Shutdown)
        # ═══════════════════════════════════════════════════════════════════
        #
        # Arming, SetMode, and CommandLong service clients are intentionally
        # removed.  QGroundControl owns all MAVLink service calls.
        #
        # self._arming_cli  = self.create_client(CommandBool, '/mavros/cmd/arming')
        # self._setmode_cli = self.create_client(SetMode, '/mavros/set_mode')
        # self._command_cli = self.create_client(CommandLong, '/mavros/cmd/command')

        # ═══════════════════════════════════════════════════════════════════
        #  SUBSCRIBERS — ACTIVE (Telemetry Downlink)
        # ═══════════════════════════════════════════════════════════════════
        # All MAVROS telemetry subscriptions are preserved and fully active.

        # IMU data (primary source for orientation — always active, GPS-independent)
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

        # QR code data from webcam_streamer (per-camera topics)
        self._qr_front_sub = self.create_subscription(
            String, '/ryugu/qr/front',
            lambda msg: self._handle_qr_callback(msg.data, 0),
            10, callback_group=self._cb_group)
        self._qr_bottom_sub = self.create_subscription(
            String, '/ryugu/qr/bottom',
            lambda msg: self._handle_qr_callback(msg.data, 1),
            10, callback_group=self._cb_group)

        # ═══════════════════════════════════════════════════════════════════
        #  TIMERS — TELEMETRY ONLY (Manual Control Timer DISABLED)
        # ═══════════════════════════════════════════════════════════════════
        #
        # The manual control timer and its callback are removed to prevent
        # any MANUAL_CONTROL MAVLink messages from this node.
        #
        # self._manual_timer = self.create_timer(
        #     1.0 / self._manual_rate, self._publish_manual_control,
        #     callback_group=self._cb_group)

        # Telemetry broadcast timer — ACTIVE
        self._telem_timer = self.create_timer(
            1.0 / self._telem_rate, self._publish_telemetry,
            callback_group=self._cb_group)

        # ── Initialise socket & start receiver thread ──────────────────
        self._init_socket()
        self._start_receiver()

        self.get_logger().info(
            f'GCSBridgeNode started in TELEMETRY-ONLY mode — '
            f'listening on {self._jetson_ip}:{self._cmd_port} (commands DISABLED), '
            f'sending to {self._gcs_ip}:{self._telem_port} (telemetry ACTIVE)')

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
        """Build and send a telemetry packet to the GCS via UDP."""
        if self._sock is None:
            return
        pkt = build_packet(pkt_id, payload)
        try:
            self._sock.sendto(pkt, (self._gcs_ip, self._telem_port))
            self._tx_packet_count += 1
        except OSError:
            pass  # GCS may not be listening yet

    # ═══════════════════════════════════════════════════════════════════════
    #  Receiver thread  (Uplink: GCS → Jetson — COMMANDS DISABLED)
    # ═══════════════════════════════════════════════════════════════════════
    def _start_receiver(self):
        """
        Launch the background UDP receiver thread.

        The receiver still accepts and CRC-validates incoming packets so
        that the GCS can verify the UDP link is operational.  However, all
        received commands are logged and discarded — no MAVROS publications
        or service calls are made.
        """
        if self._sock is None:
            self.get_logger().warn('Receiver thread not started — no socket')
            return
        self._running = True
        self._receiver_thread = threading.Thread(
            target=self._receiver_loop, name='gcs-udp-rx', daemon=True)
        self._receiver_thread.start()
        self.get_logger().info('UDP receiver thread started (command execution DISABLED)')

    def _receiver_loop(self):
        """Continuously read from the UDP socket, validate, and discard commands."""
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
                # ── ALL commands are logged and discarded ───────────────
                self._log_disabled_command(pkt_id, payload)

            # Avoid busy-waiting when no data
            if not self._rx_buf:
                time.sleep(0.001)

    def _log_disabled_command(self, pkt_id: int, payload: bytes):
        """
        Log received uplink commands as disabled.

        In telemetry-only mode, ALL uplink commands (motion, mode, arm,
        gripper, ballast, estop) are received and CRC-validated for link
        diagnostic purposes, but NONE are forwarded to the Pixhawk.

        This prevents sysid collision with QGroundControl on the MAVLink bus
        while still allowing the GCS operator to verify that the UDP command
        channel (port 5001) is functioning.
        """
        cmd_name = _CMD_NAMES.get(pkt_id, f'0x{pkt_id:02X}')

        # Provide helpful context for each command type
        guidance = {
            0x81: 'Use QGC joystick instead.',
            0x82: 'Use QGC flight mode selector instead.',
            0x83: 'Gripper control via QGC only.',
            0x84: 'Ballast control via QGC only.',
            0x85: 'Use QGC arm/disarm toolbar instead.',
            0x86: 'E-STOP must be triggered via QGC or hardware switch.',
        }.get(pkt_id, '')

        if guidance:
            self.get_logger().warn(
                f'⛔ Uplink command REJECTED: {cmd_name} (0x{pkt_id:02X}) — '
                f'{guidance} '
                f'[telemetry-only mode, payload={len(payload)}B]')
        else:
            self.get_logger().debug(
                f'Unknown packet ID: 0x{pkt_id:02X} (len={len(payload)}) — ignored')

    # ═══════════════════════════════════════════════════════════════════════
    #  MAVROS subscriber callbacks  (DOWNLINK — ALL ACTIVE)
    # ═══════════════════════════════════════════════════════════════════════
    def _imu_callback(self, msg: Imu):
        """Store orientation (roll, pitch, yaw) from MAVROS IMU data."""
        q = msg.orientation
        roll, pitch, yaw = quaternion_to_euler_deg(q.x, q.y, q.z, q.w)
        with self._state_lock:
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

    def _handle_qr_callback(self, qr_data: str, camera_id: int):
        """
        Handle incoming QR data from webcam_streamer.

        Formats a TELEM_QR (0x04) UDP payload with metadata header:
          [camera_id (1B)] [zone_id (1B)] [valid (1B)] [str_len (1B)] [qr_string (N B)]

        camera_id: 0 = Front, 1 = Bottom
        zone_id:   parsed from QR string content (SIDE-A=0, SIDE-B=1, SIDE-C=2, SIDE-D=3, default=255)
        valid:     1 unless the string contains "ERR" or "INVALID"

        Throttling rules (avoid flooding the GCS):
          - If the QR string differs from the last sent string → send immediately.
          - If the same QR code is still in view → send at most once per second (1 Hz).
          - Otherwise → drop (already sent recently).
        """
        if not qr_data:
            return

        # ── Parse zone_id from QR content ──────────────────────────────
        qr_upper = qr_data.upper()
        zone_id = 255
        if "SIDE-A" in qr_upper or "SIDE_A" in qr_upper:
            zone_id = 0
        elif "SIDE-B" in qr_upper or "SIDE_B" in qr_upper:
            zone_id = 1
        elif "SIDE-C" in qr_upper or "SIDE_C" in qr_upper:
            zone_id = 2
        elif "SIDE-D" in qr_upper or "SIDE_D" in qr_upper:
            zone_id = 3

        # ── Validity check ─────────────────────────────────────────────
        valid = 0 if ("ERR" in qr_upper or "INVALID" in qr_upper) else 1

        # ── Encode string ──────────────────────────────────────────────
        qr_bytes = qr_data.encode('utf-8')
        str_len = min(len(qr_bytes), 255)

        camera_names = {0: 'Front', 1: 'Bottom'}

        now = time.monotonic()
        should_send = False

        # Build a compound key for throttling (camera + content)
        throttle_key = f'{camera_id}:{qr_data}'
        if throttle_key != self._last_qr_sent_data:
            # New / changed QR code → always send
            should_send = True
        elif now - self._last_qr_send_time >= 1.0:
            # Same QR still in view, but 1 s has elapsed → re-send at 1 Hz
            should_send = True

        if should_send:
            # Pack: camera_id (B), zone_id (B), valid (B), str_len (B) + string bytes
            payload = struct.pack(
                '<BBBB', camera_id, zone_id, valid, str_len) + qr_bytes[:str_len]

            self._send_packet(TELEM_QR, payload)
            self._last_qr_sent_data = throttle_key
            self._last_qr_send_time = now
            self.get_logger().info(
                f'QR telemetry sent [{camera_names.get(camera_id, "?")}]: '
                f'zone={zone_id} valid={valid} '
                f'"{qr_data}" ({str_len}B payload)')

    # ═══════════════════════════════════════════════════════════════════════
    #  Timer callbacks  (TELEMETRY BROADCAST — ACTIVE)
    # ═══════════════════════════════════════════════════════════════════════

    # ── _publish_manual_control — REMOVED ──────────────────────────────────
    #
    # This method previously published ManualControl messages to
    # /mavros/manual_control/send at 10 Hz based on incoming CMD_MOTION
    # packets.  It has been removed because QGroundControl now owns the
    # exclusive MAVLink control path.
    #
    # Restoring it would cause MAVLink sysid collisions between this
    # node (sysid = MAVROS component) and QGC (sysid = 255), leading to
    # the Pixhawk alternating between two control sources and becoming
    # unresponsive to both.
    #
    # def _publish_manual_control(self):
    #     ...  # REMOVED — see commit 5a1edb0 for original implementation

    def _publish_telemetry(self):
        """
        Assemble and transmit TELEM_IMU, TELEM_DEPTH, and TELEM_STATUS packets
        to the GCS at the configured rate (default 20 Hz).

        This is the sole active timer callback in telemetry-only mode.
        All binary packet structures are preserved exactly for compatibility
        with the custom PyQt5 GCS.
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
