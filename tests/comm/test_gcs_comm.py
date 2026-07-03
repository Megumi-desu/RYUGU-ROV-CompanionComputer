#!/usr/bin/env python3
"""
test_gcs_comm.py — GCS ↔ Jetson UDP Communication Test

Standalone test script (no ROS2/MAVROS required) to verify the Ethernet
Cat6 link between the Jetson Orin Nano and the GCS Laptop using a custom
binary UDP protocol.

Network:
  Jetson  192.168.1.10   ← binds UDP :5001 (receives GCS commands)
  GCS     192.168.1.100  ← receives UDP :5002 (telemetry from Jetson)

Packet Structure:
  [SYNC 0xAA55 LE (2B)] [ID (1B)] [LEN (2B LE)] [PAYLOAD (0..1024B)] [CRC16 (2B LE)]

Usage:
  python3 test_gcs_comm.py
  python3 test_gcs_comm.py --jetson-ip 192.168.1.10 --gcs-ip 192.168.1.100

Press Ctrl+C to stop.
"""

import argparse
import math
import socket
import struct
import sys
import threading
import time

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
    BLUE    = '\033[94m'
    MAGENTA = '\033[95m'
    CYAN    = '\033[96m'
    WHITE   = '\033[97m'
    BG_RED  = '\033[41m'

# ═══════════════════════════════════════════════════════════════════════════
#  CRC-16/CCITT (Poly 0x1021, Init 0xFFFF, non-reflected)
# ═══════════════════════════════════════════════════════════════════════════
_CRC_POLY = 0x1021
_crc_table = []
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
    """Compute CRC-16/CCITT over *data*."""
    crc = 0xFFFF
    for byte in data:
        idx = ((crc >> 8) ^ byte) & 0xFF
        crc = ((crc << 8) ^ _crc_table[idx]) & 0xFFFF
    return crc

# ═══════════════════════════════════════════════════════════════════════════
#  Packet constants
# ═══════════════════════════════════════════════════════════════════════════
SYNC_WORD   = 0xAA55          # stored little-endian → wire bytes 0x55 0xAA
SYNC_BYTES  = struct.pack('<H', SYNC_WORD)   # b'\x55\xAA'

HEADER_SIZE = 5               # SYNC(2) + ID(1) + LEN(2)
CRC_SIZE    = 2
MAX_PAYLOAD = 1024

# ── GCS → Jetson command IDs ──
CMD_MOTION   = 0x81
CMD_MODE     = 0x82
CMD_GRIPPER  = 0x83
CMD_ARM      = 0x85
CMD_ESTOP    = 0x86

# ── Jetson → GCS telemetry IDs ──
TELEM_IMU    = 0x01
TELEM_DEPTH  = 0x02
TELEM_STATUS = 0x03
ACK          = 0xF0

# ── Human-readable names ──
CMD_NAMES = {
    CMD_MOTION:  'CMD_MOTION',
    CMD_MODE:    'CMD_MODE',
    CMD_GRIPPER: 'CMD_GRIPPER',
    CMD_ARM:     'CMD_ARM',
    CMD_ESTOP:   'CMD_ESTOP',
}

MODE_NAMES = {
    0: 'MANUAL',
    1: 'STABILIZE',
    2: 'DEPTH_HOLD',
    3: 'AUTONOMOUS',
}

GRIPPER_NAMES = {
    0: 'STOP',
    1: 'OPEN',
    2: 'CLOSE',
}

# ═══════════════════════════════════════════════════════════════════════════
#  Packet builder / parser
# ═══════════════════════════════════════════════════════════════════════════
def build_packet(pkt_id: int, payload: bytes = b'') -> bytes:
    """
    Build a complete wire packet:
      SYNC(2) + ID(1) + LEN(2) + PAYLOAD(N) + CRC16(2)

    CRC is computed over ID + LEN + PAYLOAD (excludes SYNC, includes header).
    """
    length = len(payload)
    header = struct.pack('<HBH', SYNC_WORD, pkt_id, length)
    crc_data = struct.pack('<BH', pkt_id, length) + payload
    crc = crc16_ccitt(crc_data)
    return header + payload + struct.pack('<H', crc)


def parse_packet(buf: bytearray):
    """
    Attempt to extract one valid packet from *buf*.

    Returns:
        (pkt_id, payload, bytes_consumed)  on success
        None                               if no complete packet yet

    On sync errors the buffer is advanced past the bad byte(s).
    """
    while True:
        # Need at least HEADER_SIZE + CRC_SIZE bytes
        if len(buf) < HEADER_SIZE + CRC_SIZE:
            return None

        # Look for SYNC word
        sync_pos = buf.find(SYNC_BYTES)
        if sync_pos < 0:
            # No sync found — discard everything except the last byte
            # (which could be the first byte of a partial sync)
            if len(buf) > 1:
                discard = len(buf) - 1
                del buf[:discard]
            return None
        if sync_pos > 0:
            # Discard bytes before the sync
            del buf[:sync_pos]

        # Now buf[0:2] == SYNC_BYTES
        if len(buf) < HEADER_SIZE:
            return None

        _, pkt_id, length = struct.unpack_from('<HBH', buf, 0)

        if length > MAX_PAYLOAD:
            # Invalid length — skip this sync and try again
            del buf[:2]
            continue

        total = HEADER_SIZE + length + CRC_SIZE
        if len(buf) < total:
            return None  # wait for more data

        payload = bytes(buf[HEADER_SIZE: HEADER_SIZE + length])

        # Validate CRC over ID + LEN + PAYLOAD
        crc_data = struct.pack('<BH', pkt_id, length) + payload
        crc_expected = crc16_ccitt(crc_data)
        crc_received = struct.unpack_from('<H', buf, HEADER_SIZE + length)[0]

        if crc_received != crc_expected:
            # CRC mismatch — skip this sync and search again
            del buf[:2]
            continue

        # Valid packet
        del buf[:total]
        return (pkt_id, payload, total)


# ═══════════════════════════════════════════════════════════════════════════
#  GCS Communication Handler
# ═══════════════════════════════════════════════════════════════════════════
class GCSCommHandler:
    """
    Manages the UDP link between Jetson and GCS.

    - Receives GCS commands on JETSON_IP:5001
    - Sends telemetry to GCS_IP:5002
    - Sends ACK responses for mode/arm/estop commands
    """

    def __init__(self, jetson_ip: str, gcs_ip: str,
                 cmd_port: int = 5001, telem_port: int = 5002):
        self.jetson_ip = jetson_ip
        self.gcs_ip = gcs_ip
        self.cmd_port = cmd_port
        self.telem_port = telem_port

        self._sock: socket.socket | None = None
        self._running = False
        self._rx_buf = bytearray()

        # Statistics
        self._rx_count = 0
        self._tx_count = 0
        self._crc_errors = 0
        self._start_time = 0.0
        self._lock = threading.Lock()

    # ── Socket setup ─────────────────────────────────────────────────────
    def open(self) -> bool:
        """Create and bind the UDP socket. Returns False on failure."""
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._sock.bind((self.jetson_ip, self.cmd_port))
            self._sock.setblocking(False)
            self._sock.settimeout(0.05)   # 50ms recv timeout
            return True

        except OSError as e:
            print(f'\n{C.RED}{C.BOLD}ERROR: Could not bind to '
                  f'{self.jetson_ip}:{self.cmd_port}{C.RESET}')
            print(f'  {C.YELLOW}Reason: {e}{C.RESET}')
            print()
            print(f'  {C.BOLD}Troubleshooting:{C.RESET}')
            print(f'  1. Check that the Ethernet cable is connected.')
            print(f'  2. Verify the IP address is assigned:')
            print(f'     {C.DIM}ip addr show | grep {self.jetson_ip}{C.RESET}')
            print(f'  3. If using a different interface, set the IP:')
            print(f'     {C.DIM}sudo ip addr add {self.jetson_ip}/24 dev eth0{C.RESET}')
            print(f'  4. Check that no other process is using port {self.cmd_port}:')
            print(f'     {C.DIM}sudo ss -ulnp | grep {self.cmd_port}{C.RESET}')
            print()
            return False

    def close(self):
        if self._sock:
            self._sock.close()
            self._sock = None

    # ── Transmit ─────────────────────────────────────────────────────────
    def send_packet(self, pkt_id: int, payload: bytes = b''):
        """Build and send a packet to the GCS."""
        pkt = build_packet(pkt_id, payload)
        try:
            self._sock.sendto(pkt, (self.gcs_ip, self.telem_port))
            with self._lock:
                self._tx_count += 1
        except OSError as e:
            pass  # silently ignore send errors (GCS might not be up)

    def send_ack(self, acked_id: int):
        """Send an ACK packet with the acked command's ID."""
        payload = struct.pack('<B', acked_id)
        self.send_packet(ACK, payload)

    # ── Receive ──────────────────────────────────────────────────────────
    def receive_once(self):
        """
        Try to receive data and parse packets.
        Returns a list of (pkt_id, payload) tuples, or empty list.
        """
        packets = []

        # Read from socket into buffer
        try:
            data, addr = self._sock.recvfrom(4096)
            self._rx_buf.extend(data)
        except (socket.timeout, BlockingIOError):
            pass  # no data available

        # Parse all complete packets from buffer
        while True:
            result = parse_packet(self._rx_buf)
            if result is None:
                break
            pkt_id, payload, consumed = result
            with self._lock:
                self._rx_count += 1
            packets.append((pkt_id, payload))

        return packets

    # ── Statistics ───────────────────────────────────────────────────────
    @property
    def stats(self):
        with self._lock:
            return {
                'rx': self._rx_count,
                'tx': self._tx_count,
            }


# ═══════════════════════════════════════════════════════════════════════════
#  Command handlers
# ═══════════════════════════════════════════════════════════════════════════
def handle_motion(payload: bytes):
    """Parse and display CMD_MOTION (0x81)."""
    if len(payload) != 12:
        print(f'  {C.RED}Invalid MOTION payload length: {len(payload)}{C.RESET}')
        return
    surge, sway, heave, yaw, pitch, roll = struct.unpack('<6h', payload)
    print(f'  {C.GREEN}▸ MOTION{C.RESET}  '
          f'Surge:{surge:+5d}  Sway:{sway:+5d}  Heave:{heave:+5d}  '
          f'Yaw:{yaw:+5d}  Pitch:{pitch:+5d}  Roll:{roll:+5d}')


def handle_mode(payload: bytes, comm: GCSCommHandler):
    """Parse CMD_MODE (0x82) and send ACK."""
    if len(payload) != 1:
        print(f'  {C.RED}Invalid MODE payload length: {len(payload)}{C.RESET}')
        return
    mode_id = struct.unpack('<B', payload)[0]
    mode_name = MODE_NAMES.get(mode_id, f'UNKNOWN({mode_id})')
    print(f'  {C.CYAN}▸ MODE CHANGE{C.RESET}  '
          f'Request to ID: {mode_id} ({mode_name})')
    comm.send_ack(CMD_MODE)
    print(f'    {C.DIM}→ ACK sent (0xF0, acked_id=0x82){C.RESET}')


def handle_gripper(payload: bytes):
    """Parse CMD_GRIPPER (0x83)."""
    if len(payload) != 1:
        print(f'  {C.RED}Invalid GRIPPER payload length: {len(payload)}{C.RESET}')
        return
    state = struct.unpack('<B', payload)[0]
    state_name = GRIPPER_NAMES.get(state, f'UNKNOWN({state})')
    print(f'  {C.MAGENTA}▸ GRIPPER{C.RESET}  State: {state} ({state_name})')


def handle_arm(payload: bytes, comm: GCSCommHandler):
    """Parse CMD_ARM (0x85) and send ACK."""
    if len(payload) != 1:
        print(f'  {C.RED}Invalid ARM payload length: {len(payload)}{C.RESET}')
        return
    arm_state = struct.unpack('<B', payload)[0]
    label = 'ARM' if arm_state == 1 else 'DISARM'
    color = C.RED if arm_state == 1 else C.GREEN
    print(f'  {color}{C.BOLD}▸ {label}{C.RESET}  '
          f'Request state: {arm_state}')
    comm.send_ack(CMD_ARM)
    print(f'    {C.DIM}→ ACK sent (0xF0, acked_id=0x85){C.RESET}')


def handle_estop(comm: GCSCommHandler):
    """Handle CMD_ESTOP (0x86) and send ACK."""
    print(f'  {C.BG_RED}{C.WHITE}{C.BOLD} ⚡ E-STOP TRIGGERED! {C.RESET}')
    comm.send_ack(CMD_ESTOP)
    print(f'    {C.DIM}→ ACK sent (0xF0, acked_id=0x86){C.RESET}')


def dispatch_command(pkt_id: int, payload: bytes, comm: GCSCommHandler):
    """Route a received packet to its handler."""
    name = CMD_NAMES.get(pkt_id, f'UNKNOWN(0x{pkt_id:02X})')
    ts = time.strftime('%H:%M:%S')

    print(f'{C.DIM}[{ts}]{C.RESET} '
          f'{C.YELLOW}RX{C.RESET} {name} '
          f'({len(payload)}B)')

    if pkt_id == CMD_MOTION:
        handle_motion(payload)
    elif pkt_id == CMD_MODE:
        handle_mode(payload, comm)
    elif pkt_id == CMD_GRIPPER:
        handle_gripper(payload)
    elif pkt_id == CMD_ARM:
        handle_arm(payload, comm)
    elif pkt_id == CMD_ESTOP:
        handle_estop(comm)
    else:
        print(f'  {C.DIM}(unhandled packet ID 0x{pkt_id:02X}){C.RESET}')


# ═══════════════════════════════════════════════════════════════════════════
#  Telemetry transmitter (10 Hz background thread)
# ═══════════════════════════════════════════════════════════════════════════
class TelemetryTransmitter:
    """
    Sends mock telemetry packets to the GCS at 10 Hz.

    TELEM_IMU    (0x01): sinusoidal pitch/roll/yaw
    TELEM_DEPTH  (0x02): sinusoidal depth 0.0–1.5 m
    TELEM_STATUS (0x03): battery + arm state + mode + dummy thrusters
    """

    def __init__(self, comm: GCSCommHandler):
        self.comm = comm
        self._thread: threading.Thread | None = None
        self._running = False
        self._tick = 0

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)

    def _loop(self):
        interval = 1.0 / 10.0   # 10 Hz
        while self._running:
            t = time.monotonic()
            self._send_telemetry()
            self._tick += 1
            elapsed = time.monotonic() - t
            sleep_time = interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _send_telemetry(self):
        t = self._tick / 10.0   # seconds since start

        # ── TELEM_IMU (0x01): pitch, roll, yaw in degrees ──
        pitch = 5.0 * math.sin(2.0 * math.pi * 0.2 * t)          # ±5° at 0.2 Hz
        roll  = 3.0 * math.sin(2.0 * math.pi * 0.15 * t + 0.5)   # ±3° at 0.15 Hz
        yaw   = 180.0 * math.sin(2.0 * math.pi * 0.05 * t)       # ±180° at 0.05 Hz
        imu_payload = struct.pack('<3f', pitch, roll, yaw)
        self.comm.send_packet(TELEM_IMU, imu_payload)

        # ── TELEM_DEPTH (0x02): depth_m, altitude_m ──
        depth = 0.75 + 0.75 * math.sin(2.0 * math.pi * 0.1 * t)  # 0.0–1.5 m
        altitude = 5.0 - depth   # mock altitude above seabed
        depth_payload = struct.pack('<2f', depth, altitude)
        self.comm.send_packet(TELEM_DEPTH, depth_payload)

        # ── TELEM_STATUS (0x03): battery_v, arm_state, mode_id, 8× thruster ──
        battery_v = 15.6 + 0.1 * math.sin(2.0 * math.pi * 0.02 * t)  # ~15.5–15.7 V
        arm_state = 0   # disarmed (mock)
        mode_id   = 0   # MANUAL (mock)
        # 8 dummy thruster output values (0–255 range as placeholder)
        thrusters = bytes([
            int(128 + 50 * math.sin(2.0 * math.pi * 0.3 * t + i * 0.5))
            for i in range(8)
        ])
        status_payload = struct.pack('<fBB', battery_v, arm_state, mode_id) + thrusters
        self.comm.send_packet(TELEM_STATUS, status_payload)


# ═══════════════════════════════════════════════════════════════════════════
#  Banner & status display
# ═══════════════════════════════════════════════════════════════════════════
def print_banner(jetson_ip, gcs_ip, cmd_port, telem_port):
    print(f'\n{C.BOLD}{"═" * 64}{C.RESET}')
    print(f'{C.BOLD}{C.CYAN}      RYUGU ROV — GCS Communication Test{C.RESET}')
    print(f'{C.BOLD}{"═" * 64}{C.RESET}')
    print(f'  {C.BOLD}Jetson (this device):{C.RESET}  {C.GREEN}{jetson_ip}:{cmd_port}{C.RESET}  (RX commands)')
    print(f'  {C.BOLD}GCS Laptop:          {C.RESET}  {C.GREEN}{gcs_ip}:{telem_port}{C.RESET}  (TX telemetry)')
    print(f'{C.BOLD}{"─" * 64}{C.RESET}')
    print(f'  {C.BOLD}Protocol:{C.RESET}  SYNC(0xAA55) + ID + LEN + PAYLOAD + CRC16/CCITT')
    print(f'  {C.BOLD}Telemetry:{C.RESET} Sending mock IMU/Depth/Status at {C.CYAN}10 Hz{C.RESET}')
    print(f'  {C.BOLD}Receiver: {C.RESET} Listening for GCS commands...')
    print(f'{C.BOLD}{"─" * 64}{C.RESET}')
    print(f'  {C.DIM}Press Ctrl+C to stop{C.RESET}\n')


def print_stats(comm: GCSCommHandler, elapsed: float):
    s = comm.stats
    print(f'\n{C.BOLD}{"─" * 64}{C.RESET}')
    print(f'  {C.BOLD}Session Stats:{C.RESET}')
    print(f'    Duration:      {elapsed:.1f}s')
    print(f'    RX packets:    {s["rx"]}')
    print(f'    TX packets:    {s["tx"]}')
    if elapsed > 0:
        print(f'    TX rate:       {s["tx"] / elapsed:.1f} pkt/s')
    print(f'{C.BOLD}{"─" * 64}{C.RESET}\n')


# ═══════════════════════════════════════════════════════════════════════════
#  Main
# ═══════════════════════════════════════════════════════════════════════════
def main():
    parser = argparse.ArgumentParser(
        description='RYUGU ROV — GCS ↔ Jetson UDP communication test'
    )
    parser.add_argument('--jetson-ip', default='192.168.1.10',
                        help='Jetson IP address (default: 192.168.1.10)')
    parser.add_argument('--gcs-ip', default='192.168.1.100',
                        help='GCS Laptop IP address (default: 192.168.1.100)')
    parser.add_argument('--cmd-port', type=int, default=5001,
                        help='UDP port for incoming commands (default: 5001)')
    parser.add_argument('--telem-port', type=int, default=5002,
                        help='UDP port for outgoing telemetry (default: 5002)')
    args = parser.parse_args()

    # ── Create handler ──
    comm = GCSCommHandler(
        jetson_ip=args.jetson_ip,
        gcs_ip=args.gcs_ip,
        cmd_port=args.cmd_port,
        telem_port=args.telem_port,
    )

    if not comm.open():
        sys.exit(1)

    print_banner(args.jetson_ip, args.gcs_ip, args.cmd_port, args.telem_port)

    # ── Start telemetry transmitter (10 Hz background thread) ──
    telem = TelemetryTransmitter(comm)
    telem.start()

    start_time = time.monotonic()

    # ── Main receive loop ──
    try:
        while True:
            packets = comm.receive_once()
            for pkt_id, payload in packets:
                dispatch_command(pkt_id, payload, comm)

            # Small sleep to avoid busy-waiting (socket already has 50ms timeout)
            time.sleep(0.001)

    except KeyboardInterrupt:
        pass
    finally:
        elapsed = time.monotonic() - start_time
        telem.stop()
        comm.close()
        print_stats(comm, elapsed)
        print(f'{C.YELLOW}Communication test stopped.{C.RESET}\n')


if __name__ == '__main__':
    main()
