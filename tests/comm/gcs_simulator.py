#!/usr/bin/env python3
"""
gcs_simulator.py — Simulates the GCS Laptop for testing test_gcs_comm.py

Run this on the GCS Laptop (192.168.1.100) or locally on the Jetson using
loopback for a self-contained test.

Usage (on GCS Laptop):
  python3 gcs_simulator.py

Usage (loopback test on Jetson — both scripts on same machine):
  # Terminal 1:
  python3 test_gcs_comm.py --jetson-ip 127.0.0.1 --gcs-ip 127.0.0.1
  # Terminal 2:
  python3 gcs_simulator.py --jetson-ip 127.0.0.1 --gcs-ip 127.0.0.1

Menu:
  [1] Send CMD_MOTION (surge=+200)
  [2] Send CMD_MODE (STABILIZE)
  [3] Send CMD_GRIPPER (OPEN)
  [4] Send CMD_ARM
  [5] Send CMD_DISARM
  [6] Send CMD_ESTOP
  [T] Toggle telemetry display
  [Q] Quit
"""

import argparse
import socket
import struct
import sys
import threading
import time

# ── Import protocol elements from test_gcs_comm ──
# (Duplicated here so this script is fully self-contained)

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
    crc = 0xFFFF
    for byte in data:
        idx = ((crc >> 8) ^ byte) & 0xFF
        crc = ((crc << 8) ^ _crc_table[idx]) & 0xFFFF
    return crc


SYNC_WORD  = 0xAA55
SYNC_BYTES = struct.pack('<H', SYNC_WORD)
HEADER_SIZE = 5
CRC_SIZE    = 2
MAX_PAYLOAD = 1024

# Packet IDs
CMD_MOTION  = 0x81
CMD_MODE    = 0x82
CMD_GRIPPER = 0x83
CMD_ARM     = 0x85
CMD_ESTOP   = 0x86
TELEM_IMU   = 0x01
TELEM_DEPTH = 0x02
TELEM_STATUS = 0x03
ACK         = 0xF0

MODE_NAMES = {0: 'MANUAL', 1: 'STABILIZE', 2: 'DEPTH_HOLD', 3: 'AUTONOMOUS'}

# ANSI
class C:
    RESET = '\033[0m'; BOLD = '\033[1m'; DIM = '\033[2m'
    RED = '\033[91m'; GREEN = '\033[92m'; YELLOW = '\033[93m'
    BLUE = '\033[94m'; MAGENTA = '\033[95m'; CYAN = '\033[96m'


def build_packet(pkt_id: int, payload: bytes = b'') -> bytes:
    length = len(payload)
    header = struct.pack('<HBH', SYNC_WORD, pkt_id, length)
    crc_data = struct.pack('<BH', pkt_id, length) + payload
    crc = crc16_ccitt(crc_data)
    return header + payload + struct.pack('<H', crc)


def parse_packet(buf: bytearray):
    while True:
        if len(buf) < HEADER_SIZE + CRC_SIZE:
            return None
        sync_pos = buf.find(SYNC_BYTES)
        if sync_pos < 0:
            if len(buf) > 1:
                del buf[:len(buf) - 1]
            return None
        if sync_pos > 0:
            del buf[:sync_pos]
        if len(buf) < HEADER_SIZE:
            return None
        _, pkt_id, length = struct.unpack_from('<HBH', buf, 0)
        if length > MAX_PAYLOAD:
            del buf[:2]
            continue
        total = HEADER_SIZE + length + CRC_SIZE
        if len(buf) < total:
            return None
        payload = bytes(buf[HEADER_SIZE: HEADER_SIZE + length])
        crc_data = struct.pack('<BH', pkt_id, length) + payload
        crc_expected = crc16_ccitt(crc_data)
        crc_received = struct.unpack_from('<H', buf, HEADER_SIZE + length)[0]
        if crc_received != crc_expected:
            del buf[:2]
            continue
        del buf[:total]
        return (pkt_id, payload, total)


# ── Telemetry receiver thread ─────────────────────────────────────────────
class TelemetryReceiver:
    def __init__(self, gcs_ip: str, telem_port: int):
        self.gcs_ip = gcs_ip
        self.telem_port = telem_port
        self._sock = None
        self._thread = None
        self._running = False
        self._display = True
        self._rx_count = 0
        self._lock = threading.Lock()

    def start(self):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((self.gcs_ip, self.telem_port))
        self._sock.settimeout(0.1)
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        if self._sock:
            self._sock.close()

    def toggle_display(self):
        self._display = not self._display
        state = 'ON' if self._display else 'OFF'
        print(f'  {C.DIM}Telemetry display: {state}{C.RESET}')

    @property
    def rx_count(self):
        with self._lock:
            return self._rx_count

    def _loop(self):
        buf = bytearray()
        while self._running:
            try:
                data, _ = self._sock.recvfrom(4096)
                buf.extend(data)
            except (socket.timeout, BlockingIOError):
                continue

            while True:
                result = parse_packet(buf)
                if result is None:
                    break
                pkt_id, payload, _ = result
                with self._lock:
                    self._rx_count += 1
                if self._display:
                    self._print_telemetry(pkt_id, payload)

    def _print_telemetry(self, pkt_id, payload):
        ts = time.strftime('%H:%M:%S')
        if pkt_id == TELEM_IMU and len(payload) == 12:
            pitch, roll, yaw = struct.unpack('<3f', payload)
            print(f'  {C.DIM}[{ts}]{C.RESET} {C.BLUE}TELEM_IMU{C.RESET}    '
                  f'P:{pitch:+6.1f}°  R:{roll:+6.1f}°  Y:{yaw:+7.1f}°')
        elif pkt_id == TELEM_DEPTH and len(payload) == 8:
            depth, alt = struct.unpack('<2f', payload)
            print(f'  {C.DIM}[{ts}]{C.RESET} {C.CYAN}TELEM_DEPTH{C.RESET}  '
                  f'Depth:{depth:5.2f}m  Alt:{alt:5.2f}m')
        elif pkt_id == TELEM_STATUS and len(payload) == 14:
            batt_v, arm, mode = struct.unpack_from('<fBB', payload, 0)
            mode_name = MODE_NAMES.get(mode, f'?{mode}')
            arm_str = 'ARMED' if arm else 'DISARMED'
            print(f'  {C.DIM}[{ts}]{C.RESET} {C.MAGENTA}TELEM_STATUS{C.RESET} '
                  f'Batt:{batt_v:5.2f}V  {arm_str}  {mode_name}')
        elif pkt_id == ACK and len(payload) == 1:
            acked = struct.unpack('<B', payload)[0]
            print(f'  {C.DIM}[{ts}]{C.RESET} {C.GREEN}ACK{C.RESET}          '
                  f'Acked ID: 0x{acked:02X}')


# ── Main ──────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description='GCS Simulator for RYUGU ROV comm test')
    parser.add_argument('--jetson-ip', default='192.168.1.10')
    parser.add_argument('--gcs-ip', default='192.168.1.100')
    parser.add_argument('--cmd-port', type=int, default=5001)
    parser.add_argument('--telem-port', type=int, default=5002)
    args = parser.parse_args()

    # Send socket (GCS → Jetson)
    tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Telemetry receiver
    telem_rx = TelemetryReceiver(args.gcs_ip, args.telem_port)
    try:
        telem_rx.start()
    except OSError as e:
        print(f'{C.RED}Cannot bind {args.gcs_ip}:{args.telem_port}: {e}{C.RESET}')
        print(f'For loopback test, use: --gcs-ip 127.0.0.1')
        sys.exit(1)

    dest = (args.jetson_ip, args.cmd_port)

    print(f'\n{C.BOLD}{"═" * 60}{C.RESET}')
    print(f'{C.BOLD}{C.CYAN}      GCS Simulator → Jetson{C.RESET}')
    print(f'{C.BOLD}{"═" * 60}{C.RESET}')
    print(f'  Sending to: {args.jetson_ip}:{args.cmd_port}')
    print(f'  Receiving on: {args.gcs_ip}:{args.telem_port}')
    print(f'{C.BOLD}{"─" * 60}{C.RESET}')
    print(f'  {C.YELLOW}[1]{C.RESET} CMD_MOTION (surge=+200)')
    print(f'  {C.YELLOW}[2]{C.RESET} CMD_MODE (STABILIZE)')
    print(f'  {C.YELLOW}[3]{C.RESET} CMD_GRIPPER (OPEN)')
    print(f'  {C.YELLOW}[4]{C.RESET} CMD_ARM')
    print(f'  {C.YELLOW}[5]{C.RESET} CMD_DISARM')
    print(f'  {C.YELLOW}[6]{C.RESET} CMD_ESTOP')
    print(f'  {C.YELLOW}[T]{C.RESET} Toggle telemetry display')
    print(f'  {C.YELLOW}[Q]{C.RESET} Quit')
    print(f'{C.BOLD}{"─" * 60}{C.RESET}\n')

    try:
        while True:
            choice = input(f'{C.DIM}GCS>{C.RESET} ').strip().lower()
            if choice == '1':
                payload = struct.pack('<6h', 200, 0, 0, 0, 0, 0)
                pkt = build_packet(CMD_MOTION, payload)
                tx_sock.sendto(pkt, dest)
                print(f'  {C.GREEN}→ Sent CMD_MOTION (surge=+200){C.RESET}')
            elif choice == '2':
                payload = struct.pack('<B', 1)  # STABILIZE
                pkt = build_packet(CMD_MODE, payload)
                tx_sock.sendto(pkt, dest)
                print(f'  {C.GREEN}→ Sent CMD_MODE (1=STABILIZE){C.RESET}')
            elif choice == '3':
                payload = struct.pack('<B', 1)  # OPEN
                pkt = build_packet(CMD_GRIPPER, payload)
                tx_sock.sendto(pkt, dest)
                print(f'  {C.GREEN}→ Sent CMD_GRIPPER (1=OPEN){C.RESET}')
            elif choice == '4':
                payload = struct.pack('<B', 1)
                pkt = build_packet(CMD_ARM, payload)
                tx_sock.sendto(pkt, dest)
                print(f'  {C.GREEN}→ Sent CMD_ARM{C.RESET}')
            elif choice == '5':
                payload = struct.pack('<B', 0)
                pkt = build_packet(CMD_ARM, payload)
                tx_sock.sendto(pkt, dest)
                print(f'  {C.GREEN}→ Sent CMD_DISARM{C.RESET}')
            elif choice == '6':
                pkt = build_packet(CMD_ESTOP, b'')
                tx_sock.sendto(pkt, dest)
                print(f'  {C.RED}→ Sent CMD_ESTOP{C.RESET}')
            elif choice == 't':
                telem_rx.toggle_display()
            elif choice == 'q':
                break
            else:
                print(f'  {C.DIM}Unknown command{C.RESET}')
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        telem_rx.stop()
        tx_sock.close()
        print(f'\n  {C.DIM}Telemetry packets received: {telem_rx.rx_count}{C.RESET}')
        print(f'{C.YELLOW}GCS Simulator stopped.{C.RESET}\n')


if __name__ == '__main__':
    main()
