#!/usr/bin/env python3
"""
webcam_streamer.py — Dual USB Webcam MJPEG Streamer for RYUGU ROV

Captures video from two USB webcams and streams them over HTTP as MJPEG
so the GCS laptop can display them in a browser, VLC, or OpenCV.

Streams:
  Front Camera → http://192.168.1.10:8554/video  (/dev/video0)
  Bottom Camera → http://192.168.1.10:8555/video  (/dev/video1)

If a camera is disconnected, its stream serves a static placeholder
image instead of crashing.

Usage:
  python3 webcam_streamer.py
  python3 webcam_streamer.py --bind 0.0.0.0         # bind to all interfaces
  python3 webcam_streamer.py --front-dev /dev/video2 # custom device nodes
  python3 webcam_streamer.py --width 640 --height 480 --fps 30

View on GCS:
  Browser:  http://192.168.1.10:8554/video
  VLC:      vlc http://192.168.1.10:8554/video
  OpenCV:   cv2.VideoCapture("http://192.168.1.10:8554/video")

Press Ctrl+C to stop.
"""

import argparse
import os
import sys
import time
import threading
import signal
from http.server import HTTPServer, BaseHTTPRequestHandler
from socketserver import ThreadingMixIn

import cv2
import numpy as np

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
    CYAN    = '\033[96m'

# ═══════════════════════════════════════════════════════════════════════════
#  Placeholder image generator
# ═══════════════════════════════════════════════════════════════════════════
def generate_placeholder(width: int, height: int, label: str) -> bytes:
    """
    Generate a dark placeholder JPEG with a 'Camera Disconnected' message.
    Returns JPEG-encoded bytes.
    """
    img = np.zeros((height, width, 3), dtype=np.uint8)
    img[:] = (30, 30, 30)  # dark grey background

    # Draw a camera icon (simple rectangle + circle)
    cx, cy = width // 2, height // 2 - 30
    cv2.rectangle(img, (cx - 50, cy - 30), (cx + 50, cy + 30), (80, 80, 80), 2)
    cv2.circle(img, (cx, cy), 18, (80, 80, 80), 2)

    # Draw an X over the camera icon
    cv2.line(img, (cx - 35, cy - 25), (cx + 35, cy + 25), (0, 0, 180), 2)
    cv2.line(img, (cx + 35, cy - 25), (cx - 35, cy + 25), (0, 0, 180), 2)

    # "Camera Disconnected" text
    text = "CAMERA DISCONNECTED"
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.6
    thickness = 2
    (tw, th), _ = cv2.getTextSize(text, font, scale, thickness)
    tx = (width - tw) // 2
    ty = cy + 60
    cv2.putText(img, text, (tx, ty), font, scale, (0, 0, 200), thickness,
                cv2.LINE_AA)

    # Label (e.g., "Front Camera" / "Bottom Camera")
    (lw, lh), _ = cv2.getTextSize(label, font, 0.5, 1)
    lx = (width - lw) // 2
    ly = ty + 30
    cv2.putText(img, label, (lx, ly), font, 0.5, (120, 120, 120), 1,
                cv2.LINE_AA)

    _, jpeg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 70])
    return jpeg.tobytes()


# ═══════════════════════════════════════════════════════════════════════════
#  Camera capture thread
# ═══════════════════════════════════════════════════════════════════════════
class CameraCapture:
    """
    Captures frames from a V4L2 USB webcam in a background thread.
    Stores the latest JPEG-encoded frame for the MJPEG server to read.

    If the camera is unavailable, serves a placeholder frame.
    Periodically retries opening a disconnected camera.
    """

    def __init__(self, device: str, label: str,
                 width: int = 640, height: int = 480,
                 fps: int = 30, jpeg_quality: int = 70):
        self.device = device
        self.label = label
        self.width = width
        self.height = height
        self.fps = fps
        self.jpeg_quality = jpeg_quality

        self._lock = threading.Lock()
        self._frame_jpeg: bytes = generate_placeholder(width, height, label)
        self._connected = False
        self._running = False
        self._thread: threading.Thread | None = None
        self._cap: cv2.VideoCapture | None = None

        # Statistics
        self._frame_count = 0
        self._fps_actual = 0.0
        self._last_fps_time = 0.0
        self._fps_counter = 0

    @property
    def connected(self) -> bool:
        with self._lock:
            return self._connected

    @property
    def frame_jpeg(self) -> bytes:
        with self._lock:
            return self._frame_jpeg

    @property
    def frame_count(self) -> int:
        with self._lock:
            return self._frame_count

    @property
    def fps_actual(self) -> float:
        with self._lock:
            return self._fps_actual

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=3.0)
        if self._cap and self._cap.isOpened():
            self._cap.release()

    def _open_camera(self) -> bool:
        """Attempt to open the camera device."""
        if not os.path.exists(self.device):
            return False

        try:
            cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
            if not cap.isOpened():
                cap.release()
                return False

            # Configure capture properties
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            cap.set(cv2.CAP_PROP_FPS, self.fps)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # minimize latency

            # Try MJPG fourcc for USB cameras (usually faster than YUYV)
            cap.set(cv2.CAP_PROP_FOURCC,
                    cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))

            self._cap = cap
            with self._lock:
                self._connected = True
            return True

        except Exception:
            return False

    def _capture_loop(self):
        """Main capture loop — opens camera, reads frames, handles reconnection."""
        retry_interval = 3.0   # seconds between reconnection attempts

        while self._running:
            # ── Attempt to open camera ──
            if self._cap is None or not self._cap.isOpened():
                with self._lock:
                    self._connected = False
                    self._frame_jpeg = generate_placeholder(
                        self.width, self.height, self.label)

                if self._open_camera():
                    actual_w = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                    actual_h = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                    actual_fps = self._cap.get(cv2.CAP_PROP_FPS)
                    print(f'  {C.GREEN}✓ {self.label}{C.RESET} opened: '
                          f'{self.device} ({actual_w}x{actual_h} @ {actual_fps:.0f}fps)')
                else:
                    time.sleep(retry_interval)
                    continue

            # ── Read frame ──
            ret, frame = self._cap.read()
            if not ret:
                print(f'  {C.YELLOW}⚠ {self.label}{C.RESET}: '
                      f'read failed, reconnecting...')
                self._cap.release()
                self._cap = None
                continue

            # ── Encode to JPEG ──
            _, jpeg = cv2.imencode(
                '.jpg', frame,
                [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])

            with self._lock:
                self._frame_jpeg = jpeg.tobytes()
                self._frame_count += 1
                self._fps_counter += 1

            # ── FPS calculation ──
            now = time.monotonic()
            if self._last_fps_time == 0.0:
                self._last_fps_time = now
            elif now - self._last_fps_time >= 1.0:
                with self._lock:
                    self._fps_actual = self._fps_counter / (now - self._last_fps_time)
                    self._fps_counter = 0
                self._last_fps_time = now


# ═══════════════════════════════════════════════════════════════════════════
#  MJPEG HTTP server
# ═══════════════════════════════════════════════════════════════════════════
BOUNDARY = b'--ryugu-mjpeg-boundary'


class MJPEGHandler(BaseHTTPRequestHandler):
    """Serves MJPEG stream on /video and status info on /."""

    # Camera capture reference — set per-server instance
    camera: CameraCapture = None
    stream_fps: int = 30

    def log_message(self, format, *args):
        """Suppress default access logging to keep console clean."""
        pass

    def do_GET(self):
        if self.path == '/video':
            self._stream_video()
        elif self.path == '/':
            self._serve_status_page()
        elif self.path == '/snapshot':
            self._serve_snapshot()
        else:
            self.send_error(404, 'Not Found')

    def _stream_video(self):
        """Send continuous MJPEG stream."""
        self.send_response(200)
        self.send_header('Content-Type',
                         f'multipart/x-mixed-replace; boundary={BOUNDARY.decode()}')
        self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
        self.send_header('Pragma', 'no-cache')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()

        interval = 1.0 / self.stream_fps

        try:
            while True:
                frame = self.camera.frame_jpeg
                self.wfile.write(BOUNDARY + b'\r\n')
                self.wfile.write(b'Content-Type: image/jpeg\r\n')
                self.wfile.write(f'Content-Length: {len(frame)}\r\n'.encode())
                self.wfile.write(b'\r\n')
                self.wfile.write(frame)
                self.wfile.write(b'\r\n')
                self.wfile.flush()
                time.sleep(interval)
        except (BrokenPipeError, ConnectionResetError):
            pass  # Client disconnected

    def _serve_snapshot(self):
        """Serve a single JPEG snapshot."""
        frame = self.camera.frame_jpeg
        self.send_response(200)
        self.send_header('Content-Type', 'image/jpeg')
        self.send_header('Content-Length', str(len(frame)))
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(frame)

    def _serve_status_page(self):
        """Serve a simple HTML status page with embedded stream."""
        cam = self.camera
        status = "CONNECTED" if cam.connected else "DISCONNECTED"
        colour = "#4ade80" if cam.connected else "#f87171"

        html = f"""<!DOCTYPE html>
<html>
<head>
    <title>RYUGU ROV — {cam.label}</title>
    <style>
        body {{ background: #111; color: #eee; font-family: monospace;
               display: flex; flex-direction: column; align-items: center;
               margin: 0; padding: 20px; }}
        h1 {{ color: #38bdf8; }}
        .status {{ color: {colour}; font-weight: bold; font-size: 1.2em; }}
        img {{ border: 2px solid #333; margin-top: 10px; }}
        .info {{ color: #888; margin-top: 10px; font-size: 0.9em; }}
    </style>
</head>
<body>
    <h1>RYUGU ROV — {cam.label}</h1>
    <p class="status">{status} — {cam.device}</p>
    <img src="/video" width="{cam.width}" height="{cam.height}" />
    <p class="info">
        Frames: {cam.frame_count} | FPS: {cam.fps_actual:.1f} |
        Resolution: {cam.width}x{cam.height}<br>
        Snapshot: <a href="/snapshot" style="color:#38bdf8">/snapshot</a> |
        Stream: <a href="/video" style="color:#38bdf8">/video</a>
    </p>
</body>
</html>"""
        content = html.encode()
        self.send_response(200)
        self.send_header('Content-Type', 'text/html')
        self.send_header('Content-Length', str(len(content)))
        self.end_headers()
        self.wfile.write(content)


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    """HTTP server that handles each request in a new thread."""
    allow_reuse_address = True
    daemon_threads = True


def start_mjpeg_server(bind_ip: str, port: int, camera: CameraCapture,
                       stream_fps: int) -> ThreadedHTTPServer:
    """Create and start an MJPEG HTTP server in a background thread."""
    # Create a handler class with the camera reference bound
    handler = type('Handler', (MJPEGHandler,), {
        'camera': camera,
        'stream_fps': stream_fps,
    })

    server = ThreadedHTTPServer((bind_ip, port), handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server


# ═══════════════════════════════════════════════════════════════════════════
#  Banner
# ═══════════════════════════════════════════════════════════════════════════
def print_banner(bind_ip, front_port, bottom_port, front_dev, bottom_dev,
                 width, height, fps):
    print(f'\n{C.BOLD}{"═" * 64}{C.RESET}')
    print(f'{C.BOLD}{C.CYAN}      RYUGU ROV — Dual Webcam MJPEG Streamer{C.RESET}')
    print(f'{C.BOLD}{"═" * 64}{C.RESET}')
    print(f'  {C.BOLD}Front Camera: {C.RESET} {front_dev} → '
          f'{C.GREEN}http://{bind_ip}:{front_port}/video{C.RESET}')
    print(f'  {C.BOLD}Bottom Camera:{C.RESET} {bottom_dev} → '
          f'{C.GREEN}http://{bind_ip}:{bottom_port}/video{C.RESET}')
    print(f'{C.BOLD}{"─" * 64}{C.RESET}')
    print(f'  {C.BOLD}Resolution:{C.RESET}  {width}x{height}')
    print(f'  {C.BOLD}Target FPS:{C.RESET}  {fps}')
    print(f'  {C.BOLD}JPEG Quality:{C.RESET} 70')
    print(f'  {C.BOLD}Endpoints: {C.RESET} /video (stream)  /snapshot (JPEG)  / (status page)')
    print(f'{C.BOLD}{"─" * 64}{C.RESET}')


# ═══════════════════════════════════════════════════════════════════════════
#  Status monitor (prints camera stats periodically)
# ═══════════════════════════════════════════════════════════════════════════
def status_monitor(cameras: list, interval: float = 5.0):
    """Print camera status every `interval` seconds."""
    while True:
        time.sleep(interval)
        parts = []
        for cam in cameras:
            status = f'{C.GREEN}OK{C.RESET}' if cam.connected else f'{C.RED}OFF{C.RESET}'
            fps = f'{cam.fps_actual:5.1f}'
            parts.append(f'{cam.label}: {status} {fps}fps ({cam.frame_count} frames)')
        ts = time.strftime('%H:%M:%S')
        print(f'  {C.DIM}[{ts}]{C.RESET}  {"  |  ".join(parts)}')


# ═══════════════════════════════════════════════════════════════════════════
#  Main
# ═══════════════════════════════════════════════════════════════════════════
def main():
    parser = argparse.ArgumentParser(
        description='RYUGU ROV — Dual Webcam MJPEG Streamer')
    parser.add_argument('--bind', default='0.0.0.0',
                        help='IP to bind HTTP servers (default: 0.0.0.0)')
    parser.add_argument('--front-port', type=int, default=8554,
                        help='Front camera stream port (default: 8554)')
    parser.add_argument('--bottom-port', type=int, default=8555,
                        help='Bottom camera stream port (default: 8555)')
    parser.add_argument('--front-dev', default='/dev/video0',
                        help='Front camera device (default: /dev/video0)')
    parser.add_argument('--bottom-dev', default='/dev/video1',
                        help='Bottom camera device (default: /dev/video1)')
    parser.add_argument('--width', type=int, default=640,
                        help='Capture width (default: 640)')
    parser.add_argument('--height', type=int, default=480,
                        help='Capture height (default: 480)')
    parser.add_argument('--fps', type=int, default=30,
                        help='Target capture/stream FPS (default: 30)')
    parser.add_argument('--quality', type=int, default=70,
                        help='JPEG quality 1-100 (default: 70)')
    args = parser.parse_args()

    print_banner(args.bind, args.front_port, args.bottom_port,
                 args.front_dev, args.bottom_dev,
                 args.width, args.height, args.fps)

    # ── Check device availability ──
    for dev in [args.front_dev, args.bottom_dev]:
        if os.path.exists(dev):
            print(f'  {C.GREEN}✓{C.RESET} Found {dev}')
        else:
            print(f'  {C.YELLOW}✗{C.RESET} {dev} not found — will serve placeholder')

    print(f'{C.BOLD}{"─" * 64}{C.RESET}')

    # ── Create camera capture threads ──
    front_cam = CameraCapture(
        device=args.front_dev, label='Front Camera',
        width=args.width, height=args.height,
        fps=args.fps, jpeg_quality=args.quality,
    )
    bottom_cam = CameraCapture(
        device=args.bottom_dev, label='Bottom Camera',
        width=args.width, height=args.height,
        fps=args.fps, jpeg_quality=args.quality,
    )

    cameras = [front_cam, bottom_cam]

    # ── Start capture threads ──
    front_cam.start()
    bottom_cam.start()

    # ── Start MJPEG HTTP servers ──
    try:
        front_server = start_mjpeg_server(
            args.bind, args.front_port, front_cam, args.fps)
        print(f'  {C.GREEN}✓{C.RESET} Front Camera server started on '
              f'{C.CYAN}:{args.front_port}{C.RESET}')
    except OSError as e:
        print(f'  {C.RED}✗ Cannot start Front Camera server on '
              f'port {args.front_port}: {e}{C.RESET}')
        front_server = None

    try:
        bottom_server = start_mjpeg_server(
            args.bind, args.bottom_port, bottom_cam, args.fps)
        print(f'  {C.GREEN}✓{C.RESET} Bottom Camera server started on '
              f'{C.CYAN}:{args.bottom_port}{C.RESET}')
    except OSError as e:
        print(f'  {C.RED}✗ Cannot start Bottom Camera server on '
              f'port {args.bottom_port}: {e}{C.RESET}')
        bottom_server = None

    print(f'\n  {C.DIM}Streaming... Press Ctrl+C to stop{C.RESET}\n')

    # ── Status monitor in background ──
    monitor = threading.Thread(target=status_monitor, args=(cameras, 5.0),
                               daemon=True)
    monitor.start()

    # ── Wait for Ctrl+C ──
    try:
        signal.pause()
    except KeyboardInterrupt:
        pass
    finally:
        print(f'\n{C.YELLOW}Shutting down...{C.RESET}')
        front_cam.stop()
        bottom_cam.stop()
        if front_server:
            front_server.shutdown()
        if bottom_server:
            bottom_server.shutdown()
        print(f'{C.GREEN}Webcam streamer stopped.{C.RESET}\n')


if __name__ == '__main__':
    main()
