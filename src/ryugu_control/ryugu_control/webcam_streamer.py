#!/usr/bin/env python3
"""
webcam_streamer.py — Dual USB Webcam MJPEG Streaming Node for RYUGU ROV

Captures video from two USB webcams and streams them over HTTP as MJPEG
so the GCS laptop can display them in a browser, VLC, or OpenCV.

Streams:
  Front Camera → http://192.168.1.10:8554/video  (/dev/video0)
  Bottom Camera → http://192.168.1.10:8555/video  (/dev/video2)

Robustness:
  - If a camera is disconnected at startup, its stream serves a static
    dark placeholder image with "Camera Offline" text.
  - If a camera disconnects during runtime, the stream automatically
    falls back to the placeholder and attempts reconnection without
    blocking the other camera.
  - If only one webcam is present, the missing camera's port still
    serves a placeholder.

Endpoints on each port:
  /          — HTML status page with embedded stream
  /video     — MJPEG stream (multipart/x-mixed-replace)
  /snapshot  — Single JPEG snapshot

Usage:
  ros2 run ryugu_control webcam_streamer
  ros2 run ryugu_control webcam_streamer --ros-args -p bind:=0.0.0.0
"""

import os
import threading
import time
import signal
from http.server import HTTPServer, BaseHTTPRequestHandler
from socketserver import ThreadingMixIn
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node

# ═══════════════════════════════════════════════════════════════════════════════
#  Placeholder image generator
# ═══════════════════════════════════════════════════════════════════════════════
def generate_placeholder(width: int, height: int, label: str) -> bytes:
    """
    Generate a dark placeholder JPEG with a 'Camera Disconnected' message.

    Args:
        width, height: Image dimensions.
        label: Human-readable camera name (e.g. "Front Camera").

    Returns:
        JPEG-encoded bytes.
    """
    img = np.zeros((height, width, 3), dtype=np.uint8)
    img[:] = (30, 30, 30)  # dark grey background

    # Camera icon (rectangle + circle)
    cx, cy = width // 2, height // 2 - 30
    cv2.rectangle(img, (cx - 50, cy - 30), (cx + 50, cy + 30), (80, 80, 80), 2)
    cv2.circle(img, (cx, cy), 18, (80, 80, 80), 2)

    # Red X over the camera icon
    cv2.line(img, (cx - 35, cy - 25), (cx + 35, cy + 25), (0, 0, 180), 2)
    cv2.line(img, (cx + 35, cy - 25), (cx - 35, cy + 25), (0, 0, 180), 2)

    # "CAMERA DISCONNECTED" text
    font = cv2.FONT_HERSHEY_SIMPLEX
    text = "CAMERA DISCONNECTED"
    scale = 0.6
    thickness = 2
    (tw, th), _ = cv2.getTextSize(text, font, scale, thickness)
    tx = (width - tw) // 2
    ty = cy + 60
    cv2.putText(img, text, (tx, ty), font, scale, (0, 0, 200), thickness,
                cv2.LINE_AA)

    # Label (e.g. "Front Camera" / "Bottom Camera")
    (lw, lh), _ = cv2.getTextSize(label, font, 0.5, 1)
    lx = (width - lw) // 2
    ly = ty + 30
    cv2.putText(img, label, (lx, ly), font, 0.5, (120, 120, 120), 1,
                cv2.LINE_AA)

    _, jpeg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 70])
    return jpeg.tobytes()


# ═══════════════════════════════════════════════════════════════════════════════
#  Camera capture thread
# ═══════════════════════════════════════════════════════════════════════════════
class CameraCapture:
    """
    Captures frames from a V4L2 USB webcam in a background thread.

    Stores the latest JPEG-encoded frame for the MJPEG server to serve.
    If the camera is unavailable, serves a placeholder frame and retries
    periodically.
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
        self._thread: Optional[threading.Thread] = None
        self._cap: Optional[cv2.VideoCapture] = None

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
        """Start the background capture thread."""
        self._running = True
        self._thread = threading.Thread(
            target=self._capture_loop, name=f'cam-{self.label}', daemon=True)
        self._thread.start()

    def stop(self):
        """Stop capture and release resources."""
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=3.0)
        if self._cap is not None and self._cap.isOpened():
            self._cap.release()

    def _open_camera(self) -> bool:
        """Attempt to open the V4L2 camera device.  Returns True on success."""
        if not os.path.exists(self.device):
            return False

        try:
            cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
            if not cap.isOpened():
                cap.release()
                return False

            # ── Set MJPEG fourcc FIRST ──────────────────────────────
            # This MUST be set before width/height/FPS.  On Linux V4L2,
            # setting resolution first locks the camera into the default
            # YUYV (uncompressed) format, which saturates USB 2.0 bus
            # bandwidth and forces the driver to throttle FPS (e.g. 7.5).
            # Setting FOURCC first selects compressed MJPEG from the start,
            # freeing enough USB bandwidth for stable 30 FPS on both cameras.
            cap.set(cv2.CAP_PROP_FOURCC,
                    cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))

            # Configure remaining capture properties
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            cap.set(cv2.CAP_PROP_FPS, self.fps)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)   # minimise latency

            self._cap = cap
            with self._lock:
                self._connected = True
            return True

        except Exception:
            return False

    def _capture_loop(self):
        """Main loop: open camera, read frames, handle disconnection."""
        retry_interval = 3.0  # seconds between reconnection attempts

        while self._running:
            # ── Attempt to open camera ──────────────────────────────────
            if self._cap is None or not self._cap.isOpened():
                with self._lock:
                    self._connected = False
                    self._frame_jpeg = generate_placeholder(
                        self.width, self.height, self.label)

                if self._open_camera():
                    actual_w = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                    actual_h = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                    actual_fps = self._cap.get(cv2.CAP_PROP_FPS)
                    print(f'  [INFO] {self.label}: opened {self.device} '
                          f'({actual_w}x{actual_h} @ {actual_fps:.0f}fps)')
                else:
                    time.sleep(retry_interval)
                    continue

            # ── Read frame ──────────────────────────────────────────────
            ret, frame = self._cap.read()
            if not ret:
                print(f'  [WARN] {self.label}: read failed, reconnecting...')
                self._cap.release()
                self._cap = None
                continue

            # ── Encode to JPEG ──────────────────────────────────────────
            _, jpeg = cv2.imencode(
                '.jpg', frame,
                [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])

            with self._lock:
                self._frame_jpeg = jpeg.tobytes()
                self._frame_count += 1
                self._fps_counter += 1

            # ── FPS calculation (once per second) ───────────────────────
            now = time.monotonic()
            if self._last_fps_time == 0.0:
                self._last_fps_time = now
            elif now - self._last_fps_time >= 1.0:
                with self._lock:
                    self._fps_actual = (
                        self._fps_counter / (now - self._last_fps_time))
                    self._fps_counter = 0
                self._last_fps_time = now


# ═══════════════════════════════════════════════════════════════════════════════
#  MJPEG HTTP handler & server
# ═══════════════════════════════════════════════════════════════════════════════
BOUNDARY = b'--ryugu-mjpeg-boundary'


class MJPEGHandler(BaseHTTPRequestHandler):
    """
    HTTP request handler for MJPEG streaming.

    Routes:
      GET /video    → multipart MJPEG stream
      GET /         → HTML status page
      GET /snapshot → single JPEG frame

    Class attributes (set per server instance):
      camera: CameraCapture
      stream_fps: int
    """

    camera: CameraCapture = None      # set per-instance
    stream_fps: int = 30

    def log_message(self, format, *args):
        """Suppress default access logging."""
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
        """Send a continuous multipart MJPEG stream."""
        self.send_response(200)
        self.send_header(
            'Content-Type',
            f'multipart/x-mixed-replace; boundary={BOUNDARY.decode()}')
        self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
        self.send_header('Pragma', 'no-cache')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()

        interval = 1.0 / max(1, self.stream_fps)

        try:
            while True:
                frame = self.camera.frame_jpeg
                self.wfile.write(BOUNDARY + b'\r\n')
                self.wfile.write(b'Content-Type: image/jpeg\r\n')
                self.wfile.write(
                    f'Content-Length: {len(frame)}\r\n'.encode())
                self.wfile.write(b'\r\n')
                self.wfile.write(frame)
                self.wfile.write(b'\r\n')
                self.wfile.flush()
                time.sleep(interval)
        except (BrokenPipeError, ConnectionResetError):
            pass  # client disconnected — normal

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
        """Serve a simple HTML status page with embedded stream preview."""
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
        img {{ border: 2px solid #333; margin-top: 10px; max-width: 100%; }}
        .info {{ color: #888; margin-top: 10px; font-size: 0.9em; }}
        a {{ color: #38bdf8; }}
    </style>
    <meta http-equiv="refresh" content="5">
</head>
<body>
    <h1>🟦 RYUGU ROV — {cam.label}</h1>
    <p class="status">{status} — {cam.device}</p>
    <img src="/video" width="{cam.width}" height="{cam.height}" alt="Stream" />
    <p class="info">
        Frames: {cam.frame_count} | FPS: {cam.fps_actual:.1f} |
        Resolution: {cam.width}x{cam.height}<br>
        <a href="/snapshot">/snapshot</a> (single JPEG) |
        <a href="/video">/video</a> (live stream)
    </p>
</body>
</html>"""
        content = html.encode()
        self.send_response(200)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.send_header('Content-Length', str(len(content)))
        self.end_headers()
        self.wfile.write(content)


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    """HTTP server that handles each request in its own thread."""
    allow_reuse_address = True
    daemon_threads = True


def _start_mjpeg_server(bind_ip: str, port: int, camera: CameraCapture,
                        stream_fps: int) -> ThreadedHTTPServer:
    """Create and start an MJPEG HTTP server in a background thread."""
    # Bind the camera reference to a handler subclass
    handler_cls = type('BoundHandler', (MJPEGHandler,), {
        'camera': camera,
        'stream_fps': stream_fps,
    })

    server = ThreadedHTTPServer((bind_ip, port), handler_cls)
    thread = threading.Thread(
        target=server.serve_forever, name=f'http-{port}', daemon=True)
    thread.start()
    return server


# ═══════════════════════════════════════════════════════════════════════════════
#  WebcamStreamerNode  (ROS2 wrapper)
# ═══════════════════════════════════════════════════════════════════════════════
class WebcamStreamerNode(Node):
    """
    ROS2 node that manages dual USB webcam MJPEG streams.

    Starts two HTTP MJPEG servers on configurable ports with automatic
    camera detection, placeholder fallback, and reconnection logic.
    """

    def __init__(self):
        super().__init__('webcam_streamer')

        # ── Declare parameters ──────────────────────────────────────────
        self.declare_parameter('bind', '0.0.0.0')
        self.declare_parameter('front_port', 8554)
        self.declare_parameter('bottom_port', 8555)
        self.declare_parameter('front_dev', '/dev/video0')
        self.declare_parameter('bottom_dev', '/dev/video2')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('jpeg_quality', 70)

        bind_ip     = self.get_parameter('bind').get_parameter_value().string_value
        front_port  = self.get_parameter('front_port').get_parameter_value().integer_value
        bottom_port = self.get_parameter('bottom_port').get_parameter_value().integer_value
        front_dev   = self.get_parameter('front_dev').get_parameter_value().string_value
        bottom_dev  = self.get_parameter('bottom_dev').get_parameter_value().string_value
        width       = self.get_parameter('width').get_parameter_value().integer_value
        height      = self.get_parameter('height').get_parameter_value().integer_value
        fps         = self.get_parameter('fps').get_parameter_value().integer_value
        quality     = self.get_parameter('jpeg_quality').get_parameter_value().integer_value

        # ── Device availability check ───────────────────────────────────
        for dev, name in [(front_dev, 'Front'), (bottom_dev, 'Bottom')]:
            if os.path.exists(dev):
                self.get_logger().info(f'{name} camera found: {dev}')
            else:
                self.get_logger().warn(
                    f'{name} camera device {dev} not found — '
                    f'will serve placeholder')

        # ── Create camera capture instances ─────────────────────────────
        self._front_cam = CameraCapture(
            device=front_dev, label='Front Camera',
            width=width, height=height, fps=fps, jpeg_quality=quality)
        self._bottom_cam = CameraCapture(
            device=bottom_dev, label='Bottom Camera',
            width=width, height=height, fps=fps, jpeg_quality=quality)

        self._cameras = [self._front_cam, self._bottom_cam]

        # ── Start capture threads ───────────────────────────────────────
        self._front_cam.start()
        self._bottom_cam.start()

        # ── Start MJPEG HTTP servers ────────────────────────────────────
        self._front_server: Optional[ThreadedHTTPServer] = None
        self._bottom_server: Optional[ThreadedHTTPServer] = None

        try:
            self._front_server = _start_mjpeg_server(
                bind_ip, front_port, self._front_cam, fps)
            self.get_logger().info(
                f'Front camera stream: http://{bind_ip}:{front_port}/video')
        except OSError as e:
            self.get_logger().error(
                f'Cannot start front camera server on port {front_port}: {e}')

        try:
            self._bottom_server = _start_mjpeg_server(
                bind_ip, bottom_port, self._bottom_cam, fps)
            self.get_logger().info(
                f'Bottom camera stream: http://{bind_ip}:{bottom_port}/video')
        except OSError as e:
            self.get_logger().error(
                f'Cannot start bottom camera server on port {bottom_port}: {e}')

        # ── Status monitor timer (logs stats every 30 s) ────────────────
        self._monitor_timer = self.create_timer(30.0, self._log_status)

        self.get_logger().info(
            f'WebcamStreamerNode started — '
            f'{width}x{height} @ {fps}fps target, JPEG quality {quality}')

    def _log_status(self):
        """Periodically log camera status and statistics."""
        parts = []
        for cam in self._cameras:
            state = 'OK' if cam.connected else 'OFFLINE'
            parts.append(
                f'{cam.label}: {state} '
                f'{cam.fps_actual:.1f}fps ({cam.frame_count} frames)')
        self.get_logger().info(' | '.join(parts))

    def destroy_node(self):
        """Clean shutdown: stop cameras and HTTP servers."""
        self.get_logger().info('Shutting down webcam streamer...')

        # Stop capture threads
        for cam in self._cameras:
            cam.stop()

        # Stop HTTP servers
        for srv in (self._front_server, self._bottom_server):
            if srv is not None:
                srv.shutdown()

        self.get_logger().info('Webcam streamer stopped.')
        super().destroy_node()


# ═══════════════════════════════════════════════════════════════════════════════
#  Entry point
# ═══════════════════════════════════════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)
    node = WebcamStreamerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
