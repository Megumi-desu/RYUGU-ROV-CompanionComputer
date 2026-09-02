#!/usr/bin/env python3
"""
webcam_streamer.py — Dual USB Webcam MJPEG Streaming Node for RYUGU ROV

Captures video from two USB webcams and streams them over HTTP as MJPEG
so the GCS laptop can display them in a browser, VLC, or OpenCV.

Also performs offline QR code detection (throttled to 5 Hz) on captured
frames and publishes decoded strings to /ryugu/qr/front and /ryugu/qr/bottom for downstream
telemetry via gcs_bridge_node.

Streams:
  Front Camera → http://192.168.1.10:8554/video  (/dev/video0)
  Bottom Camera → http://192.168.1.10:8555/video  (/dev/video2)

QR Detection:
  Publishers: /ryugu/qr/front, /ryugu/qr/bottom  (std_msgs/String)
  Scan rate: 5 Hz (every ~200 ms) on each camera independently
  Headless:  No GUI — safe for Jetson Orin Nano (no imshow/waitKey)

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
from pyzbar.pyzbar import decode as pyzbar_decode
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from sensor_msgs.msg import Image as RosImage
from cv_bridge import CvBridge

# Dynamic vision_msgs import (prevents failure when the package is missing —
# same guard pattern as hook_detection_node).  When available, the
# /ryugu/vision/hook_detections subscription draws ALL detected classes;
# otherwise only the hook_target box from the Float32MultiArray is drawn.
try:
    from vision_msgs.msg import Detection2DArray
    HAS_VISION_MSGS = True
except ImportError:
    Detection2DArray = None
    HAS_VISION_MSGS = False

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
        self._frame_raw: Optional[np.ndarray] = None  # latest raw BGR frame
        self._connected = False
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._cap: Optional[cv2.VideoCapture] = None

        # QR code detection (using pyzbar for better small/dense QR support)
        self._latest_qr_data: Optional[str] = None
        self._last_qr_scan_time = 0.0
        self._qr_scan_interval = 0.2   # 5 Hz throttle
        self._latest_qr_objects = []   # cached decoded objects for overlay drawing

        # AI vision overlay state (fed by WebcamStreamerNode subscriptions
        # from hook_detection_node; drawn by the capture thread)
        self._ai_target: Optional[tuple] = None   # (cx, cy, bw, bh, conf, x1, y1, x2, y2)
        self._ai_detections: list = []            # [(x1, y1, x2, y2, class_id, conf), ...]
        self._ai_fps: float = 0.0
        self._ai_last_update: float = 0.0         # monotonic timestamp
        self._ai_stale_timeout = 0.5              # s — hide boxes if target data stops
        self._ai_liveness_timeout = 2.0           # s — AI status line stays while alive

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

    def get_raw_frame(self) -> Optional[np.ndarray]:
        """
        Return a copy of the latest raw BGR frame, or None if unavailable.

        Used by hook_detection_node (via the ROS Image publisher) to get
        frames for TensorRT inference without re-opening the camera device.
        """
        with self._lock:
            if self._frame_raw is not None:
                return self._frame_raw.copy()
            return None

    def consume_qr_data(self) -> Optional[str]:
        """
        Return the latest decoded QR string and clear it (consume pattern).

        Returns None if no new QR data has been detected since the last call.
        """
        with self._lock:
            data = self._latest_qr_data
            self._latest_qr_data = None
            return data

    # ── AI vision overlay state (thread-safe, updated by ROS callbacks) ──
    def update_ai_target(self, cx: float, cy: float, bw: float, bh: float,
                         conf: float, x1: float, y1: float, x2: float,
                         y2: float, detected: bool):
        """Store the latest hook_target state for this camera."""
        with self._lock:
            if detected:
                self._ai_target = (cx, cy, bw, bh, conf, x1, y1, x2, y2)
            else:
                self._ai_target = None   # no target → clear overlays
            self._ai_last_update = time.monotonic()

    def update_ai_detections(self, detections: list):
        """Store all-class detection boxes for this camera."""
        with self._lock:
            self._ai_detections = list(detections)
            self._ai_last_update = time.monotonic()

    def update_ai_fps(self, fps: float):
        """Store the estimated AI inference FPS for this camera."""
        with self._lock:
            self._ai_fps = fps

    def get_ai_overlay(self):
        """
        Return a snapshot (detections, target, ai_fps) for overlay drawing,
        or None if hook_detection_node has not published within the
        liveness window (2 s).

        The AI FPS readout is drawn whenever the pipeline is alive; boxes
        and the crosshair are drawn only while a fresh target exists
        (0.5 s expiry — the GCS never sees frozen detections).
        """
        with self._lock:
            if self._ai_last_update == 0.0:
                return None
            age = time.monotonic() - self._ai_last_update
            if age > self._ai_liveness_timeout:
                return None
            target = self._ai_target
            if target is not None and age > self._ai_stale_timeout:
                target = None
            return (list(self._ai_detections), target, self._ai_fps)

    def _draw_qr_overlays(self, frame, decoded_objects):
        """
        Draw bounding boxes and text labels for detected QR codes directly
        onto the BGR frame (in-place).  Called on every frame so overlays
        persist between detection cycles at full stream FPS.

        Args:
            frame: OpenCV BGR image (modified in-place).
            decoded_objects: List of pyzbar Decoded objects.
        """
        for obj in decoded_objects:
            # ── Draw polygon boundary (precise corner points) ────────
            pts = obj.polygon
            if pts is not None and len(pts) == 4:
                pts_array = np.array([(p.x, p.y) for p in pts], dtype=np.int32)
                cv2.polylines(frame, [pts_array], isClosed=True,
                              color=(0, 255, 0), thickness=2)

            # ── Draw axis-aligned bounding rectangle ─────────────────
            x, y, w, h = obj.rect
            cv2.rectangle(frame, (x, y), (x + w, y + h),
                          color=(0, 255, 0), thickness=2)

            # ── Draw label text with dark background ─────────────────
            try:
                text = obj.data.decode('utf-8')
            except UnicodeDecodeError:
                text = obj.data.hex()[:20]   # fallback for binary data
            if len(text) > 32:
                text = text[:29] + '...'

            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.5
            font_thickness = 2
            (tw, th), baseline = cv2.getTextSize(
                text, font, font_scale, font_thickness)

            label_y = y - 10 if y > 20 else y + h + 20
            # Filled black background behind text for readability
            cv2.rectangle(frame,
                          (x, label_y - th - baseline),
                          (x + tw, label_y + baseline),
                          (0, 0, 0), -1)
            cv2.putText(frame, text, (x, label_y),
                        font, font_scale, (0, 255, 0), font_thickness,
                        cv2.LINE_AA)

    def _draw_vision_overlays(self, frame, detections, target, ai_fps):
        """
        Draw AI hook-detection overlays onto the BGR frame (in-place):

          1. Bounding boxes with per-class colours (from hook_detections,
             or the hook_target box as fallback when vision_msgs is absent).
          2. Red centroid crosshair with coordinate text.
          3. Class label with confidence (e.g. "hook_target 51%").
          4. Clean AI FPS readout in the top-left corner (no status
             banners — the GCS shows only a minimal "NN FPS" text).

        Called on every frame while the target is fresh so overlays track
        at full stream FPS.
        """
        # Per-class colours/names — BGR, matching hook_detection_node
        class_colours = {
            0: (180, 180, 180),  # hook_body_grey  — silver
            1: (255, 255, 255),  # hook_body_white — white
            2: (0, 200, 255),    # hook_target     — gold/orange
        }
        class_names = {
            0: 'hook_body_grey',
            1: 'hook_body_white',
            2: 'hook_target',
        }
        font = cv2.FONT_HERSHEY_SIMPLEX

        # ── Boxes: prefer all-class detections, fall back to target bbox ──
        boxes = detections
        if not boxes and target is not None:
            _, _, _, _, conf, x1, y1, x2, y2 = target
            boxes = [(x1, y1, x2, y2, 2, conf)]

        # ── Filter & cap: confident boxes only, top 5 by score ──────────
        # Prevents UI clutter if the detector ever emits many candidates
        # (e.g. the double-sigmoid "50%" flood).  The hook node already
        # thresholds at 0.55 — this is a second line of defence.
        boxes = [b for b in boxes if b[5] >= 0.50]
        boxes = sorted(boxes, key=lambda b: b[5], reverse=True)[:5]

        for (bx1, by1, bx2, by2, cls_id, conf) in boxes:
            cls_id = int(cls_id)
            colour = class_colours.get(cls_id, (0, 255, 0))
            name = class_names.get(cls_id, f'cls_{cls_id}')
            ix1, iy1 = int(bx1), int(by1)
            ix2, iy2 = int(bx2), int(by2)

            cv2.rectangle(frame, (ix1, iy1), (ix2, iy2), colour, 2)

            # Label with confidence
            label = f'{name} {conf * 100:.0f}%'
            (tw, th), baseline = cv2.getTextSize(label, font, 0.5, 1)
            label_y = iy1 - 6 if iy1 > 20 else iy2 + th + 8
            cv2.rectangle(frame, (ix1, label_y - th - 4),
                          (ix1 + tw + 4, label_y + baseline), (0, 0, 0), -1)
            cv2.putText(frame, label, (ix1 + 2, label_y),
                        font, 0.5, colour, 1, cv2.LINE_AA)

        # ── Target centroid crosshair (red) ─────────────────────────────
        if target is not None:
            cx, cy, *_ = target
            icx, icy = int(cx), int(cy)
            cv2.circle(frame, (icx, icy), 8, (0, 0, 255), 2)
            cv2.line(frame, (icx - 12, icy), (icx + 12, icy), (0, 0, 255), 1)
            cv2.line(frame, (icx, icy - 12), (icx, icy + 12), (0, 0, 255), 1)
            cv2.putText(frame, f'({icx},{icy})', (icx + 10, icy - 10),
                        font, 0.4, (0, 0, 255), 1, cv2.LINE_AA)

        # ── Clean AI FPS overlay (top-left) ─────────────────────────────
        # Minimal readout only — no status banners ("AI DETECT: OK" /
        # "AI: NO TARGET" were removed 2026-09-02 for a cleaner GCS view).
        fps_text = f'{ai_fps:.0f} FPS'
        (tw, th), baseline = cv2.getTextSize(fps_text, font, 0.6, 2)
        cv2.rectangle(frame, (10, 10), (10 + tw + 8, 10 + th + baseline + 6),
                      (0, 0, 0), -1)
        cv2.putText(frame, fps_text, (14, 10 + th + 4),
                    font, 0.6, (0, 255, 0), 2, cv2.LINE_AA)

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

            # ── Bottom camera is physically mounted upside-down ──────────
            # Rotate 180° so the stream, overlays, QR detection, and the
            # ROS image_raw topic are all upright.
            if 'bottom' in self.label.lower():
                frame = cv2.rotate(frame, cv2.ROTATE_180)

            # ── Snapshot the CLEAN frame for ROS image publish ──────────
            # hook_detection_node consumes /ryugu/camera/*/image_raw and
            # must never see the QR/AI overlays drawn for the GCS — the
            # detector would otherwise re-detect its own boxes (feedback).
            raw_frame = frame.copy()

            # ── QR code detection via pyzbar (throttled to 5 Hz) ────────
            now = time.monotonic()
            if now - self._last_qr_scan_time >= self._qr_scan_interval:
                self._last_qr_scan_time = now
                # Convert to grayscale for faster & more reliable decoding
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                decoded_objects = pyzbar_decode(gray)
                self._latest_qr_objects = decoded_objects
                if decoded_objects:
                    # Take the first detected QR code for ROS publish
                    qr_data = decoded_objects[0].data.decode('utf-8')
                    if qr_data:
                        with self._lock:
                            self._latest_qr_data = qr_data

            # ── Draw QR bounding-box overlays on every frame ───────────
            if self._latest_qr_objects:
                self._draw_qr_overlays(frame, self._latest_qr_objects)

            # ── Draw AI hook-detection overlays (if fresh) ──────────────
            vision = self.get_ai_overlay()
            if vision is not None:
                detections, target, ai_fps = vision
                self._draw_vision_overlays(frame, detections, target, ai_fps)

            # ── Encode to JPEG ──────────────────────────────────────────
            _, jpeg = cv2.imencode(
                '.jpg', frame,
                [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])

            with self._lock:
                self._frame_jpeg = jpeg.tobytes()
                self._frame_raw = raw_frame  # clean pre-overlay BGR for ROS Image publish
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
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('fps', 30)
        self.declare_parameter('jpeg_quality', 70)
        self.declare_parameter('publish_raw_images', True)

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

        # ── QR code publishers (one per camera) ─────────────────────────
        self._qr_front_pub = self.create_publisher(String, '/ryugu/qr/front', 10)
        self._qr_bottom_pub = self.create_publisher(String, '/ryugu/qr/bottom', 10)

        # ── Raw image publishers (for hook detection / CV nodes) ────────
        self._publish_raw = self.get_parameter(
            'publish_raw_images').get_parameter_value().bool_value
        self._cv_bridge = CvBridge()
        img_qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1)
        self._img_front_pub = self.create_publisher(
            RosImage, '/ryugu/camera/front/image_raw', img_qos)
        self._img_bottom_pub = self.create_publisher(
            RosImage, '/ryugu/camera/bottom/image_raw', img_qos)

        # ── AI vision subscriptions (from hook_detection_node) ──────────
        # Incoming detection data is routed to the matching CameraCapture
        # instance, which draws overlays onto the MJPEG frames.
        self._ai_fps = [0.0, 0.0]             # per-camera AI inference FPS
        self._ai_last_msg_time = [0.0, 0.0]   # per-camera arrival timestamps

        self._hook_target_sub = self.create_subscription(
            Float32MultiArray, '/ryugu/vision/hook_target',
            self._hook_target_callback, 10)

        self._hook_detections_sub = None
        if HAS_VISION_MSGS:
            self._hook_detections_sub = self.create_subscription(
                Detection2DArray, '/ryugu/vision/hook_detections',
                self._hook_detections_callback, 10)
            self.get_logger().info(
                'Subscribed to /ryugu/vision/hook_detections (all classes)')
        else:
            self.get_logger().warn(
                'vision_msgs not available — drawing hook_target box only')

        # Poll cameras at ~10 Hz for new QR data and raw image publish
        self._qr_timer = self.create_timer(0.1, self._publish_camera_data)

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

    def _publish_camera_data(self):
        """
        Poll both cameras for new QR data and publish non-empty strings.
        Also publishes raw sensor_msgs/Image frames for CV subscriber nodes.

        Publishes to per-camera topics so the GCS knows which camera
        detected the QR code:
          - Front camera → /ryugu/qr/front
          - Bottom camera → /ryugu/qr/bottom

        Uses a consume pattern so each QR code is published exactly once
        per detection (the camera thread clears it on read).
        """
        for cam, qr_pub, img_pub in [
            (self._front_cam, self._qr_front_pub, self._img_front_pub),
            (self._bottom_cam, self._qr_bottom_pub, self._img_bottom_pub),
        ]:
            # ── QR data publish ──
            qr_data = cam.consume_qr_data()
            if qr_data:
                msg = String()
                msg.data = qr_data
                qr_pub.publish(msg)
                self.get_logger().info(
                    f'QR detected on {cam.label}: "{qr_data}"')

            if self._publish_raw and cam.connected:
                raw_frame = cam.get_raw_frame()
                if raw_frame is not None:
                    if len(raw_frame.shape) == 2:
                        raw_frame = cv2.cvtColor(raw_frame, cv2.COLOR_GRAY2BGR)
                    elif len(raw_frame.shape) == 3 and raw_frame.shape[2] == 4:
                        raw_frame = cv2.cvtColor(raw_frame, cv2.COLOR_BGRA2BGR)
                    raw_frame = np.ascontiguousarray(raw_frame, dtype=np.uint8)
                    img_msg = self._cv_bridge.cv2_to_imgmsg(
                        raw_frame, encoding='passthrough')
                    img_msg.encoding = 'bgr8'
                    img_msg.header.stamp = self.get_clock().now().to_msg()
                    img_msg.header.frame_id = cam.label.lower().replace(
                        ' ', '_')
                    img_pub.publish(img_msg)

    # ═══════════════════════════════════════════════════════════════════════
    #  AI vision callbacks (hook_detection_node → MJPEG overlay)
    # ═══════════════════════════════════════════════════════════════════════
    def _hook_target_callback(self, msg: Float32MultiArray):
        """
        Route hook_target data to the matching CameraCapture.

        Payload (12 floats — see _publish_target in hook_detection_node):
          [camera_id, cx, cy, bw, bh, conf, class_id, detected_flag,
           x1, y1, x2, y2]

        Also maintains a per-camera EMA estimate of the AI inference FPS
        from the message arrival rate (hook_detection_node publishes once
        per processed frame, detected or not).
        """
        if len(msg.data) < 8:
            return

        camera_id = int(msg.data[0])
        if camera_id not in (0, 1):
            return

        cam = self._cameras[camera_id]
        detected = msg.data[7] >= 0.5
        cx, cy, bw, bh, conf = (float(v) for v in msg.data[1:6])

        if len(msg.data) >= 12 and detected:
            x1, y1, x2, y2 = (float(v) for v in msg.data[8:12])
        else:
            # Extended fields absent → derive box from centroid + size
            x1, y1 = cx - bw / 2.0, cy - bh / 2.0
            x2, y2 = cx + bw / 2.0, cy + bh / 2.0

        cam.update_ai_target(cx, cy, bw, bh, conf, x1, y1, x2, y2, detected)

        # ── AI FPS estimate (EMA of message arrival rate) ─────────────
        now = time.monotonic()
        last = self._ai_last_msg_time[camera_id]
        if last > 0.0 and now > last:
            inst = 1.0 / (now - last)
            self._ai_fps[camera_id] = (
                self._ai_fps[camera_id] * 0.8 + inst * 0.2)
        self._ai_last_msg_time[camera_id] = now
        cam.update_ai_fps(self._ai_fps[camera_id])

    def _hook_detections_callback(self, msg):
        """
        Route all-class detection boxes to the matching CameraCapture.

        hook_detection_node stamps header.frame_id with 'front' or
        'bottom' so boxes can be attributed to the right MJPEG stream.
        """
        frame_id = msg.header.frame_id.lower()
        if frame_id == 'front':
            cam = self._front_cam
        elif frame_id == 'bottom':
            cam = self._bottom_cam
        else:
            return

        detections = []
        for det in msg.detections:
            x = det.bbox.center.position.x
            y = det.bbox.center.position.y
            sx = det.bbox.size_x
            sy = det.bbox.size_y
            if det.results:
                hyp = det.results[0].hypothesis
                conf = float(hyp.score)
                try:
                    cls_id = int(float(hyp.class_id))
                except ValueError:
                    cls_id = 0
            else:
                conf = 0.0
                cls_id = 0
            detections.append(
                (x - sx / 2.0, y - sy / 2.0, x + sx / 2.0, y + sy / 2.0,
                 cls_id, conf))

        cam.update_ai_detections(detections)

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
