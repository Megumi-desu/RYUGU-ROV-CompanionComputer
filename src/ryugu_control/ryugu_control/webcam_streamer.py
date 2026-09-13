#!/usr/bin/env python3
"""
webcam_streamer.py — Dual USB Webcam MJPEG Streaming Node for RYUGU ROV

Captures video from two USB webcams and streams them over HTTP as MJPEG
so the GCS laptop can display them in a browser, VLC, or OpenCV.

Also performs offline QR code detection (throttled to 5 Hz) on captured
frames and publishes decoded strings to /ryugu/qr/front and /ryugu/qr/bottom for downstream
telemetry via gcs_bridge_node.

Streams:
  Front Camera → http://192.168.1.10:8555/video  (JETE-W7  /dev/video0)
  Bottom Camera → http://192.168.1.10:8554/video  (Xiongmai /dev/video2)

Note: port assignment follows the GCS contract (constants.py: STREAM_URL_FRONT
= :8555, STREAM_URL_BOTTOM = :8554).  front_port/bottom_port were swapped on
2026-09-12 so the GCS's hardcoded :8555/front, :8554/bottom convention matches
the served feeds — the earlier front=:8554 / bottom=:8555 layout displayed the
two cameras cross-wired on the GCS panel.

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
        self._qr_scan_interval = 0.05  # 20 Hz hand-off (fast pre-screen)
        self._latest_qr_objects = []   # cached decoded objects for overlay drawing
        # Sticky overlays: the box stays drawn for 2 s after the last
        # successful decode so it doesn't flicker when decoding is
        # intermittent (2026-09-12).
        self._last_qr_found_at = 0.0
        # Per-camera QR preprocessing.  Both cameras get CLAHE contrast
        # normalisation (pool-scale exposure changes frame-to-frame); the
        # front camera then gets a sharpen + 2× upscale chain, the bottom
        # camera an adaptive threshold (see _decode_qr_pipeline).
        self._clahe = cv2.createCLAHE(
            clipLimit=2.0, tileGridSize=(8, 8))
        # OpenCV QRCodeDetector fallback for the front camera — tolerates
        # the blur / low contrast that occasionally defeats pyzbar.
        self._qr_detector = cv2.QRCodeDetector()
        # QR decoding runs on a DEDICATED worker thread, not the capture
        # thread: the CLAHE/upscale/adaptive-threshold decode chain can cost
        # 100s of ms per attempt, which would otherwise throttle the
        # capture loop's FPS (see 2026-09-12 regression).
        self._qr_wake = threading.Event()
        self._qr_lock = threading.Lock()
        self._qr_input: Optional[np.ndarray] = None   # latest grayscale frame
        self._qr_thread: Optional[threading.Thread] = None

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

    @staticmethod
    def _pyzbar_to_overlay(decoded, scale: float = 1.0) -> dict:
        """
        Convert a pyzbar Decoded object into the overlay-dict format used by
        _draw_qr_overlays, scaling all coordinates by *scale*.

        Args:
            decoded: pyzbar Decoded object (or None).
            scale: Multiply coordinates by this factor (e.g. 0.5 to map a
                   detection made on a 2×-upscaled image back onto the
                   display frame).

        Returns:
            {'data': str, 'rect': (x, y, w, h),
             'polygon': [(x, y), (x, y), (x, y), (x, y)]}
        """
        rect = tuple(int(v * scale) for v in decoded.rect)
        polygon = [(int(p.x * scale), int(p.y * scale))
                   for p in decoded.polygon]
        try:
            data = decoded.data.decode('utf-8')
        except UnicodeDecodeError:
            data = decoded.data.hex()[:20]   # fallback for binary data
        return {'data': data, 'rect': rect, 'polygon': polygon}

    def _fast_detect(self, gray):
        """
        Cheap pre-screen: is there anything textured to decode at all?

        A Laplacian edge-energy check on a half-res copy (~3 ms) returns
        True whenever the frame carries any real detail, so QR frames are
        NEVER rejected — this gate only short-circuits pathologically
        smooth frames (lens covered, black underwater void).  The earlier
        QRCodeDetector-based gate was dropped: it false-negatived exactly
        the small/distant SIDE-B/C labels this stack must catch.

        Threshold lowered 4.0 → 1.0 (2026-09-12): a large QR code held
        close (~10 cm) fills most of the frame with two solid regions
        (black modules + white background); the mean Laplacian energy can
        fall below 4.0 even though the QR is perfectly clear, causing the
        whole pipeline to be skipped before it has a chance to run.
        """
        small = gray[::2, ::2]
        return float(cv2.Laplacian(small, cv2.CV_64F).var()) >= 1.0

    def _decode_qr_pipeline(self, gray, fast=True):
        """
        Run the per-camera QR decode pipeline on a grayscale frame.

        When *fast* is set, a cheap edge-energy pre-screen (_fast_detect,
        ~3 ms) rejects only pathologically smooth frames (camera covered /
        black void); every textured frame, including ones carrying small
        distant SIDE-B/C labels, is passed to the full chain.

        CLAHE is computed once before the if-front branch and stored in
        *frame_qr* so both cameras share the enhancement without repeating
        the work.  The variable MUST be assigned outside the 'front' block
        so the bottom camera branch can reference it without NameError.

        ── Front camera (JETE-W7 /dev/video0) ──────────────────────────

          Pass 0 — raw pyzbar, NO preprocessing:
              A large, clear QR code (~10 cm away) decodes directly from
              the unprocessed grayscale frame.  CLAHE+sharpen were
              HURTING detection: high-contrast QR edges clipped at 0/255
              by the sharpen kernel corrupt the module boundaries that
              pyzbar relies on.  Cheapest path; handles the close-up case
              that was consistently failing with the old pipeline.

          Pass 1 — CLAHE only → multi-scale pyzbar:
              Contrast normalisation WITHOUT the sharpen kernel.
              Handles dim or uneven pool lighting while preserving module
              edges.  Three-scale sweep: 2× upscale (small/distant codes),
              native 1×, 0.5× downscale (large close-up codes).

          Pass 2 — CLAHE → sharpen → multi-scale pyzbar:
              Last resort for genuinely low-contrast or very distant codes
              where extra edge emphasis is needed.  Original pipeline,
              demoted to third place.

          Pass 3 — _crop_zoom_decode (detect → crop → 4× zoom):
              cv2.QRCodeDetector locates the code even when decoding fails;
              cropped region upscaled 4× for a second decode attempt.
              Key for distant SIDE-B/C labels (tens of pixels at 1280×720).

        ── Bottom camera (Xiongmai /dev/video2) ─────────────────────────

          Pass 0 — raw pyzbar, NO preprocessing:
              Same rationale as the front camera.  In open-air/close-up
              test and competition-pool approach conditions this is all
              that is needed.  The dome/glare passes below are preserved
              for actual underwater operation.

          Pass 1 — CLAHE only → pyzbar:
              Uneven lighting correction without binarisation artefacts.

          Pass 2 — CLAHE → adaptive threshold → pyzbar:
              Dome glare / underwater haze binarisation.  blockSize=31
              is now a fallback (not first) because it over-segments large
              close-up QR modules — a 31-pixel block is smaller than one
              QR module when the code fills most of the frame, causing
              inconsistent local thresholding that confuses the finder
              pattern scanner.  Effective for the dome scenario where
              diffuse glare is the dominant problem.

          Pass 3 — _crop_zoom_decode (same fallback as front).

        Returns:
            List of overlay dicts in display-frame coordinates (see
            _draw_qr_overlays).  Empty list = nothing decoded this cycle.
        """
        if fast and not self._fast_detect(gray):
            return []

        # CLAHE applied once, shared by Pass 1+ of both cameras.
        # MUST remain outside the 'if front' block so that the bottom
        # camera branch can reference frame_qr without a NameError.
        # (Bug 2026-09-13: moving this inside the front block silently
        # killed the bottom camera QR worker thread on every decode call.)
        frame_qr = self._clahe.apply(gray)

        if 'front' in self.label.lower():
            # ── Pass 0: raw pyzbar — no preprocessing ────────────────────
            # Clear, close-up QR decodes without any enhancement.
            overlays = [
                self._pyzbar_to_overlay(obj, 1.0)
                for obj in pyzbar_decode(gray)]
            if overlays:
                return overlays

            # ── Pass 1: CLAHE only — no sharpen ──────────────────────────
            # Dim/uneven lighting without clipping finder-pattern edges.
            # 2× upscale for distant codes; 0.5× for large close-up codes.
            up2_c = cv2.resize(frame_qr, None, fx=2.0, fy=2.0,
                               interpolation=cv2.INTER_CUBIC)
            down05_c = cv2.resize(frame_qr, None, fx=0.5, fy=0.5,
                                  interpolation=cv2.INTER_AREA)
            for img, back in ((up2_c, 0.5), (frame_qr, 1.0), (down05_c, 2.0)):
                overlays = [
                    self._pyzbar_to_overlay(obj, back)
                    for obj in pyzbar_decode(img)]
                if overlays:
                    return overlays

            # ── Pass 2: CLAHE → sharpen → multi-scale ────────────────────
            # Last resort for very low-contrast or distant codes where
            # CLAHE alone is insufficient.
            sharpen_kernel = np.array(
                [[-1, -1, -1],
                 [-1,  9, -1],
                 [-1, -1, -1]], dtype=np.float32)
            sharp = cv2.filter2D(frame_qr, -1, sharpen_kernel)
            up2 = cv2.resize(sharp, None, fx=2.0, fy=2.0,
                             interpolation=cv2.INTER_CUBIC)
            down05 = cv2.resize(sharp, None, fx=0.5, fy=0.5,
                                interpolation=cv2.INTER_AREA)
            for img, back in ((up2, 0.5), (sharp, 1.0), (down05, 2.0)):
                overlays = [
                    self._pyzbar_to_overlay(obj, back)
                    for obj in pyzbar_decode(img)]
                if overlays:
                    return overlays

            # ── Pass 3: regional detect → crop → 4× zoom ─────────────────
            return self._crop_zoom_decode(sharp)

        # ════════════════════════════════════════════════════════════════
        #  Bottom camera (Xiongmai /dev/video2)
        # ════════════════════════════════════════════════════════════════

        # ── Pass 0: raw pyzbar — no preprocessing ────────────────────────
        # Open-air / close-up conditions: raw frame is all that is needed.
        # Avoids over-processing a clear image the same way the old front
        # camera pipeline was degrading already-good frames.
        overlays = [
            self._pyzbar_to_overlay(obj, 1.0)
            for obj in pyzbar_decode(gray)]
        if overlays:
            return overlays

        # ── Pass 1: CLAHE only — no binarisation ─────────────────────────
        # Uneven lighting without the module-boundary artefacts that
        # adaptive thresholding introduces on already-clear images.
        overlays = [
            self._pyzbar_to_overlay(obj, 1.0)
            for obj in pyzbar_decode(frame_qr)]
        if overlays:
            return overlays

        # ── Pass 2: CLAHE → adaptive threshold → pyzbar ──────────────────
        # Dome glare / underwater haze binarisation.  Demoted to fallback
        # because blockSize=31 over-segments large close-up QR modules
        # (a 31-px block is smaller than one module when the code fills
        # the frame, causing noisy local thresholds on solid cells).
        thr = cv2.adaptiveThreshold(
            frame_qr, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY, 31, 2)
        overlays = [
            self._pyzbar_to_overlay(obj, 1.0)
            for obj in pyzbar_decode(thr)]
        if overlays:
            return overlays

        # ── Pass 3: regional detect → crop → 4× zoom ─────────────────────
        return self._crop_zoom_decode(frame_qr)

    def _crop_zoom_decode(self, enhanced):
        """
        Detect-then-crop-and-zoom fallback for small/distant QR codes.

        cv2.QRCodeDetector can LOCATE a code (4 corner points) even when its
        own decoder — and pyzbar — fail on the small code.  Crop that region
        with a margin, upscale it 4×, then let pyzbar and QRCodeDetector
        decode the zoomed crop.  This is the key pathway for distant zone
        labels (e.g. SIDE-B / SIDE-C tags fixed to pool structures), which
        occupy only a few tens of pixels in a 1280×720 frame.

        Args:
            enhanced: contrast-enhanced grayscale image to probe.

        Returns:
            List of overlay dicts in *original* (enhanced) coordinates.
            Empty list if no code is located/decoded.
        """
        probes = [enhanced]
        probes.append(cv2.resize(enhanced, None, fx=0.5, fy=0.5,
                                 interpolation=cv2.INTER_AREA))
        for probe in probes:
            ok, pts = self._qr_detector.detect(probe)
            if not ok or pts is None or len(pts) != 4:
                continue
            back = 1.0 if probe is enhanced else 2.0
            xs = [p[0][0] for p in pts]
            ys = [p[0][1] for p in pts]
            x0, y0 = int(min(xs) * back), int(min(ys) * back)
            x1, y1 = int(max(xs) * back), int(max(ys) * back)
            margin = int(max(x1 - x0, y1 - y0) * 0.4) + 8
            x0c, y0c = max(0, x0 - margin), max(0, y0 - margin)
            x1c = min(enhanced.shape[1], x1 + margin)
            y1c = min(enhanced.shape[0], y1 + margin)
            crop = enhanced[y0c:y1c, x0c:x1c]
            if crop.size == 0:
                continue
            zoom = cv2.resize(crop, None, fx=4.0, fy=4.0,
                              interpolation=cv2.INTER_CUBIC)
            for obj in pyzbar_decode(zoom):
                overlay = self._pyzbar_to_overlay(obj, 1.0 / 4.0)
                rx, ry, rw, rh = overlay['rect']
                overlay['rect'] = (rx + x0c, ry + y0c, rw, rh)
                overlay['polygon'] = [
                    (px + x0c, py + y0c) for px, py in overlay['polygon']]
                return [overlay]
            data, pts2, _ = self._qr_detector.detectAndDecode(zoom)
            if data and pts2 is not None and len(pts2) == 4:
                polygon = [
                    (int(p[0][0] * 0.25) + x0c, int(p[0][1] * 0.25) + y0c)
                    for p in pts2]
                xs2 = [p[0] for p in polygon]
                ys2 = [p[1] for p in polygon]
                return [{'data': data,
                         'rect': (min(xs2), min(ys2),
                                  max(xs2) - min(xs2), max(ys2) - min(ys2)),
                         'polygon': polygon}]
        return []

    def _qr_worker_loop(self):
        """
        Dedicated QR decode worker — one per camera.

        Waits for the capture loop to hand off a new grayscale frame (5 Hz),
        then runs the expensive per-camera decode pipeline (_decode_qr_pipeline)
        OFF the capture thread.  Only the newest pending input is processed,
        so intermediate frames are skipped if decoding runs slower than the
        hand-off cadence.

        Results are cached under the main lock so the capture thread can draw
        overlays (and ROS can publish) without racing this worker.

        This split fixed the 2026-09-12 regression where the inline CLAHE/
        upscale/adaptive-threshold chain throttled capture to ~1 fps.
        """
        while self._running:
            self._qr_wake.wait(timeout=0.5)
            self._qr_wake.clear()
            with self._qr_lock:
                gray = self._qr_input
                self._qr_input = None
            if gray is None:
                continue

            decoded_objects = self._decode_qr_pipeline(gray)

            with self._lock:
                if decoded_objects:
                    self._latest_qr_objects = decoded_objects
                    self._last_qr_found_at = time.monotonic()
                    # Take the first detected QR code for ROS publish
                    qr_data = decoded_objects[0]['data']
                    if qr_data:
                        self._latest_qr_data = qr_data
                elif time.monotonic() - self._last_qr_found_at > 2.0:
                    # No QR for 2 s → drop the sticky overlay
                    self._latest_qr_objects = []

    def _draw_qr_overlays(self, frame, decoded_objects):
        """
        Draw bounding boxes and text labels for detected QR codes directly
        onto the BGR frame (in-place).  Called on every frame so overlays
        persist between detection cycles at full stream FPS.

        Args:
            frame: OpenCV BGR image (modified in-place).
            decoded_objects: List of overlay dicts:
                {'data': str, 'rect': (x, y, w, h),
                 'polygon': [(x0, y0), (x1, y1), ...]}
                with coordinates already in *display* frame space (the
                decode pipeline scales upscaled detections back down before
                caching).
        """
        for obj in decoded_objects:
            # ── Draw polygon boundary (precise corner points) ────────
            polygon = obj.get('polygon')
            if polygon and len(polygon) == 4:
                pts_array = np.array(polygon, dtype=np.int32)
                cv2.polylines(frame, [pts_array], isClosed=True,
                              color=(0, 255, 0), thickness=2)

            # ── Draw axis-aligned bounding rectangle ─────────────────
            x, y, w, h = obj.get('rect', (0, 0, 0, 0))
            cv2.rectangle(frame, (x, y), (x + w, y + h),
                          color=(0, 255, 0), thickness=2)

            # ── Draw label text with dark background ─────────────────
            text = obj.get('data', 'QR')
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
        """Start the background capture and QR worker threads."""
        self._running = True
        self._thread = threading.Thread(
            target=self._capture_loop, name=f'cam-{self.label}', daemon=True)
        self._thread.start()
        self._qr_thread = threading.Thread(
            target=self._qr_worker_loop, name=f'qrd-{self.label}', daemon=True)
        self._qr_thread.start()

    def stop(self):
        """Stop capture, QR worker, and release resources."""
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=3.0)
        self._qr_wake.set()   # unblock the QR worker so it can exit
        if self._qr_thread is not None:
            self._qr_thread.join(timeout=3.0)
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

            # ── Front (JETE-W7) image tuning ──────────────────────────
            # V4L2 diagnostics show this fixed-focus lens ships with
            # sharpness=5/31, which softens small distant QR labels.  Bump
            # edge contrast on the front camera; the bottom (Xiongmai)
            # captures through a sealed dome and stays at defaults.
            if 'front' in self.label.lower():
                cap.set(cv2.CAP_PROP_SHARPNESS, 25)
                cap.set(cv2.CAP_PROP_SHARPNESS, 25)   # some drivers need a 2nd write

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

            # Capture a single timestamp for this iteration.  Used for both
            # the QR-scan throttle and the FPS window — keeping them on the
            # same reference avoids double-calling time.monotonic() and
            # ensures the measured FPS reflects the true inter-frame rate
            # rather than frame-read + processing time.
            loop_now = time.monotonic()

            # ── Front camera is physically mounted upside-down ──────────
            # Rotate 180° so the stream, overlays, QR detection, and the
            # ROS image_raw topic are all upright.  (The Xiongmai bottom
            # dome feeds upright already — verified 2026-09-12: with the
            # rotation on the wrong camera BOTH feeds rendered upside-down.)
            if 'front' in self.label.lower():
                frame = cv2.rotate(frame, cv2.ROTATE_180)

            # ── Snapshot the CLEAN frame for ROS image publish ──────────
            # hook_detection_node consumes /ryugu/camera/*/image_raw and
            # must never see the QR/AI overlays drawn for the GCS — the
            # detector would otherwise re-detect its own boxes (feedback).
            raw_frame = frame.copy()

            # ── QR code detection (throttled to 5 Hz) ────────────────────
            # Only the cheap grayscale conversion happens here; the heavy
            # decode pipeline runs in the QR worker thread (_qr_worker_loop)
            # so expensive CLAHE/upscale/adaptive-threshold work never
            # throttles the capture loop's FPS.
            if loop_now - self._last_qr_scan_time >= self._qr_scan_interval:
                self._last_qr_scan_time = loop_now
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                with self._qr_lock:
                    self._qr_input = gray
                self._qr_wake.set()

            # ── Draw QR bounding-box overlays on every frame ───────────
            with self._lock:
                qr_overlays = list(self._latest_qr_objects)
            if qr_overlays:
                self._draw_qr_overlays(frame, qr_overlays)

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
            if self._last_fps_time == 0.0:
                self._last_fps_time = loop_now
            elif loop_now - self._last_fps_time >= 1.0:
                elapsed = loop_now - self._last_fps_time
                with self._lock:
                    self._fps_actual = self._fps_counter / elapsed
                    self._fps_counter = 0
                self._last_fps_time = loop_now


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
        """
        Send a continuous multipart MJPEG stream.

        Poll-based pacing: a frame is served only when the capture thread has
        advanced frame_count (i.e. a genuinely NEW frame), so the delivery
        rate matches the cameras' real capture rate instead of blindly
        re-serving stale JPEGs at stream_fps.  While the camera is
        disconnected the cached placeholder is re-sent at stream_fps so
        clients still see the offline state update.
        """
        self.send_response(200)
        self.send_header(
            'Content-Type',
            f'multipart/x-mixed-replace; boundary={BOUNDARY.decode()}')
        self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
        self.send_header('Pragma', 'no-cache')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()

        interval = 1.0 / max(1, self.stream_fps)
        poll_period = min(interval, 0.05)
        last_count = -1   # force-send the first available frame

        try:
            while True:
                frame = self.camera.frame_jpeg
                count = self.camera.frame_count
                if count != last_count or not self.camera.connected:
                    self.wfile.write(BOUNDARY + b'\r\n')
                    self.wfile.write(b'Content-Type: image/jpeg\r\n')
                    self.wfile.write(
                        f'Content-Length: {len(frame)}\r\n'.encode())
                    self.wfile.write(b'\r\n')
                    self.wfile.write(frame)
                    self.wfile.write(b'\r\n')
                    self.wfile.flush()
                    last_count = count
                time.sleep(poll_period)
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
        self.declare_parameter('front_port', 8555)   # GCS STREAM_URL_FRONT  = :8555
        self.declare_parameter('bottom_port', 8554)  # GCS STREAM_URL_BOTTOM = :8554
        self.declare_parameter('front_dev', '/dev/video0')   # JETE-W7    — physically front-facing
        self.declare_parameter('bottom_dev', '/dev/video2')  # Xiongmai — physically bottom-facing
        # ── Per-camera resolution ──
        self.declare_parameter('front_width', 1280)
        self.declare_parameter('front_height', 720)
        self.declare_parameter('bottom_width', 1280)
        self.declare_parameter('bottom_height', 720)
        # Legacy shared params (kept for backwards-compat; overridden per-camera above)
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('fps', 30)
        self.declare_parameter('jpeg_quality', 80)
        self.declare_parameter('bottom_jpeg_quality', 80)  # Higher quality for QR precision
        self.declare_parameter('publish_raw_images', True)

        bind_ip      = self.get_parameter('bind').get_parameter_value().string_value
        front_port   = self.get_parameter('front_port').get_parameter_value().integer_value
        bottom_port  = self.get_parameter('bottom_port').get_parameter_value().integer_value
        front_dev    = self.get_parameter('front_dev').get_parameter_value().string_value
        bottom_dev   = self.get_parameter('bottom_dev').get_parameter_value().string_value
        front_width  = self.get_parameter('front_width').get_parameter_value().integer_value
        front_height = self.get_parameter('front_height').get_parameter_value().integer_value
        bottom_width  = self.get_parameter('bottom_width').get_parameter_value().integer_value
        bottom_height = self.get_parameter('bottom_height').get_parameter_value().integer_value
        fps          = self.get_parameter('fps').get_parameter_value().integer_value
        quality      = self.get_parameter('jpeg_quality').get_parameter_value().integer_value
        bottom_quality = self.get_parameter('bottom_jpeg_quality').get_parameter_value().integer_value

        # ── Device availability check ───────────────────────────────────
        for dev, name in [(front_dev, 'Front'), (bottom_dev, 'Bottom')]:
            if os.path.exists(dev):
                self.get_logger().info(f'{name} camera found: {dev}')
            else:
                self.get_logger().warn(
                    f'{name} camera device {dev} not found — '
                    f'will serve placeholder')

        # ── Create camera capture instances ─────────────────────────────
        # Front camera (JETE-W7 /dev/video0): 1280×720 @ 30fps
        self._front_cam = CameraCapture(
            device=front_dev, label='Front Camera',
            width=front_width, height=front_height, fps=fps,
            jpeg_quality=quality)
        # Bottom camera (Xiongmai /dev/video2): 1280×720 @ 30fps, higher JPEG quality for QR
        self._bottom_cam = CameraCapture(
            device=bottom_dev, label='Bottom Camera',
            width=bottom_width, height=bottom_height, fps=fps,
            jpeg_quality=bottom_quality)

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
            f'Front (JETE-W7 /dev/video0): {front_width}x{front_height} @ {fps}fps, Q{quality} | '
            f'Bottom (Xiongmai /dev/video2): {bottom_width}x{bottom_height} @ {fps}fps, Q{bottom_quality}')

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
