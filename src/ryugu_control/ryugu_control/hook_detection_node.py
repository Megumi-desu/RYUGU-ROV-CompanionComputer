#!/usr/bin/env python3
"""
hook_detection_node.py — Real-Time ROV Hook Detection via TensorRT

Performs real-time underwater ROV payload hook detection using a compiled
YOLO-NAS TensorRT engine (FP16) on the Jetson Orin Nano.

Subscribes to raw camera streams published by webcam_streamer and runs
GPU-accelerated inference to detect hook components and targets.

Model:
  - YOLO-NAS with TensorRT FP16 engine
  - Input:  [1, 3, 640, 640] NCHW float32, RGB, RAW pixel values in
             [0, 255] — NO normalisation.  The Versi 2 model was trained
             with the stock YOLO-NAS recipe (DetectionMosaic +
             DetectionPaddedRescale) which does NOT standardise the
             image.  Feeding x/255.0 (or ImageNet mean/std) suppresses
             every score to ~0.09 (verified 2026-09-02 with
             scripts/test_tensorrt_inference.py).
  - Output: boxes [1, 8400, 4] (x1, y1, x2, y2)
  -         scores [1, 8400, 3] (class probabilities — the exported
             ONNX already applies sigmoid, so [0.0, 1.0].  Do NOT
             re-apply sigmoid; postprocess() detects activation level
             automatically.)

Classes (default):
  0: hook_body_grey
  1: hook_body_white
  2: hook_target

Subscriptions:
  /ryugu/camera/front/image_raw   (sensor_msgs/Image)
  /ryugu/camera/bottom/image_raw  (sensor_msgs/Image)

Publishers:
  /ryugu/vision/hook_target             (Float32MultiArray)  — target centroid + bbox
    [camera_id, cx, cy, bw, bh, conf, class_id, detected_flag, x1, y1, x2, y2]
  /ryugu/vision/hook_detections         (Detection2DArray)   — standard dets
  /ryugu/vision/hook_debug_image/compressed (CompressedImage) — annotated debug

Usage:
  ros2 run ryugu_control hook_detection_node
  ros2 run ryugu_control hook_detection_node --ros-args \
      -p conf_threshold:=0.6 -p active_camera:=both
"""

import ctypes
import os
import threading
import time
from typing import List, Optional, Tuple

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout
from cv_bridge import CvBridge
from rcl_interfaces.msg import SetParametersResult

import importlib

# Dynamic vision_msgs import (prevents IDE linter errors when package is missing)
try:
    _vm = importlib.import_module('vision_msgs.msg')
    Detection2D = _vm.Detection2D
    Detection2DArray = _vm.Detection2DArray
    ObjectHypothesisWithPose = _vm.ObjectHypothesisWithPose
    HAS_VISION_MSGS = True
except Exception:
    Detection2D = None
    Detection2DArray = None
    ObjectHypothesisWithPose = None
    HAS_VISION_MSGS = False

# ── Load CUDA Runtime Library (libcudart.so) via ctypes ───────────────────────
_cudart = None
for lib_name in (
    'libcudart.so',
    'libcudart.so.12',
    'libcudart.so.11.0',
    '/usr/local/cuda/lib64/libcudart.so',
):
    try:
        _cudart = ctypes.CDLL(lib_name)
        break
    except Exception:
        pass

if _cudart is not None:
    _cudart.cudaMalloc.argtypes = [ctypes.POINTER(ctypes.c_void_p), ctypes.c_size_t]
    _cudart.cudaMalloc.restype = ctypes.c_int
    _cudart.cudaFree.argtypes = [ctypes.c_void_p]
    _cudart.cudaFree.restype = ctypes.c_int
    _cudart.cudaMemcpyAsync.argtypes = [
        ctypes.c_void_p, ctypes.c_void_p, ctypes.c_size_t, ctypes.c_int, ctypes.c_void_p
    ]
    _cudart.cudaMemcpyAsync.restype = ctypes.c_int
    _cudart.cudaStreamCreate.argtypes = [ctypes.POINTER(ctypes.c_void_p)]
    _cudart.cudaStreamCreate.restype = ctypes.c_int
    _cudart.cudaStreamDestroy.argtypes = [ctypes.c_void_p]
    _cudart.cudaStreamDestroy.restype = ctypes.c_int
    _cudart.cudaStreamSynchronize.argtypes = [ctypes.c_void_p]
    _cudart.cudaStreamSynchronize.restype = ctypes.c_int

CUDA_MEMCPY_HOST_TO_DEVICE = 1
CUDA_MEMCPY_DEVICE_TO_HOST = 2

# TensorRT import
try:
    import tensorrt as trt
    HAS_TENSORRT = _cudart is not None
except ImportError:
    HAS_TENSORRT = False

# ═══════════════════════════════════════════════════════════════════════════════
#  Constants
# ═══════════════════════════════════════════════════════════════════════════════
INPUT_H, INPUT_W = 640, 640
INPUT_SHAPE = (1, 3, INPUT_H, INPUT_W)
NUM_CANDIDATES = 8400

# Per-class colours for debug annotation (BGR)
CLASS_COLOURS = [
    (180, 180, 180),  # hook_body_grey  — silver
    (255, 255, 255),  # hook_body_white — white
    (0, 200, 255),    # hook_target     — gold/orange
]


# ═══════════════════════════════════════════════════════════════════════════════
#  TensorRT Engine Wrapper (Ctypes CUDA memory management)
# ═══════════════════════════════════════════════════════════════════════════════
class HostDeviceMem:
    """Container for matched CPU host array and GPU device memory pointer."""

    def __init__(self, name: str, shape: Tuple[int, ...], dtype: np.dtype):
        self.name = name
        self.shape = shape
        self.dtype = dtype
        self.size = int(np.prod(shape))
        self.nbytes = self.size * np.dtype(dtype).itemsize

        self.host = np.zeros(shape, dtype=dtype)
        self.device = ctypes.c_void_p()

        status = _cudart.cudaMalloc(ctypes.byref(self.device), self.nbytes)
        if status != 0:
            raise RuntimeError(f'cudaMalloc failed for tensor {name} with code {status}')

    def free(self):
        if self.device and self.device.value:
            _cudart.cudaFree(self.device)
            self.device = ctypes.c_void_p()

    def __repr__(self):
        return (f'HostDeviceMem(name={self.name}, shape={self.shape}, '
                f'dtype={self.dtype}, device=0x{self.device.value:x})')


class TRTEngine:
    """
    Manages a TensorRT engine lifecycle: loading, buffer allocation,
    synchronous inference execution, and cleanup.
    """

    def __init__(self, engine_path: str, logger=None):
        self._logger = logger
        self._engine = None
        self._context = None
        self._stream = ctypes.c_void_p()

        status = _cudart.cudaStreamCreate(ctypes.byref(self._stream))
        if status != 0:
            raise RuntimeError(f'cudaStreamCreate failed with code {status}')

        self._inputs: List[HostDeviceMem] = []
        self._outputs: List[HostDeviceMem] = []

        self._load_engine(engine_path)
        self._allocate_buffers()

    def _log(self, msg: str):
        if self._logger:
            self._logger.info(msg)

    def _load_engine(self, engine_path: str):
        """Deserialise the TensorRT engine from a file."""
        trt_logger = trt.Logger(trt.Logger.WARNING)
        with open(engine_path, 'rb') as f:
            runtime = trt.Runtime(trt_logger)
            self._engine = runtime.deserialize_cuda_engine(f.read())
        assert self._engine is not None, f'Failed to load engine: {engine_path}'
        self._context = self._engine.create_execution_context()
        assert self._context is not None, 'Failed to create execution context'
        self._log(f'TRT engine loaded: {engine_path}')

    def _allocate_buffers(self):
        """Allocate device memory for all engine bindings."""
        for i in range(self._engine.num_io_tensors):
            name = self._engine.get_tensor_name(i)
            shape = tuple(self._engine.get_tensor_shape(name))
            dtype = trt.nptype(self._engine.get_tensor_dtype(name))
            mode = self._engine.get_tensor_mode(name)

            if mode == trt.TensorIOMode.INPUT:
                self._context.set_input_shape(name, INPUT_SHAPE)
                shape = INPUT_SHAPE

            mem = HostDeviceMem(name, shape, dtype)

            if mode == trt.TensorIOMode.INPUT:
                self._inputs.append(mem)
            else:
                self._outputs.append(mem)

            self._context.set_tensor_address(name, mem.device.value)

            self._log(f'  Tensor [{name}]: shape={shape}, dtype={dtype}, '
                      f'mode={"INPUT" if mode == trt.TensorIOMode.INPUT else "OUTPUT"}')

    def infer(self, input_data: np.ndarray) -> List[np.ndarray]:
        """
        Run synchronous inference on the GPU.

        Args:
            input_data: Pre-processed NCHW float32 array matching INPUT_SHAPE.

        Returns:
            List of output numpy arrays (boxes, scores).
        """
        np.copyto(self._inputs[0].host, input_data.astype(self._inputs[0].dtype))

        # Copy Host -> Device
        for inp in self._inputs:
            _cudart.cudaMemcpyAsync(
                inp.device,
                inp.host.ctypes.data,
                inp.nbytes,
                CUDA_MEMCPY_HOST_TO_DEVICE,
                self._stream,
            )

        # Execute TensorRT execution context
        self._context.execute_async_v3(stream_handle=self._stream.value)

        # Copy Device -> Host
        for out in self._outputs:
            _cudart.cudaMemcpyAsync(
                out.host.ctypes.data,
                out.device,
                out.nbytes,
                CUDA_MEMCPY_DEVICE_TO_HOST,
                self._stream,
            )

        # Synchronise stream
        _cudart.cudaStreamSynchronize(self._stream)

        return [out.host.copy() for out in self._outputs]

    def destroy(self):
        """Free GPU memory and streams."""
        for mem in self._inputs + self._outputs:
            mem.free()
        self._inputs.clear()
        self._outputs.clear()
        if self._stream and self._stream.value:
            _cudart.cudaStreamDestroy(self._stream)
            self._stream = ctypes.c_void_p()
        self._log('TRT engine resources freed.')


# ═══════════════════════════════════════════════════════════════════════════════
#  Pre / Post Processing
# ═══════════════════════════════════════════════════════════════════════════════
def preprocess(frame: np.ndarray) -> Tuple[np.ndarray, Tuple[int, int]]:
    """
    Prepare a BGR OpenCV frame for YOLO-NAS TensorRT inference.

    The deployed engine (Versi 2) expects RAW float32 pixels in
    [0, 255] — the training recipe does not standardise the image, so
    dividing by 255.0 or applying ImageNet mean/std here suppresses all
    detections (top score ≈ 0.09 instead of 0.5–0.7).  See
    scripts/test_tensorrt_inference.py for the verification matrix.

    Returns:
        (input_tensor, original_shape) where input_tensor is NCHW float32.
    """
    orig_h, orig_w = frame.shape[:2]
    # Resize to model input dimensions
    resized = cv2.resize(frame, (INPUT_W, INPUT_H), interpolation=cv2.INTER_LINEAR)
    # BGR → RGB
    rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
    # Raw pixels as float32 — NO /255.0, NO mean/std (Versi 2 recipe)
    x = rgb.astype(np.float32)
    # HWC → NCHW
    nchw = np.transpose(x, (2, 0, 1))[np.newaxis, ...]
    return np.ascontiguousarray(nchw), (orig_h, orig_w)


def postprocess(
    raw_boxes: np.ndarray,
    raw_scores: np.ndarray,
    orig_shape: Tuple[int, int],
    conf_threshold: float = 0.40,
    iou_threshold: float = 0.45,
    num_classes: int = 3,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Post-process YOLO-NAS outputs: activation check, filter, NMS, scale.

    Args:
        raw_boxes:  [1, 8400, 4] — x1, y1, x2, y2 in 640×640 coords.
        raw_scores: [1, 8400, num_classes] — class scores.  The exported
                    ONNX model already applies sigmoid, so these are
                    probabilities in [0.0, 1.0]; the activation check
                    below still handles logit outputs defensively.
        orig_shape: (orig_h, orig_w) of the source frame.
        conf_threshold: Minimum confidence to keep.
        iou_threshold:  IoU threshold for NMS.
        num_classes: Number of detection classes.

    Returns:
        (boxes, scores, class_ids) — arrays of surviving detections
        (max 10, highest confidence first).  boxes are in original image
        coordinates [x1, y1, x2, y2].
    """
    orig_h, orig_w = orig_shape
    sx = orig_w / INPUT_W
    sy = orig_h / INPUT_H

    # Remove batch dimension
    boxes = raw_boxes[0]     # [8400, 4]
    raw_class_scores = raw_scores[0]   # [8400, num_classes]

    # ── Smart probability check ─────────────────────────────────────────
    # The exported YOLO-NAS ONNX already applies sigmoid: scores arrive in
    # [0.0, 1.0].  Re-applying sigmoid there would map background anchors
    # (≈0.0) to exactly 0.50, flooding the output with false "50%" boxes.
    # If any value falls outside [0, 1] the engine is emitting raw logits
    # and sigmoid IS required.
    if raw_class_scores.min() < 0.0 or raw_class_scores.max() > 1.0:
        scores_all = _sigmoid(raw_class_scores)
    else:
        scores_all = raw_class_scores  # Already probabilities [0.0 .. 1.0]!

    # Per-candidate: best class and its score
    class_ids = np.argmax(scores_all, axis=1)           # [8400]
    confidences = scores_all[np.arange(len(class_ids)), class_ids]  # [8400]

    # Filter by confidence
    mask = confidences >= conf_threshold
    if not np.any(mask):
        return (np.empty((0, 4), dtype=np.float32),
                np.empty((0,), dtype=np.float32),
                np.empty((0,), dtype=np.int32))

    filt_boxes = boxes[mask]          # [N, 4]
    filt_scores = confidences[mask]   # [N]
    filt_classes = class_ids[mask]    # [N]

    # Scale boxes to original image coords
    filt_boxes[:, [0, 2]] *= sx
    filt_boxes[:, [1, 3]] *= sy

    # Convert x1y1x2y2 → xywh for OpenCV NMS
    x1 = filt_boxes[:, 0]
    y1 = filt_boxes[:, 1]
    w = filt_boxes[:, 2] - filt_boxes[:, 0]
    h = filt_boxes[:, 3] - filt_boxes[:, 1]
    xywh = np.stack([x1, y1, w, h], axis=1)

    # OpenCV NMS
    indices = cv2.dnn.NMSBoxes(
        xywh.tolist(),
        filt_scores.tolist(),
        conf_threshold,
        iou_threshold,
    )

    if len(indices) == 0:
        return (np.empty((0, 4), dtype=np.float32),
                np.empty((0,), dtype=np.float32),
                np.empty((0,), dtype=np.int32))

    indices = indices.flatten()

    # ── Cap NMS results at max_det (10) highest-confidence detections ──
    max_det = 10
    if len(indices) > max_det:
        top = np.argsort(filt_scores[indices])[::-1][:max_det]
        indices = indices[top]

    return (filt_boxes[indices],
            filt_scores[indices],
            filt_classes[indices].astype(np.int32))


def _sigmoid(x: np.ndarray) -> np.ndarray:
    """Numerically stable sigmoid."""
    return np.where(
        x >= 0,
        1.0 / (1.0 + np.exp(-x)),
        np.exp(x) / (1.0 + np.exp(x)),
    )


# ═══════════════════════════════════════════════════════════════════════════════
#  Debug Image Annotation
# ═══════════════════════════════════════════════════════════════════════════════
def annotate_frame(
    frame: np.ndarray,
    boxes: np.ndarray,
    scores: np.ndarray,
    class_ids: np.ndarray,
    class_names: List[str],
    target_class_id: int,
    fps: float,
) -> np.ndarray:
    """Draw bounding boxes, labels, target centroid, and FPS on a frame."""
    vis = frame.copy()

    for i in range(len(boxes)):
        x1, y1, x2, y2 = boxes[i].astype(int)
        cls_id = int(class_ids[i])
        conf = float(scores[i])
        colour = CLASS_COLOURS[cls_id % len(CLASS_COLOURS)]
        name = class_names[cls_id] if cls_id < len(class_names) else f'cls_{cls_id}'

        # Bounding box
        cv2.rectangle(vis, (x1, y1), (x2, y2), colour, 2)

        # Label with confidence
        label = f'{name} {conf:.0%}'
        (tw, th), baseline = cv2.getTextSize(
            label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        label_y = max(y1 - 6, th + 6)
        cv2.rectangle(vis, (x1, label_y - th - 4),
                      (x1 + tw + 4, label_y + baseline), (0, 0, 0), -1)
        cv2.putText(vis, label, (x1 + 2, label_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 1, cv2.LINE_AA)

        # Target centroid crosshair
        if cls_id == target_class_id:
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)
            cv2.circle(vis, (cx, cy), 8, (0, 0, 255), 2)
            cv2.line(vis, (cx - 12, cy), (cx + 12, cy), (0, 0, 255), 1)
            cv2.line(vis, (cx, cy - 12), (cx, cy + 12), (0, 0, 255), 1)
            cv2.putText(vis, f'({cx},{cy})', (cx + 10, cy - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1,
                        cv2.LINE_AA)

    # FPS overlay (top-left)
    fps_text = f'FPS: {fps:.1f}'
    cv2.putText(vis, fps_text, (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA)

    return vis


# ═══════════════════════════════════════════════════════════════════════════════
#  ROS 2 Node
# ═══════════════════════════════════════════════════════════════════════════════
class HookDetectionNode(Node):
    """ROS 2 node for real-time hook detection using TensorRT."""

    CAMERA_FRONT = 0
    CAMERA_BOTTOM = 1

    def __init__(self):
        super().__init__('hook_detection_node')

        if not HAS_TENSORRT:
            self.get_logger().fatal(
                'TensorRT or libcudart.so not available in this environment.')
            raise RuntimeError('TensorRT/CUDA not available')

        # ── Declare ROS parameters ──────────────────────────────────────
        self.declare_parameter('engine_path', '')
        self.declare_parameter('conf_threshold', 0.70)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('class_names',
                               ['hook_body_grey', 'hook_body_white',
                                'hook_target'])
        self.declare_parameter('target_class', 'hook_target')
        self.declare_parameter('active_camera', 'front')
        self.declare_parameter('debug_jpeg_quality', 70)

        # ── Read parameters ─────────────────────────────────────────────
        engine_path = self.get_parameter(
            'engine_path').get_parameter_value().string_value

        # ── Engine path resolution (three-tier fallback) ────────────────
        # 1) Explicit engine_path parameter (if provided, used as-is).
        # 2) Source workspace models dir — reliable even when the colcon
        #    share symlink is stale or the package isn't installed.
        # 3) Installed package share directory (colcon-installed engine).
        if not engine_path:
            from ament_index_python.packages import get_package_share_directory

            engine_filename = 'yolo_nas_rov_hook_fp16.engine'
            src_engine = os.path.join(
                '/home/icad/RYUGU-ROV-CompanionComputer',
                'src', 'ryugu_control', 'models', engine_filename)
            share_engine = os.path.join(
                get_package_share_directory('ryugu_control'),
                'models', engine_filename)

            if os.path.isfile(src_engine):
                engine_path = src_engine
            else:
                engine_path = share_engine

        self._conf_threshold = self.get_parameter(
            'conf_threshold').get_parameter_value().double_value
        self._iou_threshold = self.get_parameter(
            'iou_threshold').get_parameter_value().double_value
        self._class_names = list(self.get_parameter(
            'class_names').get_parameter_value().string_array_value)
        self._num_classes = len(self._class_names)

        target_class = self.get_parameter(
            'target_class').get_parameter_value().string_value
        self._target_class_id = (
            self._class_names.index(target_class)
            if target_class in self._class_names else 2)

        self._active_camera = self.get_parameter(
            'active_camera').get_parameter_value().string_value.lower()
        self._debug_quality = self.get_parameter(
            'debug_jpeg_quality').get_parameter_value().integer_value

        self.get_logger().info(f'Engine path: {engine_path}')
        self.get_logger().info(f'Classes: {self._class_names}')
        self.get_logger().info(f'Target class: {target_class} '
                               f'(id={self._target_class_id})')
        self.get_logger().info(f'Active camera: {self._active_camera}')
        self.get_logger().info(f'Confidence threshold: {self._conf_threshold}')
        self.get_logger().info(f'IoU threshold: {self._iou_threshold}')

        # ── Dynamic parameter tuning ────────────────────────────────────
        # conf_threshold is tunable at runtime without rebuilding:
        #   ros2 param set /hook_detection_node conf_threshold 0.40
        self.add_on_set_parameters_callback(self._on_set_parameters)

        # ── Initialise TensorRT Engine ──────────────────────────────────
        try:
            self._engine = TRTEngine(engine_path, logger=self.get_logger())
        except Exception as e:
            raise RuntimeError(f'Failed to load TRT engine: {e}') from e

        # ── ROS utilities ───────────────────────────────────────────────
        self._cv_bridge = CvBridge()
        self._inference_lock = threading.Lock()

        # FPS tracking (exponential moving average)
        self._fps = 0.0
        self._fps_alpha = 0.1

        # Top-score readout throttle (live monitoring aid)
        self._last_score_log = 0.0

        # ── QoS for camera subscriptions ────────────────────────────────
        cam_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)

        self._infer_cbg = MutuallyExclusiveCallbackGroup()

        # ── Subscriptions ───────────────────────────────────────────────
        if self._active_camera in ('front', 'both'):
            self.create_subscription(
                Image, '/ryugu/camera/front/image_raw',
                self._front_image_cb, cam_qos,
                callback_group=self._infer_cbg)
            self.get_logger().info('Subscribed to front camera')

        if self._active_camera in ('bottom', 'both'):
            self.create_subscription(
                Image, '/ryugu/camera/bottom/image_raw',
                self._bottom_image_cb, cam_qos,
                callback_group=self._infer_cbg)
            self.get_logger().info('Subscribed to bottom camera')

        # ── Publishers ──────────────────────────────────────────────────
        self._target_pub = self.create_publisher(
            Float32MultiArray, '/ryugu/vision/hook_target', 10)
        self._detections_pub = (
            self.create_publisher(Detection2DArray, '/ryugu/vision/hook_detections', 10)
            if HAS_VISION_MSGS else None
        )
        self._debug_pub = self.create_publisher(
            CompressedImage, '/ryugu/vision/hook_debug_image/compressed', 1)

        self.get_logger().info(
            '═══ HookDetectionNode ready — waiting for camera frames ═══')

    def _on_set_parameters(self, params):
        """
        Handle runtime parameter updates (via `ros2 param set`).

        conf_threshold is applied live so the team can adapt to pool
        lighting conditions without rebuilding or restarting the node.
        """
        for param in params:
            if param.name == 'conf_threshold':
                value = float(param.value.double_value)
                if not 0.0 <= value <= 1.0:
                    self.get_logger().warn(
                        f'Rejected conf_threshold={value:.3f} — '
                        f'must be within [0.0, 1.0]')
                    return SetParametersResult(
                        successful=False,
                        reason='conf_threshold must be within [0.0, 1.0]')
                self._conf_threshold = value
                self.get_logger().info(
                    f'✅ Dynamic conf_threshold = {value:.2f} applied')
        return SetParametersResult(successful=True)

    def _front_image_cb(self, msg: Image):
        self._process_image(msg, self.CAMERA_FRONT)

    def _bottom_image_cb(self, msg: Image):
        self._process_image(msg, self.CAMERA_BOTTOM)

    def _process_image(self, msg: Image, camera_id: int):
        """Full inference pipeline: preprocess → infer → postprocess → publish."""
        t_start = time.monotonic()

        try:
            frame = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'CvBridge error: {e}')
            return

        input_tensor, orig_shape = preprocess(frame)

        with self._inference_lock:
            outputs = self._engine.infer(input_tensor)

        raw_boxes = None
        raw_scores = None
        for out in outputs:
            if out.shape[-1] == 4:
                raw_boxes = out
            elif out.shape[-1] == self._num_classes:
                raw_scores = out

        if raw_boxes is None or raw_scores is None:
            self.get_logger().warn(
                f'Unexpected output shapes: {[o.shape for o in outputs]}')
            return

        boxes, scores, class_ids = postprocess(
            raw_boxes, raw_scores, orig_shape,
            self._conf_threshold, self._iou_threshold, self._num_classes)

        # ── Top-score readout (every ~10 s) for live monitoring ────────
        # Shows the strongest raw score on this frame even when nothing
        # passes the threshold — the operator can tell at a glance whether
        # the model is firing at all (tune conf_threshold accordingly).
        now_mono = time.monotonic()
        if now_mono - self._last_score_log >= 10.0:
            self._last_score_log = now_mono
            top_raw = float(raw_scores.max())
            self.get_logger().info(
                f'📊 Detections: {len(boxes)} | top raw score={top_raw:.3f} '
                f'(threshold={self._conf_threshold:.2f})')

        dt = time.monotonic() - t_start
        instant_fps = 1.0 / dt if dt > 0 else 0.0
        self._fps = (self._fps_alpha * instant_fps +
                     (1.0 - self._fps_alpha) * self._fps)

        stamp = self.get_clock().now().to_msg()
        self._publish_target(boxes, scores, class_ids, camera_id, stamp)
        self._publish_detections(boxes, scores, class_ids, camera_id, stamp)
        self._publish_debug_image(frame, boxes, scores, class_ids, camera_id)

    def _publish_target(self, boxes, scores, class_ids, camera_id: int, stamp):
        """
        Publish target centroid and bounding box for navigation & overlay.

        Payload (12 floats):
          [camera_id, cx, cy, bw, bh, conf, class_id, detected_flag,
           x1, y1, x2, y2]

        detected_flag is 1.0 when a hook_target is visible, 0.0 otherwise.
        x1/y1/x2/y2 are the target box corners in original frame coords —
        consumed by webcam_streamer to draw overlays on the MJPEG stream.
        """
        msg = Float32MultiArray()
        msg.layout = MultiArrayLayout()
        msg.layout.dim = [MultiArrayDimension(
            label='hook_target', size=12, stride=12)]

        target_mask = class_ids == self._target_class_id
        if np.any(target_mask):
            target_boxes = boxes[target_mask]
            target_scores = scores[target_mask]
            best_idx = int(np.argmax(target_scores))

            x1, y1, x2, y2 = target_boxes[best_idx]
            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0
            bw = x2 - x1
            bh = y2 - y1
            conf = target_scores[best_idx]

            msg.data = [
                float(camera_id),
                float(cx),
                float(cy),
                float(bw),
                float(bh),
                float(conf),
                float(self._target_class_id),
                1.0,
                float(x1),
                float(y1),
                float(x2),
                float(y2),
            ]
        else:
            msg.data = [
                float(camera_id),
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                float(self._target_class_id),
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            ]

        self._target_pub.publish(msg)

    def _publish_detections(self, boxes, scores, class_ids, camera_id: int, stamp):
        """
        Publish standard Detection2DArray.

        header.frame_id carries the camera name ('front' / 'bottom') so
        consumers (webcam_streamer overlay) can route boxes to the right
        MJPEG stream.
        """
        if not HAS_VISION_MSGS or self._detections_pub is None:
            return

        det_array = Detection2DArray()
        det_array.header.stamp = stamp
        det_array.header.frame_id = (
            'front' if camera_id == self.CAMERA_FRONT else 'bottom')

        for i in range(len(boxes)):
            det = Detection2D()
            det.header = det_array.header

            x1, y1, x2, y2 = boxes[i]
            det.bbox.center.position.x = float((x1 + x2) / 2.0)
            det.bbox.center.position.y = float((y1 + y2) / 2.0)
            det.bbox.size_x = float(x2 - x1)
            det.bbox.size_y = float(y2 - y1)

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = str(int(class_ids[i]))
            hyp.hypothesis.score = float(scores[i])
            det.results.append(hyp)

            det_array.detections.append(det)

        self._detections_pub.publish(det_array)

    def _publish_debug_image(self, frame, boxes, scores, class_ids, camera_id: int):
        """Publish annotated debug image as CompressedImage (JPEG)."""
        vis = annotate_frame(
            frame, boxes, scores, class_ids,
            self._class_names, self._target_class_id, self._fps)

        cam_label = 'FRONT' if camera_id == self.CAMERA_FRONT else 'BOTTOM'
        cv2.putText(vis, f'CAM: {cam_label}', (10, vis.shape[0] - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 0), 1,
                    cv2.LINE_AA)

        _, jpeg = cv2.imencode(
            '.jpg', vis,
            [cv2.IMWRITE_JPEG_QUALITY, self._debug_quality])

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        msg.format = 'jpeg'
        msg.data = jpeg.tobytes()

        self._debug_pub.publish(msg)

    def destroy_node(self):
        """Clean shutdown: free GPU resources."""
        if rclpy.ok():
            self.get_logger().info('Shutting down HookDetectionNode...')
        try:
            self._engine.destroy()
        except Exception as e:
            if rclpy.ok():
                self.get_logger().warn(f'Error freeing TRT engine: {e}')
        super().destroy_node()


# ═══════════════════════════════════════════════════════════════════════════════
#  Entry point
# ═══════════════════════════════════════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)

    node = HookDetectionNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
