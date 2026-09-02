#!/usr/bin/env python3
"""
test_tensorrt_inference.py — Standalone YOLO-NAS TensorRT / ONNX diagnostic.

NO ROS 2 dependency.  Purpose: determine the correct preprocessing for the
deployed TensorRT engine (or an ONNX export) by comparing normalisation
modes on a sample image or a live webcam frame:

  Mode A "raw":       x.astype(np.float32) / 255.0      — what the ROS
                      pipeline (hook_detection_node) currently feeds the
                      engine
  Mode B "imagenet":  (x/255.0 - mean) / std            — ImageNet
                      mean/std standardisation, the SuperGradients
                      training-time default (DetectionPaddedRescale)
  Mode C "passthru":  x.astype(np.float32)              — raw 0-255 pixels;
                      only correct if normalisation was baked into the
                      ONNX graph (it is NOT for the current export)

For each configuration the script prints the top-10 raw class scores with
bounding boxes, runs NMS, and saves an annotated image.  In compare mode
the best configuration is additionally saved as test_output.jpg.

Usage:
  python3 scripts/test_tensorrt_inference.py                 # engine + default test image, compare all
  python3 scripts/test_tensorrt_inference.py --mode b        # ImageNet norm only
  python3 scripts/test_tensorrt_inference.py --webcam 0      # live webcam frame
  python3 scripts/test_tensorrt_inference.py --onnx /path/to/model.onnx
  python3 scripts/test_tensorrt_inference.py --letterbox     # also try letterbox resize (pad 114) for Mode B
  python3 scripts/test_tensorrt_inference.py --channels rgb  # RGB only (skip BGR cross-check)
"""

import argparse
import ctypes
import time
from pathlib import Path

import cv2
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_ENGINE = (REPO_ROOT / 'src' / 'ryugu_control' / 'models'
                  / 'yolo_nas_rov_hook_fp16.engine')
DEFAULT_IMAGE = Path('/home/icad/Downloads/ROV_YOLO-NAS_Model/Versi 2'
                     '/rov_yolo_nas_run/test_inference_samples.png')
DEFAULT_OUT = REPO_ROOT / 'scripts' / 'test_output.jpg'

INPUT_H = INPUT_W = 640
IMAGENET_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
IMAGENET_STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)
PAD_VALUE = 114  # SuperGradients DetectionBottomRightPadding pad value
DEFAULT_CLASSES = ['hook_body_grey', 'hook_body_white', 'hook_target']

MODE_LABELS = {
    'a': 'raw /255.0',
    'b': 'ImageNet (x/255-mean)/std',
    'c': 'passthru 0-255',
}
# Per-class BGR colours (match hook_detection_node / webcam_streamer)
CLASS_COLOURS = [
    (180, 180, 180),  # hook_body_grey  — silver
    (255, 255, 255),  # hook_body_white — white
    (0, 200, 255),    # hook_target     — gold/orange
]


# ═══════════════════════════════════════════════════════════════════════════════
#  CUDA runtime (ctypes) — same approach as hook_detection_node
# ═══════════════════════════════════════════════════════════════════════════════
def _load_cudart():
    """Load libcudart via ctypes.  Returns the lib handle or None."""
    for name in (
        'libcudart.so',
        'libcudart.so.12',
        'libcudart.so.11.0',
        '/usr/local/cuda/lib64/libcudart.so',
        '/usr/local/cuda-12.6/lib64/libcudart.so',
    ):
        try:
            rt = ctypes.CDLL(name)
            rt.cudaMalloc.argtypes = [
                ctypes.POINTER(ctypes.c_void_p), ctypes.c_size_t]
            rt.cudaMalloc.restype = ctypes.c_int
            rt.cudaFree.argtypes = [ctypes.c_void_p]
            rt.cudaFree.restype = ctypes.c_int
            rt.cudaMemcpyAsync.argtypes = [
                ctypes.c_void_p, ctypes.c_void_p, ctypes.c_size_t,
                ctypes.c_int, ctypes.c_void_p]
            rt.cudaMemcpyAsync.restype = ctypes.c_int
            rt.cudaStreamCreate.argtypes = [ctypes.POINTER(ctypes.c_void_p)]
            rt.cudaStreamCreate.restype = ctypes.c_int
            rt.cudaStreamDestroy.argtypes = [ctypes.c_void_p]
            rt.cudaStreamDestroy.restype = ctypes.c_int
            rt.cudaStreamSynchronize.argtypes = [ctypes.c_void_p]
            rt.cudaStreamSynchronize.restype = ctypes.c_int
            return rt
        except OSError:
            continue
    return None


class TrtRunner:
    """
    Minimal synchronous TensorRT engine runner (ctypes CUDA memory
    management, mirroring hook_detection_node's TRTEngine).
    """

    def __init__(self, engine_path: str):
        self._cudart = _load_cudart()
        if self._cudart is None:
            raise RuntimeError(
                'libcudart.so not found — set LD_LIBRARY_PATH to include '
                '/usr/local/cuda/lib64 and retry')
        import tensorrt as trt
        self._trt = trt
        self._stream = ctypes.c_void_p()
        if self._cudart.cudaStreamCreate(ctypes.byref(self._stream)) != 0:
            raise RuntimeError('cudaStreamCreate failed')

        logger = trt.Logger(trt.Logger.WARNING)
        with open(engine_path, 'rb') as f:
            self._engine = trt.Runtime(logger).deserialize_cuda_engine(f.read())
        if self._engine is None:
            raise RuntimeError(f'Failed to deserialise engine: {engine_path}')
        self._context = self._engine.create_execution_context()

        self._inputs, self._outputs = [], []
        for i in range(self._engine.num_io_tensors):
            name = self._engine.get_tensor_name(i)
            shape = tuple(self._engine.get_tensor_shape(name))
            dtype = trt.nptype(self._engine.get_tensor_dtype(name))
            mode = self._engine.get_tensor_mode(name)

            if mode == trt.TensorIOMode.INPUT:
                self._context.set_input_shape(name, (1, 3, INPUT_H, INPUT_W))
                shape = (1, 3, INPUT_H, INPUT_W)

            host = np.zeros(shape, dtype=dtype)
            device = ctypes.c_void_p()
            if self._cudart.cudaMalloc(
                    ctypes.byref(device), host.nbytes) != 0:
                raise RuntimeError(f'cudaMalloc failed for {name}')
            mem = {'name': name, 'host': host, 'device': device}
            (self._inputs if mode == trt.TensorIOMode.INPUT
             else self._outputs).append(mem)
            self._context.set_tensor_address(name, device.value)

    def describe(self):
        """Print engine binding summary."""
        print('  TensorRT engine bindings:')
        for m in self._inputs + self._outputs:
            print(f"    '{m['name']}'  shape={m['host'].shape}  "
                  f"dtype={m['host'].dtype}  "
                  f"{'INPUT' if m in self._inputs else 'OUTPUT'}")

    def run(self, tensor: np.ndarray):
        """Run one synchronous inference.  Returns list of output arrays."""
        inp = self._inputs[0]
        np.copyto(inp['host'], np.ascontiguousarray(tensor, inp['host'].dtype))
        H2D, D2H = 1, 2
        self._cudart.cudaMemcpyAsync(
            inp['device'], inp['host'].ctypes.data, inp['host'].nbytes,
            H2D, self._stream)
        self._context.execute_async_v3(stream_handle=self._stream.value)
        outputs = []
        for out in self._outputs:
            self._cudart.cudaMemcpyAsync(
                out['host'].ctypes.data, out['device'], out['host'].nbytes,
                D2H, self._stream)
            outputs.append(out['host'].copy())
        self._cudart.cudaStreamSynchronize(self._stream)
        return outputs


class OnnxRunner:
    """Minimal ONNX Runtime (CPU) runner for cross-checking the ONNX."""

    def __init__(self, onnx_path: str):
        import onnxruntime as ort
        so = ort.SessionOptions()
        so.log_severity_level = 3
        self._sess = ort.InferenceSession(
            onnx_path, sess_options=so, providers=['CPUExecutionProvider'])
        self._input_name = self._sess.get_inputs()[0].name

    def describe(self):
        print('  ONNX bindings:')
        for i in self._sess.get_inputs():
            print(f"    '{i.name}'  shape={i.shape}  dtype={i.type}")
        for o in self._sess.get_outputs():
            print(f"    '{o.name}'  shape={o.shape}  dtype={o.type}  OUTPUT")

    def run(self, tensor: np.ndarray):
        return list(self._sess.run(None, {self._input_name: tensor}))


# ═══════════════════════════════════════════════════════════════════════════════
#  Pre / post processing
# ═══════════════════════════════════════════════════════════════════════════════
def preprocess(frame, mode, channels, letterbox):
    """
    Prepare a BGR uint8 frame for inference.

    Returns (tensor, scale, offset) where scale/offset map 640-space
    boxes back to original image coordinates:
      direct resize:   x_orig = x_640 * (W/640)   → scale=None
      letterbox:       x_orig = (x_640 - offset) / scale
    """
    h, w = frame.shape[:2]
    if letterbox:
        scale = min(INPUT_W / w, INPUT_H / h)
        nw, nh = int(round(w * scale)), int(round(h * scale))
        resized = cv2.resize(frame, (nw, nh), interpolation=cv2.INTER_LINEAR)
        img = np.full((INPUT_H, INPUT_W, 3), PAD_VALUE, np.uint8)
        img[:nh, :nw] = resized      # bottom-right padding (SG convention)
        ox, oy = 0.0, 0.0
    else:
        img = cv2.resize(frame, (INPUT_W, INPUT_H),
                         interpolation=cv2.INTER_LINEAR)
        scale, ox, oy = None, 0.0, 0.0

    if channels == 'rgb':
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    x = img.astype(np.float32)
    if mode == 'a':
        x /= 255.0
    elif mode == 'b':
        x = (x / 255.0 - IMAGENET_MEAN) / IMAGENET_STD
    # mode 'c': passthrough raw 0-255

    tensor = np.ascontiguousarray(np.transpose(x, (2, 0, 1))[np.newaxis])
    return tensor, scale, ox, oy


def _sigmoid(x: np.ndarray) -> np.ndarray:
    return np.where(
        x >= 0,
        1.0 / (1.0 + np.exp(-x)),
        np.exp(x) / (1.0 + np.exp(x)),
    )


def analyse(boxes_out, scores_out, conf_thr, iou_thr):
    """
    Decode raw YOLO-NAS outputs.

    Returns (top_rows, detections, max_score):
      top_rows   — top-10 [(candidate_idx, score, class_id, [x1,y1,x2,y2])]
                   in 640-space
      detections — post-NMS [(box640, score, class_id)] in 640-space
    """
    boxes = np.asarray(boxes_out[0], dtype=np.float32)    # [8400, 4]
    scores = np.asarray(scores_out[0], dtype=np.float32)  # [8400, C]

    # Activation guard (same as hook_detection_node): the exported ONNX
    # already applies sigmoid, so scores arrive in [0, 1]; only apply
    # sigmoid defensively if the outputs look like raw logits.
    if scores.min() < 0.0 or scores.max() > 1.0:
        scores = _sigmoid(scores)

    best = scores.max(axis=1)
    class_ids = scores.argmax(axis=1)

    order = np.argsort(best)[::-1]
    top_rows = [(int(i), float(best[i]), int(class_ids[i]),
                 boxes[i].tolist()) for i in order[:10]]

    detections = []
    mask = best >= conf_thr
    if np.any(mask):
        fb, fs, fc = boxes[mask], best[mask], class_ids[mask]
        xywh = np.stack(
            [fb[:, 0], fb[:, 1], fb[:, 2] - fb[:, 0], fb[:, 3] - fb[:, 1]],
            axis=1)
        idx = cv2.dnn.NMSBoxes(
            xywh.tolist(), fs.tolist(), conf_thr, iou_thr)
        if len(idx):
            idx = np.asarray(idx).flatten()
            detections = [(fb[i], float(fs[i]), int(fc[i])) for i in idx]
            detections.sort(key=lambda d: -d[1])

    return top_rows, detections, float(best.max())


def box_to_orig(box640, frame, scale, ox, oy):
    """Map a 640-space box to original image coordinates."""
    x1, y1, x2, y2 = box640
    h, w = frame.shape[:2]
    if scale is None:
        sx, sy = w / INPUT_W, h / INPUT_H
        return (x1 * sx, y1 * sy, x2 * sx, y2 * sy)
    return ((x1 - ox) / scale, (y1 - oy) / scale,
            (x2 - ox) / scale, (y2 - oy) / scale)


def annotate(frame, detections, classes, scale, ox, oy, title):
    """Draw NMS detections + title strip on a copy of the frame."""
    vis = frame.copy()
    font = cv2.FONT_HERSHEY_SIMPLEX

    for box640, score, cls_id in detections:
        x1, y1, x2, y2 = box_to_orig(box640, frame, scale, ox, oy)
        ix1, iy1, ix2, iy2 = int(x1), int(y1), int(x2), int(y2)
        colour = CLASS_COLOURS[cls_id % len(CLASS_COLOURS)]
        name = classes[cls_id] if cls_id < len(classes) else f'cls_{cls_id}'
        cv2.rectangle(vis, (ix1, iy1), (ix2, iy2), colour, 2)
        label = f'{name} {score * 100:.0f}%'
        (tw, th), baseline = cv2.getTextSize(label, font, 0.5, 1)
        label_y = iy1 - 6 if iy1 > 20 else iy2 + th + 8
        cv2.rectangle(vis, (ix1, label_y - th - 4),
                      (ix1 + tw + 4, label_y + baseline), (0, 0, 0), -1)
        cv2.putText(vis, label, (ix1 + 2, label_y),
                    font, 0.5, colour, 1, cv2.LINE_AA)

    # Title strip (top-left)
    cv2.rectangle(vis, (0, 0), (vis.shape[1], 30), (0, 0, 0), -1)
    cv2.putText(vis, title, (10, 22), font, 0.6, (0, 255, 0), 2, cv2.LINE_AA)
    return vis


# ═══════════════════════════════════════════════════════════════════════════════
#  Main
# ═══════════════════════════════════════════════════════════════════════════════
def grab_webcam_frame(index: int) -> np.ndarray:
    cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError(f'Cannot open /dev/video{index}')
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    ret, frame = cap.read()
    cap.release()
    if not ret:
        raise RuntimeError(f'Failed to read frame from /dev/video{index}')
    return frame


def main():
    ap = argparse.ArgumentParser(
        description='Standalone YOLO-NAS TensorRT/ONNX inference diagnostic '
                    '(no ROS). Compares preprocessing normalisation modes.')
    ap.add_argument('--engine', default=str(DEFAULT_ENGINE),
                    help='Path to TensorRT .engine file')
    ap.add_argument('--onnx', default=None,
                    help='Path to ONNX model (overrides --engine)')
    ap.add_argument('--image', default=str(DEFAULT_IMAGE),
                    help='Sample image path')
    ap.add_argument('--webcam', type=int, default=None, metavar='N',
                    help='Use live webcam /dev/videoN instead of --image')
    ap.add_argument('--mode', choices=['a', 'b', 'c', 'all'], default='all',
                    help='Normalisation mode to test (default: all)')
    ap.add_argument('--channels', choices=['rgb', 'bgr', 'both'],
                    default='both',
                    help='Channel order to test (default: both)')
    ap.add_argument('--letterbox', action='store_true',
                    help='Additionally test letterbox resize (pad 114, the '
                         'training-time resize) for every selected mode')
    ap.add_argument('--conf', type=float, default=0.25,
                    help='Confidence threshold for NMS/annotation '
                         '(default: 0.25)')
    ap.add_argument('--iou', type=float, default=0.45,
                    help='IoU threshold for NMS (default: 0.45)')
    ap.add_argument('--top', type=int, default=10,
                    help='Number of top raw scores to print (default: 10)')
    ap.add_argument('--out', default=str(DEFAULT_OUT),
                    help='Annotated output image path '
                         '(default: scripts/test_output.jpg)')
    ap.add_argument('--classes', default=','.join(DEFAULT_CLASSES),
                    help='Comma-separated class names')
    args = ap.parse_args()

    classes = [c.strip() for c in args.classes.split(',') if c.strip()]

    # ── Load input frame ────────────────────────────────────────────────
    if args.webcam is not None:
        frame = grab_webcam_frame(args.webcam)
        src_desc = f'/dev/video{args.webcam}'
    else:
        if not Path(args.image).is_file():
            raise SystemExit(f'Image not found: {args.image}')
        frame = cv2.imread(args.image)
        if frame is None:
            raise SystemExit(f'Failed to read image: {args.image}')
        src_desc = args.image

    # ── Load model ──────────────────────────────────────────────────────
    if args.onnx:
        runner = OnnxRunner(args.onnx)
        model_desc = f'ONNX: {args.onnx}'
    else:
        if not Path(args.engine).is_file():
            raise SystemExit(f'Engine not found: {args.engine}')
        runner = TrtRunner(args.engine)
        model_desc = f'TensorRT engine: {args.engine}'

    print('═══════════════════════════════════════════════════════════════')
    print(' RYUGU ROV — YOLO-NAS preprocessing diagnostic')
    print('═══════════════════════════════════════════════════════════════')
    print(f' Image: {src_desc} ({frame.shape[1]}x{frame.shape[0]})')
    print(f' Model: {model_desc}')
    print(f' Classes: {classes}')
    runner.describe()
    print()

    # ── Build configuration grid ────────────────────────────────────────
    modes = ['a', 'b', 'c'] if args.mode == 'all' else [args.mode]
    channels = ['rgb', 'bgr'] if args.channels == 'both' else [args.channels]
    configs = [(m, ch, False) for m in modes for ch in channels]
    if args.letterbox:
        configs += [(m, ch, True) for m in modes for ch in channels]

    results = []  # (config, top_rows, detections, max_score, vis_img)
    out_path = Path(args.out)

    for mode, ch, letterbox in configs:
        tensor, scale, ox, oy = preprocess(frame, mode, ch, letterbox)

        t0 = time.perf_counter()
        outputs = runner.run(tensor)
        latency_ms = (time.perf_counter() - t0) * 1000.0

        if len(outputs) < 2:
            print(f'[!] Expected 2 outputs, got {len(outputs)} — skipping')
            continue
        boxes_out, scores_out = outputs[0], outputs[1]
        # Defensive: swap if order differs
        if boxes_out.shape[-1] != 4:
            boxes_out, scores_out = scores_out, boxes_out

        top_rows, detections, max_score = analyse(
            boxes_out, scores_out, args.conf, args.iou)

        cfg_label = (f'Mode {mode.upper()} ({MODE_LABELS[mode]}) — '
                     f'{ch.upper()}' + (' — letterbox' if letterbox else ''))
        print('───────────────────────────────────────────────────────────────')
        print(f' {cfg_label}')
        print(f' Mode {mode.upper()} Max Score: {max_score:.4f}   '
              f'({latency_ms:.1f} ms)')
        print(f' Top {args.top} raw candidates:')
        print(f'   {"#":>6s}  {"class":<16s} {"score":>8s}  '
              f'{"x1":>8s} {"y1":>8s} {"x2":>8s} {"y2":>8s}')
        for idx, score, cls_id, box640 in top_rows[:args.top]:
            x1, y1, x2, y2 = box_to_orig(box640, frame, scale, ox, oy)
            name = classes[cls_id] if cls_id < len(classes) else f'cls_{cls_id}'
            print(f'   {idx:6d}  {name:<16s} {score:8.4f}  '
                  f'{x1:8.1f} {y1:8.1f} {x2:8.1f} {y2:8.1f}')

        print(f' Detected Objects (NMS @ conf {args.conf:.2f}):')
        if detections:
            for box640, score, cls_id in detections:
                x1, y1, x2, y2 = box_to_orig(box640, frame, scale, ox, oy)
                name = classes[cls_id] if cls_id < len(classes) else f'cls_{cls_id}'
                print(f'   [{name}, {score:.4f}, '
                      f'{x1:.1f}, {y1:.1f}, {x2:.1f}, {y2:.1f}]')
        else:
            print('   (none above threshold)')

        vis = annotate(frame, detections, classes, scale, ox, oy, cfg_label)
        results.append(
            ((mode, ch, letterbox), cfg_label, max_score, vis))

        suffix = f'_mode{mode}_{ch}' + ('_letterbox' if letterbox else '')
        per_cfg_path = out_path.with_name(out_path.stem + suffix + '.jpg')
        cv2.imwrite(str(per_cfg_path), vis)
        print(f' Saved annotated image: {per_cfg_path}')
        print()

    # ── Summary ─────────────────────────────────────────────────────────
    results.sort(key=lambda r: -r[2])
    print('═══════════════════════════════════════════════════════════════')
    print(' SUMMARY (best first)')
    for _, label, max_score, _ in results:
        print(f'   max raw = {max_score:8.4f}   {label}')
    best_label = results[0][1]
    best_vis = results[0][3]
    cv2.imwrite(str(out_path), best_vis)
    print(f' BEST: {best_label}')
    print(f' Annotated best-result image saved to: {out_path}')
    print('═══════════════════════════════════════════════════════════════')


if __name__ == '__main__':
    main()
