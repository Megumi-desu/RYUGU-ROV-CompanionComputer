#!/usr/bin/env python3
"""
Re-export the RYUGU ROV YOLO-NAS ONNX from the GOOD training checkpoint.

────────────────────────────────────────────────────────────────────────────
WHY THIS EXISTS  (incident of 2026-08-28/29)
────────────────────────────────────────────────────────────────────────────
The ONNX currently deployed was exported from training run
RUN_20260827_160024_720472, whose validation mAP@0.50 was ~0.0015 (a dead
run).  The retry run RUN_20260827_164215_006959 converged to mAP@0.50 =
0.971, but its checkpoint was never exported — the original notebook's
`ckpt_candidates[-1]` glob-pick silently chose the wrong run.  The broken
ONNX scores ~0.08 on its own training test images, so the detector never
fires and no overlays ever appear.

ALWAYS hardcode the good checkpoint path.  Do NOT re-introduce glob[-1].

────────────────────────────────────────────────────────────────────────────
FULL RECOVERY PROCEDURE
────────────────────────────────────────────────────────────────────────────
1) Install torch + super-gradients (CPU is enough for export):

       pip install --user torch
       pip install --user super-gradients --no-deps
       pip install --user hydra-core omegaconf torchmetrics torchinfo \
           pycocotools onnx onnxruntime

2) Re-export + verify (this script — fails if the ONNX does not fire):

       python3 scripts/reexport_yolo_nas.py \
           --ckpt /home/icad/Downloads/YOLO-NAS_ROVhook-v1/checkpoints/yolo_nas_rov_hook/RUN_20260827_164215_006959/ckpt_best.pth \
           --data /home/icad/Downloads/YOLO-NAS_ROVhook-v1/dataset \
           --out  /home/icad/Downloads/YOLO-NAS_ROVhook-v1/yolo_nas_rov_hook_v2.onnx

   PASS criterion: worst max score over the dataset test images > 0.70
   (the broken ONNX scores ~0.08).  Do not deploy an ONNX that fails this.

3) Rebuild the TensorRT FP16 engine (TensorRT 10.x flag syntax):

       /usr/src/tensorrt/bin/trtexec \
           --onnx=/home/icad/Downloads/YOLO-NAS_ROVhook-v1/yolo_nas_rov_hook_v2.onnx \
           --saveEngine=/home/icad/RYUGU-ROV-CompanionComputer/src/ryugu_control/models/yolo_nas_rov_hook_fp16.engine \
           --fp16 --memPoolSize=workspace:4096

4) Rebuild workspace + restart the stack:

       cd /home/icad/RYUGU-ROV-CompanionComputer
       colcon build --symlink-install
       sudo systemctl restart ryugu-rov

5) Verify:
   - journalctl -u ryugu-rov -f  → the 📊 line every ~10 s should report
     "top raw score=0.8x" instead of ~0.08 when the hook is in view.
   - The GCS stream (:8554/:8555) shows green "AI DETECT: OK (NN FPS)"
     plus boxes/crosshair; yellow "AI: NO TARGET" otherwise.

6) Fix the training notebook on the laptop: hardcode the checkpoint path
   in the export cell (see above).  Never glob-pick again.
────────────────────────────────────────────────────────────────────────────
"""

import argparse
import glob
import os

import cv2
import numpy as np
import torch
from super_gradients.training import models


def export(ckpt_path: str, out_path: str):
    print(f"[+] Loading checkpoint: {ckpt_path}")
    model = models.get(
        'yolo_nas_s',
        num_classes=3,
        checkpoint_path=ckpt_path,
    )
    model.eval()

    print("[+] Exporting to ONNX via SuperGradients (legacy TorchScript exporter)...")
    models.convert_to_onnx(
        model=model,
        out_path=out_path,
        prep_model_for_conversion_kwargs={'input_size': (1, 3, 640, 640)},
        torch_onnx_export_kwargs={'dynamo': False},
    )
    print(f"[+] ONNX exported: {out_path} ({os.path.getsize(out_path)} bytes)")


def verify(onnx_path: str, data_dir: str, n: int = 8) -> bool:
    """Run the exported ONNX on dataset test images (CPU) and report."""
    import onnxruntime as ort
    so = ort.SessionOptions()
    so.log_severity_level = 3
    sess = ort.InferenceSession(
        onnx_path, sess_options=so, providers=['CPUExecutionProvider'])

    imgs = sorted(glob.glob(os.path.join(data_dir, 'test', 'images', '*')))[:n]
    worst = 1.0
    for p in imgs:
        img = cv2.imread(p)
        r = cv2.resize(img, (640, 640), interpolation=cv2.INTER_LINEAR)
        rgb = cv2.cvtColor(r, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        inp = np.ascontiguousarray(
            np.transpose(rgb, (2, 0, 1))[np.newaxis, ...])
        input_name = sess.get_inputs()[0].name
        _, scores = sess.run(None, {input_name: inp})
        mx = float(scores[0].max())
        worst = min(worst, mx)
        print(f"    {os.path.basename(p)[:40]:42s} max={mx:.4f}")
    ok = worst > 0.70
    print(f"[+] Worst max score over {len(imgs)} test images: {worst:.4f} "
          f"({'PASS' if ok else 'FAIL — DO NOT DEPLOY'})")
    return ok


def main():
    ap = argparse.ArgumentParser(
        description='Re-export YOLO-NAS ONNX from a checkpoint and verify '
                    'it fires on training-domain test images.')
    ap.add_argument('--ckpt', required=True)
    ap.add_argument('--data', required=True)
    ap.add_argument('--out', required=True)
    args = ap.parse_args()

    export(args.ckpt, args.out)
    ok = verify(args.out, args.data)
    raise SystemExit(0 if ok else 1)


if __name__ == '__main__':
    main()
