#!/usr/bin/env python3
"""
qr_diag_front.py — Real-time QR detection diagnostic for JETE front camera.

Grabs frames from /dev/video0 and tries each pipeline pass independently,
reporting which pass decodes the QR, Laplacian variance, and timing.

Run:
    python3 tests/comm/qr_diag_front.py
Press Ctrl+C to stop.
"""

import time
import cv2
import numpy as np
from pyzbar.pyzbar import decode as pyzbar_decode


DEVICE = '/dev/video0'
WIDTH  = 1280
HEIGHT = 720


def laplacian_var(gray):
    small = gray[::2, ::2]
    return float(cv2.Laplacian(small, cv2.CV_64F).var())


def try_pyzbar(img, label, back=1.0):
    t0 = time.monotonic()
    decoded = pyzbar_decode(img)
    dt = (time.monotonic() - t0) * 1000
    if decoded:
        data = decoded[0].data.decode('utf-8', errors='replace')
        return True, data, dt, label
    return False, None, dt, label


def main():
    cap = cv2.VideoCapture(DEVICE, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not cap.isOpened():
        print(f'[ERROR] Cannot open {DEVICE}')
        return

    print(f'[INFO] Opened {DEVICE} — grabbing frames. Press Ctrl+C to stop.\n')

    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    sharpen_kernel = np.array([[-1,-1,-1],[-1,9,-1],[-1,-1,-1]], dtype=np.float32)

    frame_num = 0
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print('[WARN] Read failed, retrying...')
                time.sleep(0.1)
                continue

            frame = cv2.rotate(frame, cv2.ROTATE_180)  # JETE mounted upside-down
            gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            lap   = laplacian_var(gray)
            frame_num += 1

            gate_old = lap >= 4.0  # old threshold
            gate_new = lap >= 1.0  # new threshold
            gate_str = (
                f'Laplacian={lap:.2f}  '
                f'gate_old(4.0)={"PASS" if gate_old else "FAIL ← WAS BLOCKING HERE"}  '
                f'gate_new(1.0)={"PASS" if gate_new else "FAIL"}'
            )

            if not gate_new:
                print(f'[#{frame_num:04d}] SMOOTH FRAME SKIPPED — {gate_str}')
                time.sleep(0.1)
                continue

            results = []

            # Pass 0 — raw pyzbar (no preprocessing)
            ok, data, dt, lbl = try_pyzbar(gray, 'Pass0_raw')
            results.append((ok, data, dt, lbl))

            if not ok:
                # Pass 1a — CLAHE only, native scale
                frame_qr = clahe.apply(gray)
                ok, data, dt, lbl = try_pyzbar(frame_qr, 'Pass1_clahe_1x')
                results.append((ok, data, dt, lbl))

            if not ok:
                # Pass 1b — CLAHE + 2× upscale
                up2_c = cv2.resize(frame_qr, None, fx=2.0, fy=2.0, interpolation=cv2.INTER_CUBIC)
                ok, data, dt, lbl = try_pyzbar(up2_c, 'Pass1_clahe_2x', back=0.5)
                results.append((ok, data, dt, lbl))

            if not ok:
                # Pass 1c — CLAHE + 0.5× downscale
                down05_c = cv2.resize(frame_qr, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
                ok, data, dt, lbl = try_pyzbar(down05_c, 'Pass1_clahe_0.5x', back=2.0)
                results.append((ok, data, dt, lbl))

            if not ok:
                # Pass 2 — CLAHE + sharpen, native
                sharp = cv2.filter2D(frame_qr, -1, sharpen_kernel)
                ok, data, dt, lbl = try_pyzbar(sharp, 'Pass2_clahe_sharp_1x')
                results.append((ok, data, dt, lbl))

            if not ok:
                # Pass 2b — CLAHE + sharpen + 2×
                up2 = cv2.resize(sharp, None, fx=2.0, fy=2.0, interpolation=cv2.INTER_CUBIC)
                ok, data, dt, lbl = try_pyzbar(up2, 'Pass2_clahe_sharp_2x', back=0.5)
                results.append((ok, data, dt, lbl))

            if not ok:
                # OpenCV QRCodeDetector fallback
                qr_det = cv2.QRCodeDetector()
                t0 = time.monotonic()
                data_cv, pts, _ = qr_det.detectAndDecode(gray)
                dt_cv = (time.monotonic() - t0) * 1000
                if data_cv:
                    results.append((True, data_cv, dt_cv, 'Pass3_cv_detect'))
                    ok = True
                else:
                    results.append((False, None, dt_cv, 'Pass3_cv_detect'))

            # ── Print summary line ──────────────────────────────────────
            total_ms = sum(r[2] for r in results)
            winner   = next((r for r in results if r[0]), None)

            if winner:
                print(f'[#{frame_num:04d}] ✅ DECODED via {winner[3]}  '
                      f'data="{winner[1]}"  total={total_ms:.1f}ms  {gate_str}')
            else:
                passes_tried = ', '.join(f'{r[3]}({r[2]:.1f}ms)' for r in results)
                print(f'[#{frame_num:04d}] ❌ NOT DECODED  tried=[{passes_tried}]  '
                      f'total={total_ms:.1f}ms  {gate_str}')

            time.sleep(0.2)   # 5 Hz

    except KeyboardInterrupt:
        print('\n[INFO] Stopped.')
    finally:
        cap.release()


if __name__ == '__main__':
    main()
