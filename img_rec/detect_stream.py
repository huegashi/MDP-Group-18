#!/usr/bin/env python3
import subprocess
import sys
import time
import cv2
import numpy as np
from ultralytics import YOLO

# --- config ---
WIDTH, HEIGHT, FPS = 840, 680, 30
MODEL_PATH = "Week6_last.pt"      # your YOLOv11 weights
SHOW = True                       # set False if running headless/SSH (no GUI)
INFER_EVERY = 3                   # run YOLO every Nth frame (tune this)
FONT = cv2.FONT_HERSHEY_SIMPLEX
# ---------------

model = YOLO(MODEL_PATH)

# Start libcamera-vid as MJPEG to stdout (each frame is a complete JPEG)
cmd = [
    "libcamera-vid",
    "-t", "0",                    # no timeout
    "--width", str(WIDTH),
    "--height", str(HEIGHT),
    "--framerate", str(FPS),
    "--codec", "mjpeg",
    "--inline",
    "--nopreview",
    "-o", "-"                     # write to stdout
]

proc = subprocess.Popen(
    cmd,
    stdout=subprocess.PIPE,
    stderr=subprocess.DEVNULL,
    bufsize=0
)

# JPEG markers
SOI = b"\xff\xd8"  # start of image
EOI = b"\xff\xd9"  # end of image

buffer = b""

# FPS accounting
frame_idx = 0
disp_frames = 0
infer_frames = 0
last_disp_t = time.time()
last_infer_t = time.time()
disp_fps = 0.0
infer_fps = 0.0

print("🚀 Streaming from libcamera-vid… press 'q' to stop.")
try:
    while True:
        chunk = proc.stdout.read(4096)
        if not chunk:
            print("Stream ended.")
            break
        buffer += chunk

        # Find full JPEG(s) in the buffer
        while True:
            start = buffer.find(SOI)
            end   = buffer.find(EOI)
            if start != -1 and end != -1 and end > start:
                jpg = buffer[start:end+2]
                buffer = buffer[end+2:]

                # Decode JPEG -> BGR image
                frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                if frame is None:
                    continue

                frame_idx += 1
                disp_frames += 1

                # --- Display FPS update (once per ~second) ---
                now = time.time()
                if now - last_disp_t >= 1.0:
                    disp_fps = disp_frames / (now - last_disp_t)
                    disp_frames = 0
                    last_disp_t = now

                # Run YOLO every Nth frame; show raw frame otherwise
                if frame_idx % INFER_EVERY == 0:
                    t0 = time.time()
                    results = model(frame)
                    annotated = results[0].plot()
                    t1 = time.time()
                    infer_frames += 1
                    # Smooth/EMA-like update of infer_fps (or compute per-second if you prefer)
                    dt = max(t1 - last_infer_t, 1e-6)
                    # Update every second or on first run
                    if (t1 - last_infer_t) >= 1.0 or infer_fps == 0.0:
                        infer_fps = infer_frames / (t1 - last_infer_t)
                        infer_frames = 0
                        last_infer_t = t1
                else:
                    annotated = frame

                # --- Overlay FPS text ---
                text = f"Cam FPS: {disp_fps:4.1f} | Infer FPS: {infer_fps:4.1f} | Frame: {frame_idx}"
                cv2.putText(annotated, text, (10, 30), FONT, 0.8, (0, 255, 0), 2, cv2.LINE_AA)

                if SHOW:
                    cv2.imshow("YOLOv11 Live (libcamera-vid MJPEG)", annotated)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        raise KeyboardInterrupt
                # If running headless, you could save/stream annotated frames here
            else:
                # no full JPEG yet; read more bytes
                break

except KeyboardInterrupt:
    print("\nStopping…")

finally:
    try:
        proc.terminate()
    except Exception:
        pass
    cv2.destroyAllWindows()
