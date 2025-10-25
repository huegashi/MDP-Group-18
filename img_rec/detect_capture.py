#!/usr/bin/env python3
import subprocess
import sys
import cv2
import numpy as np
from ultralytics import YOLO

# --- config ---
WIDTH, HEIGHT, FPS = 1640, 1232, 30
MODEL_PATH = "Week6_last.pt"      # your YOLOv11 weights
SHOW = True                       # set False if running headless/SSH (no GUI)
# ---------------

model = YOLO(MODEL_PATH)

# Start libcamera-vid as MJPEG to stdout
# Note: --codec mjpeg makes each frame a complete JPEG image.
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

proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=0)

# JPEG markers
SOI = b"\xff\xd8"  # start of image
EOI = b"\xff\xd9"  # end of image

buffer = b""
print("🚀 Streaming from libcamera-vid… press Ctrl+C to stop.")
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

                # YOLO inference
                results = model(frame)
                annotated = results[0].plot()

                if SHOW:
                    cv2.imshow("YOLOv11 Live (libcamera-vid MJPEG)", annotated)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        raise KeyboardInterrupt
                else:
                    # headless path: do something else (save frames, etc.)
                    pass
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



