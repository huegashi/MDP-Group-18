# yolo11_live_client.py
# Mission: Agent 2079 – Vision inference console (laptop)
# Pulls MJPEG frames from the Raspberry Pi stream, runs YOLOv11, and displays annotated video.

import cv2
import time
from ultralytics import YOLO

# === CONFIG ===
PI_STREAM_URL = "http://192.168.18.1" \
":8000/stream.mjpg"  # ← change to your Pi's IP
MODEL_PATH = "Week8_best.pt"                   # ← your custom YOLOv11 weights
CONF_THRESHOLD = 0.25
IOU_THRESHOLD = 0.45
TARGET_SIZE = (640, 480)  # (width, height) – should match stream for best speed

# === INIT ===
print("[Agent 2079] Loading YOLOv11 model...")
model = YOLO(MODEL_PATH)

# Optional: set model parameters (depends on ultralytics version)
model.overrides["conf"] = CONF_THRESHOLD
model.overrides["iou"] = IOU_THRESHOLD

print(f"[Agent 2079] Connecting to stream: {PI_STREAM_URL}")
cap = cv2.VideoCapture(PI_STREAM_URL)
if not cap.isOpened():
    raise RuntimeError("Could not open stream. Check URL, firewall, and that the Pi server is running.")

# FPS tracking
fps_t0 = time.time()
frame_count = 0

# Window title + slight agent flair
WINDOW_NAME = "Agent 2079 — Vision Console (YOLOv11)"
cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

try:
    while True:
        ok, frame = cap.read()
        if not ok:
            # Try to recover without crashing
            time.sleep(0.01)
            continue

        # Optional: resize to match training size or to improve speed
        if TARGET_SIZE:
            frame = cv2.resize(frame, TARGET_SIZE)

        # Inference (BGR ndarray is fine)
        # Returns a list of Results; we use .plot() for a quick overlay
        results = model.predict(frame, verbose=False)[0]
        annotated = results.plot()  # draws boxes, labels, confidences

        # FPS calc
        frame_count += 1
        if frame_count % 10 == 0:
            dt = time.time() - fps_t0
            fps = frame_count / max(dt, 1e-6)
            cv2.setWindowTitle(WINDOW_NAME, f"{WINDOW_NAME} — {fps:.1f} FPS")

        cv2.imshow(WINDOW_NAME, annotated)

        # Controls
        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord('q'):  # ESC or q
            print("[Agent 2079] Vision console: extraction complete. Shutting down.")
            break
        elif key == ord('p'):
            # Pause/resume
            print("[Agent 2079] Hold… (press 'p' to resume)")
            while True:
                k2 = cv2.waitKey(50) & 0xFF
                if k2 == ord('p'):
                    print("[Agent 2079] Resume.")
                    break

finally:
    cap.release()
    cv2.destroyAllWindows()

#pscp C:\Users\gauth\OneDrive - Nanyang Technological University\Documents\SC2079\rpi_stream.py pi@192.168.1.42:/home/pi/
