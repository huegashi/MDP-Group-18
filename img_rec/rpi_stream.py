# pi_stream_server.py
# Mission: Agent 2079 – Vision uplink node
# Streams the Raspberry Pi camera as MJPEG over HTTP.

from flask import Flask, Response
import cv2
import time

# Try picamera2 first (preferred on Bullseye/Bookworm)
try:
    from picamera2 import Picamera2
    USE_PICAM2 = True
except ImportError:
    USE_PICAM2 = False

app = Flask(__name__)

if USE_PICAM2:
    picam2 = Picamera2()
    # Configure for 640x480 @ ~30fps; adjust as needed for bandwidth/latency
    config = picam2.create_preview_configuration(main={"size": (640, 480), "format": "RGB888"})
    picam2.configure(config)
    picam2.start()
else:
    # Fallback: USB camera via OpenCV (if not using PiCam2)
    # Change index if you have multiple cameras
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

def mjpeg_generator():
    # Agent motif—minimal, non-spammy:
    # print("[Agent 2079] Vision uplink: transmitting frames...")
    while True:
        if USE_PICAM2:
            frame = picam2.capture_array()  # RGB
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        else:
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.01)
                continue

        # JPEG encode
        ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if not ok:
            continue

        # multipart/x-mixed-replace stream
        yield (b"--frame\r\n"
               b"Content-Type: image/jpeg\r\n\r\n" +
               buf.tobytes() +
               b"\r\n")

@app.route("/stream.mjpg")
def stream():
    return Response(mjpeg_generator(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/")
def index():
    return "Agent 2079 Vision Stream: /stream.mjpg"

if __name__ == "__main__":
    # Listen on all interfaces so your laptop can connect; change port if needed
    app.run(host="0.0.0.0", port=8000, threaded=True)
