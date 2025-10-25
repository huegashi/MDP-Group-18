import time
from datetime import datetime
from picamera import PiCamera
from picamera.array import PiRGBArray
import cv2
import os

def run_camera(folder_name):
    camera = PiCamera()
    camera.resolution = (640, 480)
    rawCapture = PiRGBArray(camera, size=(640, 480))
    time.sleep(0.1)  # allow camera to warm up

    id = 0
    print("Camera running. Press 'y' to save photo, 'q' to quit, 'n' to skip.")

    try:
        for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
            image = frame.array
            cv2.imshow("Frame", image)

            key = cv2.waitKey(1) & 0xFF
            rawCapture.truncate(0)

            if key == ord("q"):  # quit program
                break
            elif key == ord("y"):  # take and save photo
                filename = f"{folder_name}/{id}.jpg"
                cv2.imwrite(filename, image)
                print(f"✅ Saved {filename}")
                id += 1
            elif key == ord("n"):  # do nothing, just skip
                print("❌ Skipped")

    finally:
        camera.close()
        cv2.destroyAllWindows()
        print("Camera stopped.")

if __name__ == "__main__":
    folder_name = datetime.now().strftime("%d_%m_%Y__%H_%M_%S")
    os.makedirs(folder_name, exist_ok=True)
    run_camera(folder_name)
