# Modern Ultralytics rewrite of your torch.hub helper
# pip install ultralytics
from ultralytics import YOLO
from pathlib import Path
import torch


def load_yolo(model_ref="yolov11n.pt", device=None, verbose=True):
    """
    Load a YOLO model using the modern Ultralytics API.

    Args:
        model_ref (str | Path): Model name (e.g., 'yolov11n.pt', 'yolov8n.pt') or local path to .pt
        device (str | torch.device | None): 'cpu', 'cuda', 'mps', or None to auto-pick
        verbose (bool): Print model info

    Returns:
        YOLO: Loaded model on the chosen device
    """
    # Auto-pick device if not provided
    if device is None:
        if torch.cuda.is_available():
            device = "cuda"
        elif getattr(torch.backends, "mps", None) and torch.backends.mps.is_available():  # Apple Silicon
            device = "mps"
        else:
            device = "cpu"

    model = YOLO(str(model_ref))
    # Move to device (Ultralytics uses .to() on the underlying model)
    try:
        model.model.to(device)
    except Exception:
        # some exported formats (onnx, openvino, etc.) won't have .model
        pass

    if verbose:
        print(f"Loaded {model_ref} on device: {device}")
    return model


def demo_infer(model: YOLO):
    """
    Run a quick batched inference demo on various input types, then print & save results.
    """
    import cv2
    import numpy as np
    from PIL import Image

    # Prepare sample inputs (replace with your own)
    imgs = [
        "WIN_20250829_00_22_45_Pro.jpg",           # URL
        "https://ultralytics.com/images/bus.jpg",                           # local file (installed with package)
        Image.open("WIN_20250829_00_22_45_Pro.jpg"),               # PIL
        cv2.imread("WIN_20250829_00_22_45_Pro.jpg")[:, :, ::-1],   # OpenCV (BGR->RGB)
        np.zeros((320, 640, 3), dtype=np.uint8),                # numpy
    ]

    # Inference (imgsz replaces old 'size')
    results = model(imgs, imgsz=320, save=True)

    # Print and save results (saves to runs/detect/...)
    for r in results:
        print(r)          # prints summary info
        print(r.boxes)    # shows bounding boxes tensor
        print(r.probs)

# annotated images to runs/detect/exp*/


if __name__ == "__main__":
    # Examples:
    # 1) Pretrained small model
    model = load_yolo("Week5_yolo.pt")
    # 2) Or use your custom local weights:
    # model = load_yolo("path/to/new_yolo11.pt")

