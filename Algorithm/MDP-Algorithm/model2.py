import os
import shutil
import time
import glob
import torch
from PIL import Image
import cv2
import random
import string
import numpy as np
import random
from ultralytics import YOLO

def get_random_string(length):
    """
    Generate a random string of fixed length 

    Inputs
    ------
    length: int - length of the string to be generated

    Returns
    -------
    str - random string

    """
    result_str = ''.join(random.choice(string.ascii_letters) for i in range(length))
    return result_str

def load_model():
    """
    Load the model from the local directory
    """
    #model = torch.hub.load('./', 'custom', path='YOLOv5_new.pt', source='local')
    model = YOLO("/Users/kiki/Desktop/MDP-FINAL-main 2/MDP-Algorithm/Week7_best.pt")
    return model

def draw_own_bbox(img,x1,y1,x2,y2,label,color=(36,255,12),text_color=(0,0,0)):
    """
    Draw bounding box on the image with text label and save both the raw and annotated image in the 'own_results' folder

    Inputs
    ------
    img: numpy.ndarray - image on which the bounding box is to be drawn

    x1: int - x coordinate of the top left corner of the bounding box

    y1: int - y coordinate of the top left corner of the bounding box

    x2: int - x coordinate of the bottom right corner of the bounding box

    y2: int - y coordinate of the bottom right corner of the bounding box

    label: str - label to be written on the bounding box

    color: tuple - color of the bounding box

    text_color: tuple - color of the text label

    Returns
    -------
    None

    """

    # Reformat the label to {label name}-{label id}
    x1, x2, y1, y2 = map(int, (x1, x2, y1, y2))
    # Convert the coordinates to int
    # Create a random string to be used as the suffix for the image name, just in case the same name is accidentally used
    rand = str(int(time.time()))

    # Save the raw image
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    cv2.imwrite(f"own_results/raw_image_{label}_{rand}.jpg", img)

    # Draw the bounding box
    img = cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
    # For the text background, find space required by the text so that we can put a background with that amount of width.
    (w, h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
    # Print the text  
    img = cv2.rectangle(img, (x1, y1 - 20), (x1 + w, y1), color, -1)
    img = cv2.putText(img, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, text_color, 1)
    # Save the annotated image
    cv2.imwrite(f"own_results/annotated_image_{label}_{rand}.jpg", img)


def predict_image(image, model, signal):
    """
    Ultralytics YOLOv11/YOLOv8 version.
    - Loads uploads/<image>
    - Runs model.predict(...)
    - Applies your confidence/area/signal heuristics
    - Draws our custom bbox label "<name>-<cls_id>"
    - Returns cls_id as string (or 'NA')
    """

    try:
        img_path = os.path.join("uploads", image)
        pil_img = Image.open(img_path).convert("RGB")

        # Run Ultralytics inference (no pandas; results is a list)
        results = model.predict(
            pil_img,
            conf=0.25,
            save=True,            # set False for faster continuous use
            project="runs",
            name="detect",
            verbose=False
        )
        r = results[0]

        if r.boxes is None or len(r.boxes) == 0:
            return "NA"

        # Extract tensors -> numpy
        xyxy = r.boxes.xyxy.cpu().numpy()                # (N, 4)
        conf = r.boxes.conf.cpu().numpy()                # (N,)
        cls  = r.boxes.cls.cpu().numpy().astype(int)     # (N,)
        names = model.names                               # {id: name}

        # Build a list of dicts similar to your old pandas rows
        preds = []
        for i in range(len(cls)):
            x1, y1, x2, y2 = xyxy[i]
            area = (x2 - x1) * (y2 - y1)
            name_i = names.get(int(cls[i]), str(int(cls[i])))
            preds.append({
                "xmin": float(x1),
                "ymin": float(y1),
                "xmax": float(x2),
                "ymax": float(y2),
                "confidence": float(conf[i]),
                "class": int(cls[i]),
                "name": name_i,
                "bboxArea": float(area),
            })

        # Sort by area (largest first) and drop Bullseye (if present)
        preds.sort(key=lambda p: p["bboxArea"], reverse=True)
        preds = [p for p in preds if p["name"] != "Bullseye"]

        if not preds:
            return "NA"

        # --- Your shortlist + signal logic ---
        pred = "NA"
        if len(preds) == 1:
            pred = preds[0]
        else:
            shortlist = []
            current_area = preds[0]["bboxArea"]
            for p in preds:
                if p["confidence"] > 0.5 and (
                    p["bboxArea"] >= current_area * 0.8 or
                    (p["name"] == "One" and p["bboxArea"] >= current_area * 0.6)
                ):
                    shortlist.append(p)
                    current_area = p["bboxArea"]

            if len(shortlist) == 1:
                pred = shortlist[0]
            elif len(shortlist) > 1:
                # Sort by xmin for L/R choice
                shortlist.sort(key=lambda p: p["xmin"])
                if signal == "L":
                    pred = shortlist[0]
                elif signal == "R":
                    pred = shortlist[-1]
                else:
                    # Center choice
                    pred = next((p for p in shortlist if 250 < p["xmin"] < 774), None)
                    if pred is None:
                        shortlist.sort(key=lambda p: p["bboxArea"])
                        pred = shortlist[-1]

        if isinstance(pred, str):
            return "NA"

        # Draw custom bbox (make sure draw_own_bbox converts RGB->BGR)
        draw_own_bbox(np.array(pil_img), pred["xmin"], pred["ymin"], pred["xmax"], pred["ymax"],
                      f"{pred['name']}-{pred['class']}")

        # Return YOLO class id as string (no manual mapping)
        return str(int(pred["class"]))

    except Exception as e:
        print(f"Error during prediction (YOLOv11): {e}")
        return "NA"


def predict_image_week_9(image, model):
    import os
    import numpy as np
    from PIL import Image

    img = Image.open(os.path.join('uploads', image)).convert('RGB')

    results = model.predict(
        img,
        conf=0.25,
        save=True,           # set False for speed
        project='runs',
        name='detect',
        verbose=False
    )
    r = results[0]
    if r.boxes is None or len(r.boxes) == 0:
        return 'NA'

    xyxy = r.boxes.xyxy.cpu().numpy()
    conf = r.boxes.conf.cpu().numpy()
    cls  = r.boxes.cls.cpu().numpy().astype(int)
    names = model.names

    areas = (xyxy[:,2]-xyxy[:,0]) * (xyxy[:,3]-xyxy[:,1])
    order = np.argsort(-areas)

    chosen = None
    for i in order:
        if conf[i] > 0.5 and names.get(int(cls[i]), str(cls[i])) != 'Bullseye':
            chosen = i
            break
    if chosen is None:
        return 'NA'

    x1, y1, x2, y2 = xyxy[chosen]
    cls_id = int(cls[chosen])
    name = names.get(cls_id, str(cls_id))

    # if you have draw_own_bbox:
    # draw_own_bbox(np.array(img), x1, y1, x2, y2, f"{name}-{cls_id}")

    return name


def stitch_image():
    """
    Stitches the images in the folder together and saves it into runs/stitched folder
    """
    # Initialize path to save stitched image
    imgFolder = 'runs'
    stitchedPath = os.path.join(imgFolder, f'stitched-{int(time.time())}.jpeg')

    # Find all files that ends with ".jpg" (this won't match the stitched images as we name them ".jpeg")
    imgPaths = glob.glob(os.path.join(imgFolder+"/detect/*/", "*.jpg"))
    # Open all images
    images = [Image.open(x) for x in imgPaths]
    # Get the width and height of each image
    width, height = zip(*(i.size for i in images))
    # Calculate the total width and max height of the stitched image, as we are stitching horizontally
    total_width = sum(width)
    max_height = max(height)
    stitchedImg = Image.new('RGB', (total_width, max_height))
    x_offset = 0

    # Stitch the images together
    for im in images:
        stitchedImg.paste(im, (x_offset, 0))
        x_offset += im.size[0]
    # Save the stitched image to the path
    stitchedImg.save(stitchedPath)

    # Move original images to "originals" subdirectory
    for img in imgPaths:
        shutil.move(img, os.path.join(
            "runs", "originals", os.path.basename(img)))

    return stitchedImg

def stitch_image_own():
    """
    Stitches the images in the folder together and saves it into own_results folder

    Basically similar to stitch_image() but with different folder names and slightly different drawing of bounding boxes and text
    """
    imgFolder = 'own_results'
    stitchedPath = os.path.join(imgFolder, f'stitched-{int(time.time())}.jpeg')

    imgPaths = glob.glob(os.path.join(imgFolder+"/annotated_image_*.jpg"))
    imgTimestamps = [imgPath.split("_")[-1][:-4] for imgPath in imgPaths]
    
    sortedByTimeStampImages = sorted(zip(imgPaths, imgTimestamps), key=lambda x: x[1])

    images = [Image.open(x[0]) for x in sortedByTimeStampImages]
    width, height = zip(*(i.size for i in images))
    total_width = sum(width)
    max_height = max(height)
    stitchedImg = Image.new('RGB', (total_width, max_height))
    x_offset = 0

    for im in images:
        stitchedImg.paste(im, (x_offset, 0))
        x_offset += im.size[0]
    stitchedImg.save(stitchedPath)

    return stitchedImg
