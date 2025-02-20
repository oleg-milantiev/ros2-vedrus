import cv2
import numpy as np
from ultralytics import YOLO
import os
from datetime import datetime

CLASSES = ["person", "mug", "teaspoon"]
MODEL = YOLO("yolov8x-world.pt")
MODEL.set_classes(CLASSES)
OUTPUT_DIR = "./out"

if not os.path.exists(OUTPUT_DIR):
    os.makedirs(OUTPUT_DIR)

def get_max_frame_count():
    files = [f for f in os.listdir(OUTPUT_DIR) if f.endswith(".png")]
    if not files:
        return 0
    max_count = max(int(f[-9:-4]) for f in files if len(f) >= 9 and f[-9:-4].isdigit())
    return max_count

def process_frame(frame, frame_count):
    frame_resized = cv2.resize(frame, (640, 640))
    results = MODEL.predict(frame_resized, conf=0.25)
    
    if len(results[0].boxes) > 0:
        timestamp = datetime.now().strftime("%y%m%d")
        filename = f"{timestamp}{frame_count:05d}"
        
        cv2.imwrite(f"{OUTPUT_DIR}/{filename}.png", frame_resized)
        
        with open(f"{OUTPUT_DIR}/{filename}.txt", "w") as f:
            for box in results[0].boxes:
                cls_id = int(box.cls)
                x, y, w, h = box.xywh[0].tolist()
                x_center = x / 640
                y_center = y / 640
                width = w / 640
                height = h / 640
                f.write(f"{cls_id} {x_center} {y_center} {width} {height}\n")

def main(source):
    frame_count = get_max_frame_count() + 1
    
    if isinstance(source, str) and source.endswith(".mp4"):
        cap = cv2.VideoCapture(source)
    else:
        cap = cv2.VideoCapture(int(source))
    
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        
        process_frame(frame, frame_count)
        frame_count += 1
    
    cap.release()

if __name__ == "__main__":
    import sys
    source = sys.argv[1] if len(sys.argv) > 1 else 0
    main(source)