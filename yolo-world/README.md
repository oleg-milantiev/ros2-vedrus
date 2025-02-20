# YOLO-World Frame Processor

This script processes video input from a webcam or MP4 file, detects specified objects using YOLO-World, and saves frames with detections for YOLOv8 training.

## Usage
1. Install dependencies:
pip install -r requirements.txt

2. Run the script:
- For webcam: `python main.py`
- For MP4 file: `python main.py path/to/video.mp4`

## Requirements
- Python 3.8+
- NVIDIA GPU (recommended) with CUDA support for faster inference
- Pre-trained `yolov8s-world.pt` model (download from Ultralytics)

## Configuration
- Edit `CLASSES` in `main.py` to specify objects to detect (e.g., `["tree", "house", "barn"]`).
- Output is saved to `./out/` with filenames in YYMMDDNN format (e.g., `25022000001.png`).

## Output Format
- **Images**: PNG files (640x640) in `./out/`.
- **Labels**: TXT files in YOLO format (`class_id x_center y_center width height`), normalized to [0, 1].

## Notes
- Ensure the YOLO-World model file (`yolov8s-world.pt`) is in the working directory.
- Frames without detections are skipped.