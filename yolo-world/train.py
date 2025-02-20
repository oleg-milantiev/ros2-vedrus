from ultralytics import YOLO

MODEL_PATH = "yolov8l.pt"
DATA_PATH = "./yolo/data.yaml"
EPOCHS = 50
IMG_SIZE = 640
BATCH_SIZE = 16

model = YOLO(MODEL_PATH)
model.train(data=DATA_PATH, epochs=EPOCHS, imgsz=IMG_SIZE, batch=BATCH_SIZE, device=0)

if __name__ == "__main__":
    print("Training completed.")
