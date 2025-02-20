import os
import shutil
from sklearn.model_selection import train_test_split

SOURCE_DIR = "./out"
TARGET_DIR = "./yolo"
CLASSES = ["person", "mug", "teaspoon"]

def create_yolo_structure():
    for folder in ["train/images", "train/labels", "val/images", "val/labels"]:
        os.makedirs(os.path.join(TARGET_DIR, folder), exist_ok=True)

def split_and_copy_files():
    image_files = [f for f in os.listdir(SOURCE_DIR) if f.endswith(".png")]
    train_files, val_files = train_test_split(image_files, test_size=0.2, random_state=42)
    
    for img_file in train_files:
        shutil.copy(os.path.join(SOURCE_DIR, img_file), os.path.join(TARGET_DIR, "train/images"))
        txt_file = img_file.replace(".png", ".txt")
        shutil.copy(os.path.join(SOURCE_DIR, txt_file), os.path.join(TARGET_DIR, "train/labels"))
    
    for img_file in val_files:
        shutil.copy(os.path.join(SOURCE_DIR, img_file), os.path.join(TARGET_DIR, "val/images"))
        txt_file = img_file.replace(".png", ".txt")
        shutil.copy(os.path.join(SOURCE_DIR, txt_file), os.path.join(TARGET_DIR, "val/labels"))

def create_data_yaml():
    yaml_content = f"""path: {os.path.abspath(TARGET_DIR)}
train: train/images
val: val/images
nc: {len(CLASSES)}
names: {CLASSES}
"""
    with open(os.path.join(TARGET_DIR, "data.yaml"), "w") as f:
        f.write(yaml_content)

if __name__ == "__main__":
    create_yolo_structure()
    split_and_copy_files()
    create_data_yaml()