from ultralytics import YOLO
import cv2
import time

# Загрузка модели
model = YOLO("runs/detect/train7/weights/best.pt")  # Укажите путь к вашей модели

# Подключение к веб-камере
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Ошибка: не удалось подключиться к веб-камере")
    exit()

# Переменные для расчета FPS
prev_time = time.time()
fps = 0

while True:
    ret, frame = cap.read()
    if not ret:
        print("Ошибка: не удалось получить кадр")
        break

    # Предсказание объектов
    results = model(frame, conf=0.25)

    # Отрисовка результатов
    for result in results:
        boxes = result.boxes
        for box in boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            score = box.conf[0].item()
            class_id = int(box.cls[0].item())
            class_name = model.names[class_id]
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = f"{class_name} {score:.2f}"
            cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Расчет FPS
    curr_time = time.time()
    fps = 1 / (curr_time - prev_time)
    prev_time = curr_time

    # Отрисовка FPS на кадре
    cv2.putText(frame, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    # Отображение кадра
    cv2.imshow("YOLO Detection", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
