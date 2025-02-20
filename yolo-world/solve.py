from ultralytics import YOLO
import cv2
import numpy as np

# Загрузка модели
model = YOLO("best.pt")  # Укажите путь к вашей модели

# Подключение к веб-камере (0 — основная камера)
cap = cv2.VideoCapture(0)

# Проверка, открылась ли камера
if not cap.isOpened():
    print("Ошибка: не удалось подключиться к веб-камере")
    exit()

# Основной цикл обработки видео
while True:
    # Чтение кадра с веб-камеры
    ret, frame = cap.read()
    if not ret:
        print("Ошибка: не удалось получить кадр")
        break

    # Предсказание объектов с помощью модели
    results = model(frame, conf=0.25)  # Порог уверенности 0.25

    # Отрисовка результатов на кадре
    for result in results:
        boxes = result.boxes  # Данные о боксах
        for box in boxes:
            # Координаты прямоугольника
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            # Уверенность (score)
            score = box.conf[0].item()
            # Класс объекта
            class_id = int(box.cls[0].item())
            class_name = model.names[class_id]

            # Отрисовка прямоугольника
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            # Текст с классом и уверенностью
            label = f"{class_name} {score:.2f}"
            cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Отображение кадра
    cv2.imshow("YOLO Detection", frame)

    # Выход по нажатию 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Освобождение ресурсов
cap.release()
cv2.destroyAllWindows()
