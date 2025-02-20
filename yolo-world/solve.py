from ultralytics import YOLO
import cv2
import numpy as np

# �������� ������
model = YOLO("best.pt")  # ������� ���� � ����� ������

# ����������� � ���-������ (0 � �������� ������)
cap = cv2.VideoCapture(0)

# ��������, ��������� �� ������
if not cap.isOpened():
    print("������: �� ������� ������������ � ���-������")
    exit()

# �������� ���� ��������� �����
while True:
    # ������ ����� � ���-������
    ret, frame = cap.read()
    if not ret:
        print("������: �� ������� �������� ����")
        break

    # ������������ �������� � ������� ������
    results = model(frame, conf=0.25)  # ����� ����������� 0.25

    # ��������� ����������� �� �����
    for result in results:
        boxes = result.boxes  # ������ � ������
        for box in boxes:
            # ���������� ��������������
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            # ����������� (score)
            score = box.conf[0].item()
            # ����� �������
            class_id = int(box.cls[0].item())
            class_name = model.names[class_id]

            # ��������� ��������������
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            # ����� � ������� � ������������
            label = f"{class_name} {score:.2f}"
            cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # ����������� �����
    cv2.imshow("YOLO Detection", frame)

    # ����� �� ������� 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# ������������ ��������
cap.release()
cv2.destroyAllWindows()
