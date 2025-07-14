import cv2
from ultralytics import YOLO
import os

# PyTorch güvenlik sorunu için
os.environ['TORCH_WEIGHTS_ONLY'] = 'False'

# Model yükle
model = YOLO('yolov8x.pt')

# Kamerayı aç
cap = cv2.VideoCapture(0)

# Kamera ayarları
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

print("Nesne tespiti başlatılıyor...")
print("Çıkmak için 'q' tuşuna basın")

while True:
    # Frame oku
    ret, frame = cap.read()
    if not ret:
        break
    
    # YOLOv8 ile tespit yap
    results = model(frame, conf=0.5, verbose=False)
    
    # Sonuçları çiz
    annotated_frame = results[0].plot()
    
    # Göster
    cv2.imshow('YOLOv8 Object Detection', annotated_frame)
    
    # Çıkış
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Temizle
cap.release()
cv2.destroyAllWindows()