from ultralytics import YOLO
import cv2

# Cargar el modelo YOLO
model = YOLO("yolov8n.pt")

# Configurar la captura de video desde la cámara (source='0')
cap = cv2.VideoCapture(0)

# Ajustar el umbral de confianza
conf_threshold = 0.7

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break
    
    # Realizar la predicción en el frame actual con el umbral de confianza ajustado
    results = model.predict(source=frame, conf=conf_threshold, classes=[0])

    # Filtrar las detecciones para la clase de personas (generalmente la clase 0)
    person_detections = [box for box in results[0].boxes if box.cls[0] == 0]
    
    # Dibujar las detecciones y obtener las coordenadas
    for box in person_detections:
        bbox = box.xyxy[0].cpu().numpy()  # Obtener las coordenadas del bounding box
        x1, y1, x2, y2 = map(int, bbox)  # Convertir las coordenadas a enteros
        cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)  # Dibujar el rectángulo alrededor de la persona
        cv2.putText(frame, f"Person: {box.conf[0]:.2f}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0), 2)  # Añadir etiqueta
        
        # Imprimir las coordenadas
        print(f"Person detected at: x1={x1}, y1={y1}, x2={x2}, y2={y2}")

    # Mostrar el frame con las detecciones
    cv2.imshow("Person Detection", frame)

    # Salir del loop si se presiona la tecla 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Liberar los recursos
cap.release()
cv2.destroyAllWindows()
