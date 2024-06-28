#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

# Inicializar el nodo ROS
rospy.init_node('person_detection_node', anonymous=True)

# Crear los publicadores
video_pub = rospy.Publisher('/video_detections', Image, queue_size=10)
position_pub = rospy.Publisher('/position', Int32MultiArray, queue_size=10)

# Crear el objeto CvBridge
bridge = CvBridge()

# Cargar el modelo YOLO
model = YOLO("yolov8n.pt")

# Ajustar el umbral de confianza
conf_threshold = 0.7

def image_callback(msg):
    # Convertir la imagen de ROS a OpenCV
    frame = bridge.imgmsg_to_cv2(msg, "bgr8")

    # Realizar la predicción en el frame actual con el umbral de confianza ajustado
    results = model.predict(source=frame, conf=conf_threshold, classes=[0])

    # Filtrar las detecciones para la clase de personas (generalmente la clase 0)
    person_detections = [box for box in results[0].boxes if box.cls[0] == 0]
    
    positions = Int32MultiArray()
    
    # Dibujar las detecciones y obtener las coordenadas
    for box in person_detections:
        bbox = box.xyxy[0].cpu().numpy()  # Obtener las coordenadas del bounding box
        x1, y1, x2, y2 = map(int, bbox)  # Convertir las coordenadas a enteros
        cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)  # Dibujar el rectángulo alrededor de la persona
        cv2.putText(frame, f"Person: {box.conf[0]:.2f}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0), 2)  # Añadir etiqueta
        
        # Añadir coordenadas al mensaje
        positions.data.extend([x1, y1, x2, y2])
        
    # Publicar las coordenadas en el tópico /position
    position_pub.publish(positions)

    # Convertir la imagen procesada de OpenCV a ROS
    img_msg = bridge.cv2_to_imgmsg(frame, "bgr8")
    
    # Publicar la imagen en el tópico /video_detections
    video_pub.publish(img_msg)

# Suscribirse al tópico /camera
image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, image_callback)

# Mantener el nodo en funcionamiento
rospy.spin()
