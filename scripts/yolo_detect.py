#!/usr/bin/env python3
"""Detección de objetos con YOLOv8n usando la cámara RGB del Orbbec.
Requiere: pip install ultralytics
FPS esperado en Jetson Nano: ~15-20 fps con YOLOv8n
"""
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

try:
    from ultralytics import YOLO
    model = YOLO('yolov8n.pt')
except ImportError:
    rospy.logerr("ultralytics no instalado. Ejecutar: pip3 install ultralytics")
    raise

bridge = CvBridge()


def callback(msg):
    frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    results = model(frame, verbose=False)
    annotated = results[0].plot()
    cv2.imshow("YOLOv8n Detection", annotated)
    cv2.waitKey(1)


if __name__ == '__main__':
    rospy.init_node('yolo_detector')
    rospy.Subscriber('/camera/color/image_raw', Image, callback)
    rospy.loginfo("yolo_detect.py iniciado — detectando objetos")
    rospy.spin()
    cv2.destroyAllWindows()
