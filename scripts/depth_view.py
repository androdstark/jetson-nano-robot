#!/usr/bin/env python
"""Visualización del depth stream de Orbbec Astra Pro Plus.
No usa GPU. Requiere ROS Melodic + ros_astra_camera corriendo.

Uso:
    # Terminal 1
    roslaunch astra_camera astra_pro.launch
    # Terminal 2 (con DISPLAY configurado)
    export DISPLAY=:0
    python depth_view.py
"""
import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np


def callback(msg):
    depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
    viz = cv2.convertScaleAbs(depth, alpha=0.05)
    cv2.imshow("Depth — Orbbec Astra Pro Plus", viz)
    cv2.waitKey(1)


if __name__ == '__main__':
    rospy.init_node('depth_viewer')
    rospy.Subscriber('/camera/depth/image_raw', Image, callback)
    rospy.loginfo("depth_view.py iniciado — esperando frames en /camera/depth/image_raw")
    rospy.spin()
    cv2.destroyAllWindows()
