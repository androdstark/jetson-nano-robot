#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
esp32_bridge.py — ROS Melodic (Python 2.7)
Puente Serial entre Jetson Nano y ESP32.

Suscribe  : /cmd_vel  (geometry_msgs/Twist)
Publica   : /odom     (nav_msgs/Odometry)
Broadcast : odom → base_link  (tf)
"""

from __future__ import print_function
import rospy
import serial
import json
import math
import threading

from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
import tf
import tf2_ros


class Esp32Bridge(object):

    def __init__(self):
        rospy.init_node('esp32_bridge')

        # ── Parámetros ROS ────────────────────────────────────────────────
        self.port         = rospy.get_param('~port',          '/dev/ttyUSB0')
        self.baud         = rospy.get_param('~baud',          115200)
        self.wheel_base   = rospy.get_param('~wheel_base',    0.20)
        self.wheel_radius = rospy.get_param('~wheel_radius',  0.033)
        self.ticks_per_rev= rospy.get_param('~ticks_per_rev', 20)
        self.cmd_timeout  = rospy.get_param('~cmd_timeout',   0.5)  # s

        # ── Estado odométrico ─────────────────────────────────────────────
        self.x     = 0.0
        self.y     = 0.0
        self.theta = 0.0

        self.prev_ticks_l = None
        self.prev_ticks_r = None
        self.odom_lock    = threading.Lock()

        # ── Serial ────────────────────────────────────────────────────────
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            rospy.loginfo("Serial abierto: %s @ %d", self.port, self.baud)
        except serial.SerialException as e:
            rospy.logfatal("No se pudo abrir %s: %s", self.port, e)
            raise SystemExit(1)

        # ── Publishers & TF ───────────────────────────────────────────────
        self.odom_pub   = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.tf_br      = tf.TransformBroadcaster()

        # ── Subscriber /cmd_vel ───────────────────────────────────────────
        self.last_cmd_time = rospy.Time.now()
        self.cmd_lock      = threading.Lock()
        self.last_cmd      = Twist()
        rospy.Subscriber('/cmd_vel', Twist, self.cb_cmd_vel, queue_size=1)

        # ── Timers ────────────────────────────────────────────────────────
        # Leer Serial en hilo separado para no bloquear
        self.read_thread = threading.Thread(target=self.read_loop)
        self.read_thread.daemon = True
        self.read_thread.start()

        # Safety timeout: publicar v=0 si no llega cmd_vel en X segundos
        rospy.Timer(rospy.Duration(0.05), self.safety_timeout_cb)

        rospy.loginfo("esp32_bridge listo.")

    # ─── Callback /cmd_vel ────────────────────────────────────────────────────
    def cb_cmd_vel(self, msg):
        with self.cmd_lock:
            self.last_cmd      = msg
            self.last_cmd_time = rospy.Time.now()
        self.send_cmd(msg.linear.x, msg.angular.z)

    # ─── Safety timeout ───────────────────────────────────────────────────────
    def safety_timeout_cb(self, event):
        with self.cmd_lock:
            age = (rospy.Time.now() - self.last_cmd_time).to_sec()
        if age > self.cmd_timeout:
            self.send_cmd(0.0, 0.0)

    # ─── Serializar y enviar al ESP32 ─────────────────────────────────────────
    def send_cmd(self, v, w):
        payload = json.dumps({'v': round(float(v), 4),
                              'w': round(float(w), 4)}) + '\n'
        try:
            self.ser.write(payload.encode('utf-8'))
        except serial.SerialException as e:
            rospy.logerr_throttle(5.0, "Error escritura serial: %s", e)

    # ─── Hilo de lectura Serial → odometría ───────────────────────────────────
    def read_loop(self):
        while not rospy.is_shutdown():
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            except serial.SerialException as e:
                rospy.logerr_throttle(5.0, "Error lectura serial: %s", e)
                continue

            if not line:
                continue

            try:
                data = json.loads(line)
            except ValueError:
                continue

            # Campos esperados: lv, rv, ticks_l, ticks_r
            if not all(k in data for k in ('lv', 'rv', 'ticks_l', 'ticks_r')):
                continue

            self.process_odom(data)

    # ─── Cinemática diferencial y publicación ─────────────────────────────────
    def process_odom(self, data):
        ticks_l = data['ticks_l']
        ticks_r = data['ticks_r']
        lv      = float(data['lv'])   # m/s rueda izq (ya calculado en ESP32)
        rv      = float(data['rv'])   # m/s rueda der

        now = rospy.Time.now()

        with self.odom_lock:
            if self.prev_ticks_l is None:
                # Primera lectura — inicializar sin publicar
                self.prev_ticks_l = ticks_l
                self.prev_ticks_r = ticks_r
                self.prev_odom_time = now
                return

            dt = (now - self.prev_odom_time).to_sec()
            self.prev_odom_time = now

            # Calcular desplazamiento desde ticks (más preciso que lv/rv)
            meters_per_tick = (2.0 * math.pi * self.wheel_radius) / self.ticks_per_rev

            delta_l = (ticks_l - self.prev_ticks_l) * meters_per_tick
            delta_r = (ticks_r - self.prev_ticks_r) * meters_per_tick
            self.prev_ticks_l = ticks_l
            self.prev_ticks_r = ticks_r

            # Inferir signo de desplazamiento desde velocidades reportadas
            if lv < 0.0:
                delta_l = -abs(delta_l)
            if rv < 0.0:
                delta_r = -abs(delta_r)

            d_center = (delta_l + delta_r) / 2.0
            d_theta  = (delta_r - delta_l) / self.wheel_base

            # Integrar pose
            self.x     += d_center * math.cos(self.theta + d_theta / 2.0)
            self.y     += d_center * math.sin(self.theta + d_theta / 2.0)
            self.theta += d_theta

            # Velocidades instantáneas
            v_linear  = d_center / dt if dt > 0 else 0.0
            v_angular = d_theta  / dt if dt > 0 else 0.0

            # ── Quaternion ────────────────────────────────────────────────
            quat = tf.transformations.quaternion_from_euler(0, 0, self.theta)

            # ── TF odom → base_link ───────────────────────────────────────
            self.tf_br.sendTransform(
                (self.x, self.y, 0.0),
                quat,
                now,
                'base_link',
                'odom'
            )

            # ── nav_msgs/Odometry ─────────────────────────────────────────
            odom = Odometry()
            odom.header.stamp    = now
            odom.header.frame_id = 'odom'
            odom.child_frame_id  = 'base_link'

            odom.pose.pose.position.x  = self.x
            odom.pose.pose.position.y  = self.y
            odom.pose.pose.position.z  = 0.0
            odom.pose.pose.orientation = Quaternion(*quat)

            # Covarianza diagonal simple (ajustar según robot real)
            odom.pose.covariance[0]  = 0.01   # x
            odom.pose.covariance[7]  = 0.01   # y
            odom.pose.covariance[35] = 0.05   # yaw

            odom.twist.twist.linear.x  = v_linear
            odom.twist.twist.angular.z = v_angular
            odom.twist.covariance[0]   = 0.01
            odom.twist.covariance[35]  = 0.05

            self.odom_pub.publish(odom)

    # ─── Shutdown limpio ──────────────────────────────────────────────────────
    def shutdown(self):
        self.send_cmd(0.0, 0.0)
        self.ser.close()
        rospy.loginfo("esp32_bridge cerrado.")


if __name__ == '__main__':
    try:
        bridge = Esp32Bridge()
        rospy.on_shutdown(bridge.shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
