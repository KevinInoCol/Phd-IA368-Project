#!/usr/bin/env python3
"""Odometría degradada con errores sistemáticos: /pepper/odom -> /pepper/odom_noisy.

El plugin de la base (gazebo_ros_model_velocity) sólo suma ruido blanco de media cero a la
velocidad, que se promedia al integrar: en la ruta de la oficina la odometría sola acaba con
~0.15 m de ATE, tan poco que el SLAM apenas puede mejorarla. La odometría de un robot real
deriva sobre todo por errores sistemáticos (radio de rueda mal calibrado, patinaje de las ruedas
omnidireccionales al girar, deriva del giróscopo). Este nodo los añade a cada incremento de la
odometría del plugin, en el marco del robot:

    avance' = avance · (1 + trans_scale_error)
    giro'   = giro · (1 + rot_scale_error) + yaw_drift · dt     (deriva sólo con el robot en marcha)

Publica /pepper/odom_noisy y la TF odom_noisy -> base_noisy en un árbol de TF APARTE: base_noisy
es el mismo punto físico que base_footprint, pero sin conexión con el árbol real (un frame no
puede tener dos padres). El SLAM se configura con odom_frame=odom_noisy y base_frame=base_noisy
y sólo ve esta odometría; los scans que use deben ir en base_noisy (scan_fusion con
base_frame:=base_noisy, o los scans en base_footprint reetiquetados con `relabel_scans`).

Niveles (`preset`), calibrados offline sobre la ruta de 19 puntos (47 m, 25 rad girados):
    medio: 1.5 % avance, 2.5 % giro, 0.001 rad/s
    alto : 3 % avance,   5 % giro,   0.002 rad/s  -> ATE de la odometría sola ~0.9 m
Los parámetros sueltos sobrescriben el preset.
"""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster

PRESETS = {
    "ninguno": (0.0, 0.0, 0.0),
    "medio": (0.015, 0.025, 0.001),
    "alto": (0.03, 0.05, 0.002),
}


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class OdomNoise(Node):
    def __init__(self):
        super().__init__("odom_noise")
        preset = self.declare_parameter("preset", "alto").value
        if preset not in PRESETS:
            raise ValueError(f"preset desconocido: {preset} (opciones: {', '.join(PRESETS)})")
        et, er, drift = PRESETS[preset]
        self.trans_scale = float(self.declare_parameter("trans_scale_error", et).value)
        self.rot_scale = float(self.declare_parameter("rot_scale_error", er).value)
        self.yaw_drift = float(self.declare_parameter("yaw_drift", drift).value)
        self.odom_frame = self.declare_parameter("odom_frame", "odom_noisy").value
        self.base_frame = self.declare_parameter("base_frame", "base_noisy").value
        # Pares "entrada:salida" de scans en base_footprint a reetiquetar en base_frame
        relabel = self.declare_parameter("relabel_scans", [""]).value

        self.prev = None          # (x, y, yaw, t) de la odometría del plugin
        self.x = self.y = self.yaw = 0.0
        self.tf = TransformBroadcaster(self)
        self.pub = self.create_publisher(Odometry, "/pepper/odom_noisy", 10)
        self.create_subscription(Odometry, "/pepper/odom", self.odom_cb, 50)
        self.relabel_pubs = []
        for pair in relabel:
            if not pair:
                continue
            src, dst = pair.split(":")
            pub = self.create_publisher(LaserScan, dst, qos_profile_sensor_data)
            self.create_subscription(LaserScan, src, lambda m, p=pub: self.relabel_cb(m, p),
                                     qos_profile_sensor_data)
            self.relabel_pubs.append(pub)
        self.get_logger().info(
            f"odom_noise ({preset}): avance {100 * self.trans_scale:+.1f} %, giro "
            f"{100 * self.rot_scale:+.1f} %, deriva {self.yaw_drift} rad/s -> /pepper/odom_noisy "
            f"({self.odom_frame} -> {self.base_frame})")

    def odom_cb(self, msg):
        p = msg.pose.pose
        cur = (p.position.x, p.position.y, yaw_of(p.orientation),
               msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        if self.prev is not None:
            px, py, pyaw, pt = self.prev
            # Incremento en el marco del robot (la odometría del plugin ya lo integró)
            dx, dy = cur[0] - px, cur[1] - py
            c, s = math.cos(pyaw), math.sin(pyaw)
            bx, by = c * dx + s * dy, -s * dx + c * dy
            dth = wrap(cur[2] - pyaw)
            dt = max(0.0, cur[3] - pt)
            moving = abs(bx) + abs(by) + abs(dth) > 1e-6
            dth = dth * (1.0 + self.rot_scale) + (self.yaw_drift * dt if moving else 0.0)
            bx *= 1.0 + self.trans_scale
            by *= 1.0 + self.trans_scale
            mid = self.yaw + 0.5 * dth
            self.x += bx * math.cos(mid) - by * math.sin(mid)
            self.y += bx * math.sin(mid) + by * math.cos(mid)
            self.yaw = wrap(self.yaw + dth)
        self.prev = cur

        qz, qw = math.sin(0.5 * self.yaw), math.cos(0.5 * self.yaw)
        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.odom_frame
        out.child_frame_id = self.base_frame
        out.pose.pose.position.x = self.x
        out.pose.pose.position.y = self.y
        out.pose.pose.orientation.z = qz
        out.pose.pose.orientation.w = qw
        out.pose.covariance = msg.pose.covariance
        out.twist = msg.twist
        self.pub.publish(out)

        tf = TransformStamped()
        tf.header = out.header
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf.sendTransform(tf)

    def relabel_cb(self, msg, pub):
        msg.header.frame_id = self.base_frame
        pub.publish(msg)


def main():
    rclpy.init()
    node = OdomNoise()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
