#!/usr/bin/env python3
"""Fusión de los 3 láseres de Pepper para SLAM: /pepper/scan_fused.

Pepper tiene tres láseres bajos (frontal, izquierdo y derecho) de 15 rayos en ±30° cada uno:
45 rayos en total, con huecos de ~30° entre sectores. El nodo heredado del V8
(pepper_gazebo_plugin/laser_publisher.py -> /pepper/laser_2) los junta en un único scan, pero:

  1. Los rayos sin eco quedan en NaN, y Karto (slam_toolbox) ignora los NaN: el espacio libre
     sólo se limpia a lo largo de los ~45 rayos que tocan algo.
  2. Cada scan tiene como mucho 45 puntos, muy poco para el scan matching.

Este nodo añade dos opciones:

  - Acumulación temporal (accumulate = N, por defecto 4): guarda los impactos de los últimos N
    scans en el frame odom y los reproyecta a base_footprint en el instante actual. Con la
    odometría de Pepper (deriva lenta) unos pocos scans (N·0.16 s) apenas acumulan error y el
    scan resultante tiene hasta N·45 puntos.
  - Espacio libre (free_space, por defecto DESACTIVADO): un rayo sin eco se publica con
    free_range (> max_laser_range de slam_toolbox y < range_max del mensaje). Karto lo recorta a
    max_laser_range y lo traza como libre (OccupancyGrid::AddScan). PERO el scan matcher de
    Karto (GridIndexLookup::ComputeOffsets) usa GetPointReadings() sin filtrar y sólo descarta
    NaN/inf: esas lecturas entran al matching como puntos falsos a free_range. En los
    experimentos empeora la localización (ATE 0.31 m frente a 0.16 m con accumulate=4 y
    slam_toolbox por defecto) y apenas gana área libre, porque la acumulación ya limpia casi todo.
    Se deja como opción para documentar el resultado.

Los sectores que ningún láser cubre (y los rayos sin eco si free_space=false) quedan en +inf,
que Karto ignora tanto en el mapa como en el matching.

Salida: LaserScan en base_footprint, 360° con num_rays rayos (cada rayo guarda el impacto más
cercano). Así slam_toolbox lo usa igual que laser_2: ros2 launch ... scan:=/pepper/scan_fused
"""

import math
from collections import deque

import numpy as np
import rclpy
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

SCAN_TOPICS = ("/pepper/scan_left", "/pepper/scan_front", "/pepper/scan_right")


def tf_to_matrix(tf):
    """Matriz homogénea 3x3 (SE(2)) de un TransformStamped, ignorando z, roll y pitch."""
    t = tf.transform.translation
    q = tf.transform.rotation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, t.x], [s, c, t.y], [0.0, 0.0, 1.0]])


def tf_to_matrix_3d(tf):
    """Matriz homogénea 4x4 completa (los láseres de Pepper están en frames proyectados al suelo,
    pero así no se asume nada)."""
    t = tf.transform.translation
    q = tf.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w), t.x],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w), t.y],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y), t.z],
        [0.0, 0.0, 0.0, 1.0],
    ])


class ScanFusion(Node):
    def __init__(self):
        super().__init__("scan_fusion")
        self.base_frame = self.declare_parameter("base_frame", "base_footprint").value
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        # Frame del cuerpo con las TF estáticas de los láseres. Con odometría degradada
        # (odom_noise.py) base_frame es base_noisy, en otro árbol de TF: la geometría de los
        # láseres se sigue leyendo de base_footprint (el mismo punto físico).
        self.sensor_frame = self.declare_parameter("sensor_frame", "").value or self.base_frame
        self.num_rays = int(self.declare_parameter("num_rays", 360).value)
        self.accumulate = max(1, int(self.declare_parameter("accumulate", 4).value))
        self.free_space = bool(self.declare_parameter("free_space", False).value)
        # max_laser_range de slam_toolbox (7.0) < free_range < range_max del mensaje
        self.free_range = float(self.declare_parameter("free_range", 7.5).value)
        self.range_min_out = float(self.declare_parameter("range_min", 0.1).value)
        self.range_max_out = float(self.declare_parameter("range_max", 8.0).value)
        self.tf_wait = float(self.declare_parameter("tf_wait", 0.5).value)
        self.output_topic = self.declare_parameter("output_topic", "/pepper/scan_fused").value

        self.angle_increment = 2.0 * math.pi / self.num_rays
        self.angle_min = -math.pi

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(LaserScan, self.output_topic, 5)
        subs = [Subscriber(self, LaserScan, t, qos_profile=qos_profile_sensor_data)
                for t in SCAN_TOPICS]
        self.sync = ApproximateTimeSynchronizer(subs, queue_size=10, slop=0.05)
        self.sync.registerCallback(self.scans_cb)

        # Triples de scans esperando la TF odom <- base en su instante
        self.pending = deque(maxlen=20)
        # Impactos de los últimos N scans en el frame odom: arrays (M, 2)
        self.history = deque(maxlen=self.accumulate)
        self.create_timer(0.02, self.process_pending)
        self.get_logger().info(
            f"scan_fusion: {', '.join(SCAN_TOPICS)} -> {self.output_topic} "
            f"({self.num_rays} rayos, accumulate={self.accumulate}, free_space={self.free_space})")

    def scans_cb(self, left, front, right):
        self.pending.append((front.header.stamp, (left, front, right)))

    def process_pending(self):
        while self.pending:
            stamp, scans = self.pending[0]
            t = Time.from_msg(stamp)
            if self.tf_buffer.can_transform(self.odom_frame, self.base_frame, t):
                self.pending.popleft()
                self.fuse(stamp, scans)
                continue
            # La odometría llega a 20 Hz: esperar un poco a que cubra el instante del scan
            age = (self.get_clock().now() - t).nanoseconds * 1e-9
            if age > self.tf_wait:
                self.pending.popleft()
                self.get_logger().warn(f"Sin TF {self.odom_frame} <- {self.base_frame} "
                                       "para el scan: descartado", throttle_duration_sec=5.0)
                continue
            break

    def sensor_points(self, scan):
        """(impactos, rayos libres) del scan en base_footprint: arrays (K, 2) y (L, 2)."""
        try:
            tf = self.tf_buffer.lookup_transform(self.sensor_frame, scan.header.frame_id, Time())
        except TransformException as ex:
            self.get_logger().warn(f"Sin TF {self.sensor_frame} <- {scan.header.frame_id}: {ex}",
                                   throttle_duration_sec=5.0)
            return None
        m = tf_to_matrix_3d(tf)
        r = np.asarray(scan.ranges, dtype=np.float64)
        a = scan.angle_min + np.arange(len(r)) * scan.angle_increment
        hit = np.isfinite(r) & (r >= scan.range_min) & (r < scan.range_max)
        # Sin eco: inf (Gazebo) o >= range_max. Los NaN no se sabe qué son: se descartan
        free = ~np.isnan(r) & ~hit & ((r >= scan.range_max) | np.isinf(r))

        def to_base(rng, ang):
            p = np.column_stack([rng * np.cos(ang), rng * np.sin(ang),
                                 np.zeros_like(rng), np.ones_like(rng)])
            return (m @ p.T).T[:, :2]

        hits = to_base(r[hit], a[hit])
        frees = to_base(np.full(int(free.sum()), scan.range_max), a[free])
        return hits, frees

    def fuse(self, stamp, scans):
        parts = [self.sensor_points(s) for s in scans]
        if any(p is None for p in parts):
            return
        hits = np.vstack([p[0] for p in parts])
        frees = np.vstack([p[1] for p in parts])

        try:
            odom_base = tf_to_matrix(self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame, Time.from_msg(stamp)))
        except TransformException as ex:
            self.get_logger().warn(f"Sin TF odom: {ex}", throttle_duration_sec=5.0)
            return

        # Guardar los impactos actuales en odom y traer todos los guardados a la base actual
        h = np.column_stack([hits, np.ones(len(hits))])
        self.history.append((odom_base @ h.T).T[:, :2])
        acc = np.vstack(self.history)
        base_odom = np.linalg.inv(odom_base)
        acc = (base_odom @ np.column_stack([acc, np.ones(len(acc))]).T).T[:, :2]

        ranges = np.full(self.num_rays, np.inf)
        if self.free_space and len(frees):
            idx = self.bin_index(frees)
            ranges[idx] = self.free_range
        if len(acc):
            dist = np.hypot(acc[:, 0], acc[:, 1])
            idx = self.bin_index(acc)
            # El impacto más cercano de cada rayo manda (sobre el libre y sobre los demás)
            hit_ranges = np.full(self.num_rays, np.inf)
            np.minimum.at(hit_ranges, idx, dist)
            has_hit = np.isfinite(hit_ranges)
            ranges[has_hit] = hit_ranges[has_hit]

        msg = LaserScan()
        msg.header.stamp = stamp
        msg.header.frame_id = self.base_frame
        msg.angle_min = self.angle_min
        msg.angle_increment = self.angle_increment
        msg.angle_max = self.angle_min + (self.num_rays - 1) * self.angle_increment
        msg.scan_time = 0.16
        msg.range_min = self.range_min_out
        msg.range_max = self.range_max_out
        msg.ranges = ranges.astype(np.float32).tolist()
        self.pub.publish(msg)

    def bin_index(self, pts):
        ang = np.arctan2(pts[:, 1], pts[:, 0])
        idx = np.floor((ang - self.angle_min) / self.angle_increment).astype(int)
        return np.clip(idx, 0, self.num_rays - 1)


def main():
    rclpy.init()
    node = ScanFusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
