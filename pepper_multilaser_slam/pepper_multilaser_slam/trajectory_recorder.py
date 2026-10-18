#!/usr/bin/env python3
"""Graba la trayectoria real y las estimadas en un CSV para la evaluación.

Por cada mensaje de /pepper/odom_groundtruth (submuestreado a `rate` Hz) guarda, en el mismo
instante:
  - gt_*   : pose real de base_footprint en el mundo de Gazebo
  - slam_* : pose estimada por el SLAM (TF map -> base_footprint)
  - odom_* : pose de la odometría ruidosa (TF odom -> base_footprint), la referencia sin SLAM
Con odometría degradada (odom_noise.py): odom_frame:=odom_noisy base_frame:=base_noisy.
El CSV se escribe línea a línea, así que matar el nodo no pierde datos.
"""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class TrajectoryRecorder(Node):
    def __init__(self):
        super().__init__("trajectory_recorder")
        self.output = self.declare_parameter("output", "trajectory.csv").value
        self.period = 1.0 / float(self.declare_parameter("rate", 5.0).value)
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.base_frame = self.declare_parameter("base_frame", "base_footprint").value
        self.last = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.file = open(self.output, "w")
        self.file.write("t,gt_x,gt_y,gt_yaw,slam_x,slam_y,slam_yaw,odom_x,odom_y,odom_yaw\n")
        self.create_subscription(Odometry, "/pepper/odom_groundtruth", self.gt_cb, 20)
        self.get_logger().info(f"trajectory_recorder -> {self.output}")

    def pose(self, frame, stamp):
        try:
            tf = self.tf_buffer.lookup_transform(frame, self.base_frame, stamp)
        except TransformException:
            return None
        t = tf.transform.translation
        return t.x, t.y, yaw_of(tf.transform.rotation)

    def gt_cb(self, msg):
        t = Time.from_msg(msg.header.stamp)
        secs = t.nanoseconds * 1e-9
        if self.last is not None and secs - self.last < self.period:
            return
        # Las TF pueden ir un poco por detrás del ground truth: usar la última disponible
        slam = self.pose("map", t) or self.pose("map", Time())
        odom = self.pose(self.odom_frame, t) or self.pose(self.odom_frame, Time())
        if slam is None or odom is None:
            return
        self.last = secs
        p = msg.pose.pose
        row = [secs, p.position.x, p.position.y, yaw_of(p.orientation), *slam, *odom]
        self.file.write(",".join(f"{v:.5f}" for v in row) + "\n")
        self.file.flush()

    def destroy_node(self):
        self.file.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = TrajectoryRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
