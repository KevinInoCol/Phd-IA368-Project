"""Captura el dataset sintético de visitantes: teletransporta a Pepper por el museo y guarda lo
que ve su cámara frontal junto con su pose real.

    ros2 run pepper_visitors capture_dataset --world museum_with_people --out <dir> --n 900
    ros2 run pepper_visitors capture_dataset --world museum_with_people_moving --out <dir> --n 400

Con personas estáticas, cada captura apunta a una persona elegida por turnos entre las clases
(male_03 tiene 15 copias y las otras 4: así las clases quedan parejas), desde una distancia de
1–6 m y un ángulo al azar, con el rumbo desviado hasta ±17° para que no salga siempre centrada.
En el mundo con actores apunta a un punto al azar de sus trayectorias (clase "otro").
Las poses descartan paredes cercanas, personas a menos de 0.6 m y vistas tapadas por un muro.

Escribe <dir>/images/*.jpg y <dir>/meta.jsonl (imagen, pose pedida, pose de
/pepper/odom_groundtruth al tomar la imagen). Las etiquetas las pone tools/label_dataset.py.
Requiere la simulación del museo corriendo con ese mismo mundo.
"""

import argparse
import json
import math
import os
import subprocess

import cv2
import rclpy
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image

from pepper_visitors import museum

SETTLE = 0.4  # s simulados entre el teletransporte y la imagen que se guarda


class Capture(Node):
    def __init__(self):
        super().__init__("capture_dataset")
        self.bridge = CvBridge()
        self.img = self.gt = None
        self.now = 0.0
        self.create_subscription(Image, "/pepper/camera/front/image_raw", self.img_cb, 1)
        self.create_subscription(Odometry, "/pepper/odom_groundtruth", self.gt_cb, 1)
        self.create_subscription(Clock, "/clock", self.clock_cb, qos_profile_sensor_data)

    def img_cb(self, m):
        self.img = m

    def gt_cb(self, m):
        self.gt = m

    def clock_cb(self, m):
        self.now = m.clock.sec + m.clock.nanosec * 1e-9

    def teleport(self, x, y, yaw):
        req = (f'name: "pepper_MP", position: {{x: {x}, y: {y}, z: 0.006}}, '
               f'orientation: {{z: {math.sin(yaw / 2)}, w: {math.cos(yaw / 2)}}}')
        subprocess.run(["gz", "service", "-s", "/world/default/set_pose", "--reqtype", "gz.msgs.Pose",
                        "--reptype", "gz.msgs.Boolean", "--timeout", "2000", "--req", req],
                       capture_output=True, check=False)

    def wait_image(self):
        """Primera imagen tomada SETTLE s simulados después de ahora."""
        while rclpy.ok() and self.now == 0.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        t0 = self.now + SETTLE
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.img is not None:
                s = self.img.header.stamp
                if s.sec + s.nanosec * 1e-9 >= t0:
                    return self.img


def stamp_pose(odom):
    p, q = odom.pose.pose.position, odom.pose.pose.orientation
    return [p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))]


def sample_view(r, target, others, walls, dmin, dmax):
    """Pose (x, y, yaw) de Pepper mirando a `target` (x, y), o None."""
    for _ in range(300):
        d, th = r.uniform(dmin, dmax), r.uniform(0, 2 * math.pi)
        x, y = target[0] + d * math.cos(th), target[1] + d * math.sin(th)
        if not walls.clear(x, y, 0.35) or any(math.hypot(x - ox, y - oy) < 0.6 for ox, oy in others):
            continue
        if not walls.line_of_sight(x, y, *target):
            continue
        yaw = math.atan2(target[1] - y, target[0] - x) + r.uniform(-0.3, 0.3)
        return x, y, yaw
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default="museum_with_people")
    ap.add_argument("--out", required=True)
    ap.add_argument("--n", type=int, default=900)
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args(rclpy.utilities.remove_ros_args()[1:])

    sdf = museum.museum_share("worlds", args.world + ".sdf")
    persons = museum.people(sdf)
    paths = museum.actor_paths(sdf)
    walls = museum.Walls(museum.museum_share("maps", "museum_walls.txt"))
    others = [(p[2], p[3]) for p in persons]
    by_class = {}
    for p in persons:
        by_class.setdefault(p[1], []).append(p)
    classes = sorted(by_class)
    r = museum.rng(args.seed)

    os.makedirs(os.path.join(args.out, "images"), exist_ok=True)
    meta_path = os.path.join(args.out, "meta.jsonl")
    done = sum(1 for _ in open(meta_path)) if os.path.exists(meta_path) else 0

    rclpy.init()
    node = Capture()
    node.get_logger().info(f"{args.world}: {len(persons)} personas, {len(paths)} actores; "
                           f"{done} capturas previas, objetivo {args.n}")
    with open(meta_path, "a") as meta:
        k = done
        while k < args.n and rclpy.ok():
            if paths:
                path = paths[r.integers(len(paths))]
                i = r.integers(len(path) - 1) if len(path) > 1 else 0
                a, b = path[i], path[min(i + 1, len(path) - 1)]
                t = r.uniform()
                target, tag = (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])), "actores"
                view = sample_view(r, target, others, walls, 1.5, 5.0)
            else:
                p = by_class[classes[k % len(classes)]][r.integers(len(by_class[classes[k % len(classes)]]))]
                target, tag = (p[2], p[3]), p[0]
                view = sample_view(r, target, [o for o in others if o != target], walls, 1.0, 6.0)
            if view is None:
                continue
            node.teleport(*view)
            img = node.wait_image()
            if img is None:
                break
            rclpy.spin_once(node, timeout_sec=0.05)
            name = f"{args.world}_{k:05d}.jpg"
            cv2.imwrite(os.path.join(args.out, "images", name),
                        node.bridge.imgmsg_to_cv2(img, "bgr8"), [cv2.IMWRITE_JPEG_QUALITY, 95])
            meta.write(json.dumps({"image": name, "world": args.world, "target": tag,
                                   "set_pose": [round(v, 4) for v in view],
                                   "gt_pose": [round(v, 4) for v in stamp_pose(node.gt)]}) + "\n")
            meta.flush()
            k += 1
            if k % 50 == 0:
                node.get_logger().info(f"{k}/{args.n}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
