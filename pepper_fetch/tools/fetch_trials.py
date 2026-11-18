"""Prueba repetida: Pepper sale de distintos sitios del museo y coge la manzana, puesta cada vez en
otro lugar del borde de la mesa.

    python3 tools/fetch_trials.py <dir_salida> [--n 10] [--seed 1]

Con fetch_demo.launch.py en marcha. En cada intento: suelta lo que tenga ("soltar"), pone la
manzana en una posición al azar del tablero (a 4–9 cm del borde por el que llega Pepper, ±15 cm a
lo ancho, girada al azar), lleva a Pepper con Nav2 a uno de los puntos de salida, pide
"tráeme una manzana" en /pepper/fetch y espera el resultado. Mide con el ground truth de Gazebo:
  - error de la posición de la manzana que calcula Pepper (cámara + profundidad) al ir a cogerla,
  - si la manzana acaba en la mano: fetch_object dice "holding" y la manzana está más de 4 cm por
    encima del tablero (una que se cae queda en la mesa o en el suelo); la distancia a la muñeca se
    guarda como dato (16–21 cm según la pose: no sirve de criterio),
  - el tiempo desde el pedido.
Escribe <dir>/trials.jsonl y <dir>/summary.json.
"""

import argparse
import json
import math
import os
import random
import statistics
import sys
import threading
import time

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

HERE = os.path.dirname(os.path.abspath(__file__))
STARTS = [(0.0, 0.0, 0), (2.5, 1.0, 180), (-1.5, 2.5, -45), (3.0, 4.5, -120), (0.5, 4.5, -90)]   # map
END = ("holding", "not_found", "unreachable", "nav_failed", "error")


def success(rec, table_h):
    a = rec.get("apple_world_after")
    return bool(rec["phase"] == "holding" and a and a[2] > table_h + 0.04)


def summarize(recs):
    ok = [r for r in recs if r["success"]]
    ve = [r["vision_err_cm"] for r in recs if r.get("vision_err_cm") is not None]
    return {"trials": len(recs), "success": len(ok),
            "time_s_mean": round(statistics.mean(r["time_s"] for r in ok), 1) if ok else None,
            "time_s_range": [min(r["time_s"] for r in ok), max(r["time_s"] for r in ok)] if ok else None,
            "vision_err_cm_mean": round(statistics.mean(ve), 1) if ve else None,
            "vision_err_cm_max": max(ve) if ve else None,
            "results": [r["phase"] for r in recs]}


class Trials(Node):
    def __init__(self, out, n, seed):
        super().__init__("fetch_trials")
        with open(os.path.join(HERE, "..", "config", "fetch.yaml")) as f:
            self.cfg = yaml.safe_load(f)
        self.out, self.n, self.rng = out, n, random.Random(seed)
        self.gz = GzNode()
        self.poses = {}
        self.gz.subscribe(Pose_V, "/world/default/pose/info", self._poses_cb)
        self.gt = None
        self.status = []
        self.create_subscription(Odometry, "/pepper/odom_groundtruth", lambda m: setattr(self, "gt", m), 10)
        self.create_subscription(String, "/pepper/fetch_status", lambda m: self.status.append(json.loads(m.data)), 10)
        self.pub = self.create_publisher(String, "/pepper/fetch", 10)
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.tf = Buffer()
        TransformListener(self.tf, self)

    def _poses_cb(self, msg):
        for p in msg.pose:
            if p.name in ("apple", "pepper_MP"):
                self.poses[p.name] = (p.position.x, p.position.y, p.position.z)

    def spin_for(self, sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_status(self, phases, timeout):
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            for s in self.status:
                if s["phase"] in phases:
                    return s
        return None

    def send(self, text):
        self.status = []
        self.pub.publish(String(data=text))

    def release(self):
        """"soltar" hasta que fetch_object lo acepte (si está ocupado contesta "busy")."""
        for _ in range(30):
            self.send("soltar")
            s = self.wait_status(("released", "busy"), 90)
            if s and s["phase"] == "released":
                return True
            self.spin_for(3.0)
        return False

    def place_apple(self):
        t = self.cfg["table"]
        ox, oy = self.cfg["map_offset"]
        cx, cy = t["center"]
        d = self.rng.uniform(0.04, 0.09)
        lx, ly = self.rng.uniform(-0.15, 0.15), -t["size"][1] / 2 + d
        yaw = self.rng.uniform(-math.pi, math.pi)
        # borrar y volver a crear: set_pose conserva la velocidad que tuviera la manzana
        e = Entity()
        e.name, e.type = "apple", Entity.MODEL
        self.gz.request("/world/default/remove", e, Entity, Boolean, 3000)
        self.spin_for(2.0)
        ef = EntityFactory()
        ef.sdf_filename = os.path.abspath(os.path.join(HERE, "..", "models", "apple", "model.sdf"))
        ef.name = "apple"
        ef.pose.position.x, ef.pose.position.y, ef.pose.position.z = cx + lx - ox, cy + ly - oy, t["height"] + 0.003
        ef.pose.orientation.z, ef.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.gz.request("/world/default/create", ef, EntityFactory, Boolean, 5000)
        self.spin_for(2.0)
        return [round(lx, 3), round(ly, 3)], round(d, 3)

    def go(self, x, y, yaw_deg):
        g = NavigateToPose.Goal()
        g.pose = PoseStamped()
        g.pose.header.frame_id = "map"
        g.pose.pose.position.x, g.pose.pose.position.y = float(x), float(y)
        a = math.radians(yaw_deg)
        g.pose.pose.orientation.z, g.pose.pose.orientation.w = math.sin(a / 2), math.cos(a / 2)
        self.nav.wait_for_server()
        f = self.nav.send_goal_async(g)
        rclpy.spin_until_future_complete(self, f)
        r = f.result().get_result_async()
        rclpy.spin_until_future_complete(self, r, timeout_sec=180)
        return r.done() and r.result().status == GoalStatus.STATUS_SUCCEEDED

    def apple_in_base(self):
        """Centro de la manzana en base_footprint según el ground truth (mundo = odom_groundtruth)."""
        a, m = self.poses.get("apple"), self.gt
        if a is None or m is None:
            return None
        p, q = m.pose.pose.position, m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        dx, dy = a[0] - p.x, a[1] - p.y
        return [dx * math.cos(yaw) + dy * math.sin(yaw), -dx * math.sin(yaw) + dy * math.cos(yaw),
                a[2] + 0.037 - p.z]

    def wrist_dist(self):
        try:
            t = self.tf.lookup_transform("base_footprint", "r_wrist", rclpy.time.Time()).transform.translation
        except Exception:
            return None
        a = self.apple_in_base()
        return None if a is None else math.dist(a, (t.x, t.y, t.z))

    def run(self):
        f = open(os.path.join(self.out, "trials.jsonl"), "w")
        recs = []
        for i in range(self.n):
            self.release()
            start = STARTS[i % len(STARTS)]
            self.get_logger().info(f"intento {i + 1}: a la salida {start}")
            if not self.go(*start):
                self.get_logger().warn("Nav2 no llegó a la salida; sigo desde aquí")
            at, d = self.place_apple()
            rec = {"n": i + 1, "start": start, "apple_local": at, "apple_from_edge": d}
            t0 = time.time()
            self.send("tráeme una manzana")
            s = self.wait_status(("reaching",) + END, 300)
            if s and s["phase"] == "reaching":
                truth = self.apple_in_base()
                est = s.get("object_base")
                rec["object_base_est"] = est
                rec["object_base_gt"] = [round(v, 3) for v in truth] if truth else None
                if est and truth:
                    rec["vision_err_cm"] = round(100 * math.dist(est, truth), 1)
                    rec["vision_err_xy_cm"] = round(100 * math.dist(est[:2], truth[:2]), 1)
                s = self.wait_status(END, 200)
            rec["phase"] = s["phase"] if s else "timeout"
            rec["time_s"] = round(time.time() - t0, 1)
            self.spin_for(1.0)
            a = self.poses.get("apple")
            rec["apple_world_after"] = [round(v, 3) for v in a] if a else None
            wd = self.wrist_dist()
            rec["apple_wrist_cm"] = round(100 * wd, 1) if wd is not None else None
            rec["success"] = success(rec, self.cfg["table"]["height"])
            self.get_logger().info(f"intento {i + 1}: {rec['phase']} en {rec['time_s']} s, "
                                   f"error de visión {rec.get('vision_err_cm')} cm, éxito {rec['success']}")
            f.write(json.dumps(rec, ensure_ascii=False) + "\n")
            f.flush()
            recs.append(rec)
        self.release()
        summary = summarize(recs)
        with open(os.path.join(self.out, "summary.json"), "w") as g:
            json.dump(summary, g, indent=1, ensure_ascii=False)
        print(json.dumps(summary, ensure_ascii=False))


def rescore(out):
    """Vuelve a calcular éxito y resumen de <out>/trials.jsonl con el criterio actual."""
    with open(os.path.join(HERE, "..", "config", "fetch.yaml")) as f:
        h = yaml.safe_load(f)["table"]["height"]
    recs = [json.loads(line) for line in open(os.path.join(out, "trials.jsonl"))]
    for r in recs:
        r["success"] = success(r, h)
    with open(os.path.join(out, "trials.jsonl"), "w") as f:
        f.writelines(json.dumps(r, ensure_ascii=False) + "\n" for r in recs)
    with open(os.path.join(out, "summary.json"), "w") as f:
        json.dump(summarize(recs), f, indent=1, ensure_ascii=False)
    print(json.dumps(summarize(recs), ensure_ascii=False))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--rescore", action="store_true", help="sólo recalcular éxito y resumen")
    ap.add_argument("--n", type=int, default=10)
    ap.add_argument("--seed", type=int, default=1)
    a = ap.parse_args()
    if a.rescore:
        return rescore(a.out)
    os.makedirs(a.out, exist_ok=True)
    rclpy.init()
    node = Trials(a.out, a.n, a.seed)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
