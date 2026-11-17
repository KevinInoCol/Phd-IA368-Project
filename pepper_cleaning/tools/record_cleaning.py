"""Graba una corrida de limpieza y dibuja el resultado.

    python3 tools/record_cleaning.py <dir_salida> [--timeout 1800]

Mientras corre guarda en <dir>/run.jsonl, una línea por segundo: t (s simulados), pose de Pepper
(ground truth, frame map) y la que cree Pepper (TF map → base_footprint, AMCL), % limpio total y por
zona, batería, estado de cleaner y de autodock.
Termina cuando cleaner pasa a DONE y Pepper ya está cargando en la base (o el docking falló), si
la limpieza se cancela (IDLE), o al agotar el timeout. Al terminar escribe <dir>/summary.json y <dir>/cleaning.png (mapa con
baldosas limpias/sucias, trayectoria y la base; % limpio y batería en el tiempo).
"""

import argparse
import json
import math
import os
import sys
import time

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

OFFSET = (1.5, -1.0)


class Recorder(Node):
    def __init__(self, out):
        super().__init__("cleaning_recorder")
        self.out = out
        self.f = open(os.path.join(out, "run.jsonl"), "w")
        self.t = None
        self.pose = None
        self.dirt = {}
        self.battery = None
        self.clean = {}
        self.dock = {}
        self.grid = None
        self.seen_cleaning = False
        self.warned_posture = False
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Clock, "/clock", self._clock_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/pepper/odom_groundtruth", self._gt, 10)
        self.create_subscription(String, "/pepper/dirt/status", lambda m: setattr(self, "dirt", json.loads(m.data)), latched)
        self.create_subscription(OccupancyGrid, "/pepper/dirt/grid", lambda m: setattr(self, "grid", m), latched)
        self.create_subscription(BatteryState, "/pepper/battery", lambda m: setattr(self, "battery", m.percentage * 100), 10)
        self.create_subscription(String, "/pepper/clean_status", self._clean, 10)
        self.create_subscription(String, "/pepper/dock_status", lambda m: setattr(self, "dock", json.loads(m.data)), 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(1.0, self.tick)
        self.finished = False

    def _clock_cb(self, m):
        self.t = m.clock.sec + m.clock.nanosec * 1e-9

    def _gt(self, m):
        p = m.pose.pose
        q = p.orientation
        self.pose = (p.position.x + OFFSET[0], p.position.y + OFFSET[1],
                     math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def _clean(self, m):
        self.clean = json.loads(m.data)
        if self.clean.get("state") == "CLEANING":
            self.seen_cleaning = True

    def tick(self):
        if self.t is None or self.pose is None:
            return
        try:   # lo que Pepper cree (AMCL), para medir el error de localización
            tf = self.tf_buffer.lookup_transform("map", "base_footprint", rclpy.time.Time())
            ax, ay = round(tf.transform.translation.x, 3), round(tf.transform.translation.y, 3)
        except Exception:
            ax = ay = None
        try:   # postura: altura de la cámara de la cabeza sobre la base (recto ≈ 1.15 m)
            hz = round(self.tf_buffer.lookup_transform("base_footprint", "CameraTop_optical_frame",
                                                       rclpy.time.Time()).transform.translation.z, 3)
        except Exception:
            hz = None
        if hz is not None and hz < 0.9 and not self.warned_posture:
            self.warned_posture = True
            self.get_logger().warn(f"Pepper se dobló: cabeza a {hz:.2f} m (t = {self.t:.0f} s)")
        self.f.write(json.dumps({
            "head_z": hz,
            "t": round(self.t, 2), "x": round(self.pose[0], 3), "y": round(self.pose[1], 3),
            "ax": ax, "ay": ay,
            "percent": self.dirt.get("percent"),
            "zones": {k: v["percent"] for k, v in self.dirt.get("zones", {}).items()},
            "battery": None if self.battery is None else round(self.battery, 1),
            "clean_state": self.clean.get("state"), "zone": self.clean.get("zone"),
            "dock_state": self.dock.get("state")}) + "\n")
        self.f.flush()
        st = self.clean.get("state")
        ds = self.dock.get("state")
        if self.seen_cleaning and ((st == "DONE" and ds in ("CHARGING", "FAILED"))
                                   or (st == "IDLE" and ds in ("IDLE", "CHARGING", "FAILED"))):
            self.finished = True


def plot(out, grid):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np
    import yaml
    from ament_index_python.packages import get_package_share_directory
    from PIL import Image

    rows = [json.loads(l) for l in open(os.path.join(out, "run.jsonl"))]
    rows = [r for r in rows if r["percent"] is not None]
    if not rows:
        return {}
    t0 = rows[0]["t"]
    share = get_package_share_directory("pepper_museum")
    m = yaml.safe_load(open(os.path.join(share, "maps", "museum_nav.yaml")))
    img = np.array(Image.open(os.path.join(share, "maps", m["image"])))
    h, w = img.shape
    r, (ox, oy) = m["resolution"], m["origin"][:2]
    fig, (ax, bx) = plt.subplots(1, 2, figsize=(16, 8.5), gridspec_kw={"width_ratios": [1.15, 1]})
    ax.imshow(img, cmap="gray", extent=[ox, ox + w * r, oy, oy + h * r], vmin=0, vmax=255)
    if grid is not None:
        g = np.array(grid.data, dtype=float).reshape(grid.info.height, grid.info.width)
        rgba = np.zeros(g.shape + (4,))
        rgba[g == 100] = (0.45, 0.30, 0.15, 0.85)      # sucio
        rgba[g == 0] = (0.2, 0.75, 0.3, 0.55)          # limpio
        s = grid.info.resolution
        x0, y0 = grid.info.origin.position.x, grid.info.origin.position.y
        ax.imshow(rgba, origin="lower", extent=[x0, x0 + g.shape[1] * s, y0, y0 + g.shape[0] * s],
                  interpolation="nearest")
    xs, ys = [r_["x"] for r_ in rows], [r_["y"] for r_ in rows]
    ax.plot(xs, ys, "-", color="#1f6feb", lw=0.9, alpha=0.8, label="trayectoria de Pepper")
    with open(os.path.join(get_package_share_directory("pepper_cleaning"), "config", "dock.yaml")) as f:
        cx, cy = yaml.safe_load(f)["dock"]["contact"]
    ax.plot(cx, cy, "s", color="#0b5cad", ms=9, label="base de carga")
    ax.set_xlim(-6, 11)
    ax.set_ylim(-12.5, 7.5)
    ax.set_aspect("equal")
    ax.legend(loc="upper right", fontsize=8)
    ax.set_title("Piso: verde = limpiado, marrón = sigue sucio")
    ts = [(r_["t"] - t0) / 60 for r_ in rows]
    bx.plot(ts, [r_["percent"] for r_ in rows], "-", color="#2da44e", lw=2, label="% limpio (total)")
    bx.plot(ts, [r_["battery"] for r_ in rows], "-", color="#bf8700", lw=1.5, label="batería %")
    charging = [r_["dock_state"] == "CHARGING" for r_ in rows]
    for i in range(1, len(rows)):
        if charging[i]:
            bx.axvspan(ts[i - 1], ts[i], color="#bf8700", alpha=0.12, lw=0)
    bx.set_xlabel("tiempo simulado [min]")
    bx.set_ylim(0, 102)
    bx.grid(alpha=0.3)
    bx.legend(loc="lower right")
    bx.set_title("Limpieza y batería (sombreado = cargando)")
    fig.tight_layout()
    fig.savefig(os.path.join(out, "cleaning.png"), dpi=110)
    path = sum(math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1]) for i in range(1, len(xs)))
    # recargas en mitad de la limpieza (no cuenta la vuelta a la base al terminar)
    docks = sum(1 for i in range(1, len(rows))
                if charging[i] and not charging[i - 1] and rows[i]["clean_state"] == "PAUSED_DOCK")
    err = [math.hypot(r_["x"] - r_["ax"], r_["y"] - r_["ay"]) for r_ in rows if r_.get("ax") is not None]
    loc = {}
    if err:
        loc = {"amcl_rmse_m": round(math.sqrt(sum(e * e for e in err) / len(err)), 3),
               "amcl_p95_m": round(sorted(err)[int(0.95 * (len(err) - 1))], 3)}
        for z in rows[-1]["zones"]:
            ez = [math.hypot(r_["x"] - r_["ax"], r_["y"] - r_["ay"]) for r_ in rows
                  if r_.get("ax") is not None and r_["zone"] == z and r_["clean_state"] == "CLEANING"]
            if ez:
                loc.setdefault("amcl_rmse_by_zone_m", {})[z] = round(math.sqrt(sum(e * e for e in ez) / len(ez)), 3)
    hz = [r_["head_z"] for r_ in rows if r_.get("head_z") is not None]
    if hz:
        bent = [r_ for r_ in rows if r_.get("head_z") is not None and r_["head_z"] < 0.9]
        loc["head_z_min_m"] = round(min(hz), 3)
        loc["bent_from_min"] = round((bent[0]["t"] - t0) / 60, 1) if bent else None
    return {**loc, "duration_min": round(ts[-1], 1), "percent": rows[-1]["percent"], "zones": rows[-1]["zones"],
            "path_m": round(path, 1), "dock_cycles": docks,
            "battery_min": min(r_["battery"] for r_ in rows if r_["battery"] is not None)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--timeout", type=float, default=1800.0, help="s de pared")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    rclpy.init()
    node = Recorder(a.out)
    t_end = time.monotonic() + a.timeout
    try:
        while rclpy.ok() and not node.finished and time.monotonic() < t_end:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    node.f.close()
    summary = plot(a.out, node.grid)
    summary["finished"] = node.finished
    json.dump(summary, open(os.path.join(a.out, "summary.json"), "w"), indent=2, ensure_ascii=False)
    print(json.dumps(summary, ensure_ascii=False))
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
