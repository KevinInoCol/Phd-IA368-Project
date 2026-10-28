#!/usr/bin/env python3
"""Recorrido de prueba del museo para comparar láseres (hokuyo de referencia vs. los 3 de Pepper).

    python3 tools/eval_navigation.py <out_dir>            # con visitors_demo.launch.py ya corriendo
    python3 tools/eval_navigation.py --summary <dir> [<dir> ...] > summary.md

Con la simulación, Nav2 + AMCL, place_navigator y visitor_approach (log_file = <out_dir>/calls.jsonl)
en marcha: recorre los 12 lugares de museum_places.yaml siempre en el mismo orden (/pepper/go_to) y,
al llegar a los de CALL_AT, hace una llamada de visitante (/pepper/visitor_call). Graba a 5 Hz la
pose real (/pepper/odom_groundtruth, mundo → map = mundo + (1.5, −1)) y la de AMCL (TF map →
base_footprint) en track.csv, y un JSON por lugar o llamada en events.jsonl.

--summary compara carpetas: lugares alcanzados, error al llegar (pose real vs. objetivo), error de
AMCL a lo largo del recorrido, distancia mínima a paredes (maps/museum_walls.txt, z = 0.2 m) y a
las personas, y en las llamadas la distancia final a la persona real más cercana y el error de
orientación hacia ella.
"""

import csv
import json
import math
import os
import sys
import threading
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..", "..")
sys.path.insert(0, os.path.join(HERE, ".."))

OFFSET = (1.5, -1.0)   # map = mundo + OFFSET
ORDER = ["mesa", "entrada", "cartel", "introduccion", "computadores", "tecnologia", "puertas_sur",
         "sala_oeste", "rotonda", "sala_norte", "escalera", "inicio"]
CALL_AT = ["mesa", "entrada", "introduccion", "tecnologia", "puertas_sur", "sala_oeste"]
WORLD = os.path.join(ROOT, "pepper_museum", "worlds", "museum_test_people.sdf")
PLACES = os.path.join(ROOT, "pepper_museum", "maps", "museum_places.yaml")
ROBOT_RADIUS = 0.24    # robot_radius de los costmaps de pepper_nav


def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


def run(out):
    import rclpy
    import yaml
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.time import Time
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformListener

    places = yaml.safe_load(open(PLACES))["places"]
    os.makedirs(out, exist_ok=True)
    rclpy.init()
    node = Node("eval_navigation", parameter_overrides=[
        rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)])
    tf = Buffer()
    TransformListener(tf, node)
    st = {"gt": None, "place": None, "visitor": None}
    lock = threading.Lock()

    def gt_cb(m):
        q = m.pose.pose.orientation
        with lock:
            st["gt"] = (m.pose.pose.position.x, m.pose.pose.position.y,
                        math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def status_cb(key):
        def cb(m):
            with lock:
                st[key] = json.loads(m.data)
        return cb

    node.create_subscription(Odometry, "/pepper/odom_groundtruth", gt_cb, 10)
    node.create_subscription(String, "/pepper/place_status", status_cb("place"), 10)
    node.create_subscription(String, "/pepper/visitor_status", status_cb("visitor"), 10)
    pub_go = node.create_publisher(String, "/pepper/go_to", 10)
    pub_call = node.create_publisher(String, "/pepper/visitor_call", 10)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    def now():
        return node.get_clock().now().nanoseconds * 1e-9

    def poses():
        """(real en map, estimada por AMCL en map) o None."""
        with lock:
            g = st["gt"]
        if g is None:
            return None
        try:
            t = tf.lookup_transform("map", "base_footprint", Time())
        except Exception:
            return None
        q = t.transform.rotation
        est = (t.transform.translation.x, t.transform.translation.y,
               math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        return (g[0] + OFFSET[0], g[1] + OFFSET[1], g[2]), est

    stop = threading.Event()

    def record():
        with open(os.path.join(out, "track.csv"), "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "x", "y", "yaw", "est_x", "est_y", "est_yaw"])
            while not stop.is_set():
                p = poses()
                if p:
                    w.writerow([round(now(), 2), *[round(v, 4) for v in p[0] + p[1]]])
                    f.flush()
                time.sleep(0.2)

    while poses() is None or pub_go.get_subscription_count() == 0 or pub_call.get_subscription_count() == 0:
        print("esperando TF map, place_navigator y visitor_approach…", flush=True)
        time.sleep(2.0)
    time.sleep(10.0)   # AMCL converge con la pose inicial del launch
    threading.Thread(target=record, daemon=True).start()
    events = open(os.path.join(out, "events.jsonl"), "w")

    def log(ev):
        p = poses()
        if p:
            ev["real"], ev["amcl"] = [round(v, 3) for v in p[0]], [round(v, 3) for v in p[1]]
        events.write(json.dumps(ev, ensure_ascii=False) + "\n")
        events.flush()
        print(json.dumps(ev, ensure_ascii=False), flush=True)

    def wait_status(key, done, timeout):
        t0 = now()
        while now() - t0 < timeout:
            with lock:
                s = st[key]
            if s and done(s):
                return s
            time.sleep(0.2)
        return None

    for key in ORDER:
        with lock:
            st["place"] = None
        t0 = now()
        pub_go.publish(String(data=json.dumps({"place": key})))
        s = wait_status("place", lambda s: s.get("place") == key and s.get("phase") in ("arrived", "failed"), 300)
        time.sleep(1.0)
        log({"kind": "place", "place": key, "goal": places[key]["pose"],
             "result": s["phase"] if s else "timeout", "t": round(now() - t0, 1)})
        if key not in CALL_AT:
            continue
        with lock:
            st["visitor"] = None
        t0 = now()
        pub_call.publish(String(data="hola pepper"))
        s = wait_status("visitor", lambda s: s.get("phase") in ("reached", "nobody", "nav_failed", "error"), 300)
        time.sleep(1.0)
        log({"kind": "call", "at": key, "result": s["phase"] if s else "timeout",
             "text": s.get("text") if s else "", "t": round(now() - t0, 1)})
    stop.set()
    time.sleep(0.5)
    events.close()
    rclpy.try_shutdown()


# ---------------------------------------------------------------- resumen
def wall_cells():
    path = os.path.join(ROOT, "pepper_museum", "maps", "museum_walls.txt")
    lines = open(path).read().split("\n")
    res = float(lines[0].split()[2])
    c = np.array([list(map(int, l.split())) for l in lines[1:] if l.strip()], float)
    return (c + 0.5) * res + np.array(OFFSET)   # centros de celda, en map


def min_dist(points, cells, chunk=500):
    out = []
    for i in range(0, len(points), chunk):
        d = np.linalg.norm(points[i:i + chunk, None, :] - cells[None, :, :], axis=2)
        out.append(d.min(1))
    return np.concatenate(out)


def summarize(dirs):
    from pepper_visitors.museum import people
    cells = wall_cells()
    persons = np.array([(x + OFFSET[0], y + OFFSET[1]) for _, _, x, y, _ in people(WORLD)])
    rows, call_rows = [], []
    for d in dirs:
        name = os.path.basename(os.path.normpath(d))
        tr = np.array([[float(v) for v in r] for r in list(csv.reader(open(os.path.join(d, "track.csv"))))[1:]])
        ev = [json.loads(l) for l in open(os.path.join(d, "events.jsonl"))]
        pl = [e for e in ev if e["kind"] == "place"]
        calls = [e for e in ev if e["kind"] == "call"]
        loc = np.hypot(tr[:, 1] - tr[:, 4], tr[:, 2] - tr[:, 5])
        yaw = np.abs(wrap_deg(np.degrees(tr[:, 3] - tr[:, 6])))
        wall = min_dist(tr[:, 1:3], cells)
        pers = min_dist(tr[:, 1:3], persons)
        arrived = [e for e in pl if e["result"] == "arrived"]
        arr_err = [math.dist(e["real"][:2], e["goal"][:2]) for e in arrived]
        dist = np.hypot(np.diff(tr[:, 1]), np.diff(tr[:, 2])).sum()
        rows.append({
            "name": name, "ok": f"{len(arrived)} / {len(pl)}",
            "fails": ", ".join(f"{e['place']} ({e['result']})" for e in pl if e["result"] != "arrived") or "—",
            "arr": np.mean(arr_err) if arr_err else float("nan"), "arr_max": max(arr_err) if arr_err else float("nan"),
            "loc_rmse": math.sqrt(np.mean(loc ** 2)), "loc_p95": np.percentile(loc, 95), "loc_max": loc.max(),
            "yaw_rmse": math.sqrt(np.mean(yaw ** 2)),
            "t": np.mean([e["t"] for e in arrived]) if arrived else float("nan"),
            "dist": dist, "dur": tr[-1, 0] - tr[0, 0],
            "wall": wall.min(), "touch": (wall < ROBOT_RADIUS).sum() * 0.2, "pers": pers.min()})
        for c in calls:
            x, y, yaw_r = c["real"]
            dp = np.hypot(persons[:, 0] - x, persons[:, 1] - y)
            k = int(dp.argmin())
            face = wrap_deg(math.degrees(math.atan2(persons[k, 1] - y, persons[k, 0] - x) - yaw_r))
            call_rows.append((name, c["at"], c["result"], dp[k], face, c["t"]))

    f = lambda v, n=2: "—" if v != v else f"{v:.{n}f}"   # noqa: E731
    print("## Recorrido por los 12 lugares\n")
    print("| Config. | Lugares alcanzados | Fallos | Error al llegar [m] (máx.) | Error AMCL RMSE [m] | "
          "AMCL p95 / máx. [m] | Yaw AMCL RMSE [°] | Tiempo por lugar [s] | Recorrido [m] | "
          "Mín. a pared [m] | Tiempo < radio [s] | Mín. a persona [m] |")
    print("|---|---|---|---|---|---|---|---|---|---|---|---|")
    for r in rows:
        print(f"| {r['name']} | {r['ok']} | {r['fails']} | {f(r['arr'])} ({f(r['arr_max'])}) | {f(r['loc_rmse'])} | "
              f"{f(r['loc_p95'])} / {f(r['loc_max'])} | {f(r['yaw_rmse'], 1)} | {f(r['t'], 0)} | {f(r['dist'], 0)} | "
              f"{f(r['wall'])} | {f(r['touch'], 1)} | {f(r['pers'])} |")
    print("\n## Llamadas de visitante\n")
    print("| Config. | Desde | Resultado | Dist. final a la persona real [m] | Error de orientación [°] | Tiempo [s] |")
    print("|---|---|---|---|---|---|")
    for n, at, res, dp, face, t in call_rows:
        print(f"| {n} | {at} | {res} | {dp:.2f} | {face:+.0f} | {t:.0f} |")


if __name__ == "__main__":
    if len(sys.argv) > 2 and sys.argv[1] == "--summary":
        summarize(sys.argv[2:])
    elif len(sys.argv) == 2:
        run(sys.argv[1])
    else:
        print(__doc__)
