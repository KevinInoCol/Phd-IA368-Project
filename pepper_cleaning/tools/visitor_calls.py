"""Prueba: visitantes llaman a Pepper mientras limpia el museo.

    python3 tools/visitor_calls.py <dir_salida> [--plan chat,silent,charging,chat] [--first 120] [--gap 90]
                                   [--zones sala_oeste,sala_norte,entrada]

Espera a que cleaner esté limpiando y, cada `gap` s simulados tras retomar la limpieza, publica una
llamada en /pepper/visitor_call (lo que haría voice_listener o la app al oír "Pepper, ven"). Cada
llamada del plan es de un tipo:
  chat      tras llegar, una charla corta: tres frases de Pepper en /pepper/say (como las
            respuestas de Gemini en el bridge) y luego /pepper/visitor_done (el visitante se despide)
  silent    nadie habla: visitor_approach cierra la charla solo, a los talk_timeout s
  charging      lo llama mientras carga en la base con ≥ 55 % (debe salir de la base y atender)
  charging_low  lo llama al empezar a cargar, con < 35 % (no atiende: pide que vayan a la base)

Por llamada guarda en <dir>/calls.jsonl: estado de limpieza, batería y base al llamar; cuándo pausó
la limpieza; las fases de /pepper/visitor_status; la persona elegida y a qué distancia quedó Pepper
de ella (ground truth); cuándo retomó la limpieza (zona y tramo) y si el cepillo estuvo apagado
mientras atendía. Al terminar escribe <dir>/calls_summary.json.
"""

import argparse
import json
import math
import os
import sys

import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool, String

OFFSET = (1.5, -1.0)
PHRASES = ["Claro, en esta sala están los primeros computadores.",
           "Me alegra que te guste el museo.",
           "¡Hasta luego, que disfrutes la visita!"]
END_PHASES = ("done", "nobody", "nav_failed", "charging", "error")


class Calls(Node):
    def __init__(self, out, plan, first, gap, timeout, zones=None):
        super().__init__("visitor_calls_test")
        self.zones = zones       # sólo llamar mientras limpia estas zonas (None: cualquiera)
        cfg = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "config", "dirt.yaml")
        with open(cfg) as f:
            self.rects = {k: z["rect"] for k, z in yaml.safe_load(f)["zones"].items()}
        self.out, self.plan, self.first, self.gap, self.timeout = out, plan, first, gap, timeout
        self.t = None
        self.gt = None
        self.clean = {}
        self.dock = {}
        self.brush = None
        self.k = 0               # llamada actual
        self.call = None         # registro de la llamada en curso
        self.t_next = None       # cuándo llamar (s simulados)
        self.t_chat = None       # siguiente frase de la charla simulada
        self.chat_i = 0
        self.records = []
        self.finished = False
        self.create_subscription(Clock, "/clock", self._clock_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/pepper/odom_groundtruth", self._gt_cb, 10)
        self.create_subscription(String, "/pepper/clean_status", self._clean_cb, 10)
        self.create_subscription(String, "/pepper/dock_status", lambda m: setattr(self, "dock", json.loads(m.data)), 10)
        self.create_subscription(String, "/pepper/visitor_status", self._visitor_cb, 10)
        self.create_subscription(Bool, "/pepper/brush", self._brush_cb, 10)
        self.pub_call = self.create_publisher(String, "/pepper/visitor_call", 10)
        self.pub_say = self.create_publisher(String, "/pepper/say", 10)
        self.pub_done = self.create_publisher(String, "/pepper/visitor_done", 10)
        self.f = open(os.path.join(out, "calls.jsonl"), "w")
        self.create_timer(0.5, self.tick)

    def _clock_cb(self, m):
        self.t = m.clock.sec + m.clock.nanosec * 1e-9

    def _gt_cb(self, m):
        p = m.pose.pose.position
        self.gt = (p.x + OFFSET[0], p.y + OFFSET[1])

    def _brush_cb(self, m):
        self.brush = m.data
        c = self.call
        if c and m.data and self.t is not None and c.get("t_paused") and not c.get("t_resumed"):
            # al retomar, cleaner enciende el cepillo justo cuando llega el fin de la llamada:
            # sólo cuenta si se enciende antes (se decide al cerrar la llamada)
            c.setdefault("brush_on_t", []).append(self.t)

    def _clean_cb(self, m):
        self.clean = json.loads(m.data)
        c = self.call
        if c is None or self.t is None:
            return
        st = self.clean.get("state")
        if st == "PAUSED_VISITOR" and "t_paused" not in c:
            c["t_paused"] = self.t
            c["paused_at"] = [self.clean.get("zone"), self.clean.get("segment")]
        if st == "CLEANING" and "t_end" in c and "t_resumed" not in c:
            c["t_resumed"] = self.t
            c["resumed_at"] = [self.clean.get("zone"), self.clean.get("segment")]

    def _visitor_cb(self, m):
        st = json.loads(m.data)
        c = self.call
        if c is None or self.t is None:
            return
        ph = st.get("phase")
        c["phases"].append([round(self.t - c["t_call"], 1), ph, st.get("text", "")])
        self.get_logger().info(f"llamada {c['n']}: {ph} — {st.get('text', '')}")
        if ph == "approaching":
            c["target"] = st.get("target")
        if ph == "reached":
            c["t_reached"] = self.t
            c["reported_dist"] = st.get("dist")
            if self.gt and c.get("target"):
                c["gt_dist"] = round(math.dist(self.gt, c["target"]), 2)
            if c["kind"] == "chat":
                self.t_chat, self.chat_i = self.t + 6.0, 0
        if ph in END_PHASES and "t_end" not in c:
            c["t_end"] = self.t
            c["result"] = ph if ph != "done" else "reached_and_talked"
            if self.clean.get("state") == "CLEANING" and "t_resumed" not in c:
                c["t_resumed"] = self.t

    def in_zone(self, zones):
        """¿Pepper (ground truth) está dentro de la zona que limpia y esa zona es de la lista? (no
        basta la zona de cleaner: también lo es mientras va de camino a ella)"""
        z = self.clean.get("zone")
        if z not in zones or self.gt is None:
            return False
        x0, x1, y0, y1 = self.rects[z]
        return x0 <= self.gt[0] <= x1 and y0 <= self.gt[1] <= y1

    def tick(self):
        if self.t is None or self.finished:
            return
        st = self.clean.get("state")
        if self.call is None:
            if self.k >= len(self.plan):
                return self.finish()
            kind = self.plan[self.k]
            if self.t_next is None:
                if st == "CLEANING" or kind.startswith("charging"):
                    self.t_next = self.t + (self.first if self.k == 0 else self.gap)
                return
            b = self.dock.get("battery") or 0.0
            charging = self.dock.get("state") == "CHARGING"
            if kind == "charging":
                ready = charging and b >= 55.0
            elif kind == "charging_low":
                ready = charging and b < 35.0
            else:
                ready = st == "CLEANING" and self.t >= self.t_next and (
                    not self.zones or self.in_zone(self.zones))
            if not ready:
                if st == "DONE":
                    self.get_logger().warn("la limpieza terminó antes de hacer todas las llamadas")
                    return self.finish()
                return
            self.k += 1
            self.call = {"n": self.k, "kind": kind, "t_call": self.t, "clean_state": st,
                         "zone": self.clean.get("zone"), "dock_state": self.dock.get("state"),
                         "battery": self.dock.get("battery"), "gt_call": self.gt, "phases": []}
            self.get_logger().info(f"llamada {self.k} ({kind}) en {self.clean.get('zone')}, dock {self.dock.get('state')}")
            self.pub_call.publish(String(data="Pepper, ven por favor"))
            return
        c = self.call
        if self.t_chat is not None and self.t >= self.t_chat:
            if self.chat_i < len(PHRASES):
                self.pub_say.publish(String(data=PHRASES[self.chat_i]))
                self.chat_i += 1
                self.t_chat = self.t + 8.0
            else:
                self.pub_done.publish(String(data="El visitante terminó la charla."))
                self.t_chat = None
        resumed = "t_resumed" in c or (c.get("t_end") and st in ("DONE", "PAUSED_DOCK") )
        if resumed or self.t - c["t_call"] > self.timeout:
            if not resumed:
                c["result"] = c.get("result", "") + " timeout"
            t_end = c.get("t_end", self.t)
            c["brush_on_while_paused"] = any(t < t_end - 0.5 for t in c.pop("brush_on_t", []))
            for k in ("t_paused", "t_reached", "t_end", "t_resumed"):
                if k in c:
                    c[k.replace("t_", "dt_")] = round(c[k] - c["t_call"], 1)
            c["dock_after"] = self.dock.get("state")
            self.f.write(json.dumps(c, ensure_ascii=False) + "\n")
            self.f.flush()
            self.records.append(c)
            self.get_logger().info(f"llamada {c['n']}: {c.get('result')} — retomó a los {c.get('dt_resumed')} s")
            self.call, self.t_next, self.t_chat = None, None, None

    def finish(self):
        self.finished = True
        rs = self.records
        reached = [r for r in rs if r.get("t_reached")]
        resumed = [r for r in rs if "t_resumed" in r]
        summary = {
            "calls": len(rs),
            "reached": len(reached),
            "paused_ok": sum(1 for r in rs if "t_paused" in r or r.get("clean_state") != "CLEANING"),
            "resumed": len(resumed),
            "brush_on_while_paused": sum(1 for r in rs if r.get("brush_on_while_paused")),
            "pause_latency_s": [r.get("dt_paused") for r in rs],
            "reach_time_s": [r.get("dt_reached") for r in rs],
            "gt_dist_m": [r.get("gt_dist") for r in rs],
            "results": [f"{r['kind']}: {r.get('result')}" for r in rs],
        }
        with open(os.path.join(self.out, "calls_summary.json"), "w") as f:
            json.dump(summary, f, indent=1, ensure_ascii=False)
        print(json.dumps(summary, ensure_ascii=False))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--plan", default="chat,charging_low,charging,silent,chat")
    ap.add_argument("--first", type=float, default=120.0)
    ap.add_argument("--gap", type=float, default=90.0)
    ap.add_argument("--timeout", type=float, default=400.0, help="s simulados por llamada")
    ap.add_argument("--zones", default="", help="zonas (coma) en las que llamar; vacío = todas")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    rclpy.init()
    node = Calls(a.out, a.plan.split(","), a.first, a.gap, a.timeout,
                 [z for z in a.zones.split(",") if z] or None)
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        node.finish()
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
