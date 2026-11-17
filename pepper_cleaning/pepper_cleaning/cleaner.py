"""Limpieza por cobertura de las zonas del museo, con pausa para recargar en la base.

Pepper no sabe dónde está la suciedad (eso es del simulador): limpia la zona entera como una
aspiradora, en zigzag (boustrophedon) sobre el mapa. Cada zona de config/dirt.yaml se recorre en
carriles paralelos al lado largo, separados `lane_spacing` m como mucho (< 2 × radio del cepillo,
para que se solapen), el primero y el último a `edge_inset` m del borde de la zona; cada carril se corta donde el mapa no deja pasar a Pepper (`clearance`), y cada tramo se
manda a Nav2 como NavigateThroughPoses. Justo antes de mandarlo, el tramo se vuelve a cortar con el
costmap global de Nav2 (lo que ven los láseres: personas, objetos que no están en el mapa; a las
personas se les deja `people_margin` m); si un
tramo falla a mitad, se reintenta una vez lo que faltaba desde donde quedó Pepper.

Repaso: mientras limpia, Pepper recuerda por dónde pasó el cepillo según su propia localización
(TF map → base_footprint, AMCL; nunca el ground truth). Al terminar los carriles de una zona
visita, en orden de vecino más cercano, los puntos libres de la zona (rejilla de `gap_step` m) que
no cubrió: bordes, tramos cortados por personas que ya se movieron, carriles abandonados. El cepillo (/pepper/brush) va encendido mientras limpia.

Si autodock empieza a ir a la base (batería baja, /pepper/dock_status deja IDLE), la limpieza se
pausa: cancela Nav2, apaga el cepillo y, cuando autodock vuelve a IDLE tras cargar, sigue en el
tramo donde se quedó. Al terminar dice cuánto limpió y, con `dock_when_done`, vuelve a la base.

Visitantes: si alguien llama a Pepper mientras limpia, visitor_approach (pepper_visitors) publica
`searching` en /pepper/visitor_status; la limpieza se pausa igual (PAUSED_VISITOR) y Pepper busca a
la persona con la cámara, se acerca y conversa. Cuando la charla termina (`done`), o si no encontró
a nadie (`nobody`), no pudo llegar (`nav_failed`), no podía atender por batería (`charging`) o
falló (`error`), sigue limpiando en el tramo donde se quedó. Si lo llaman mientras carga en la
base, visitor_approach lo saca de ella: al volver autodock a IDLE no retoma hasta que acabe la
charla (con poca batería autodock lo volverá a llevar a la base).

Órdenes en /pepper/clean_cmd (std_msgs/String):
    "start"              todas las zonas, la más cercana primero
    "start sala_oeste"   una zona (clave o nombre de config/dirt.yaml; también JSON {"zones": [...]})
    "stop"               cancela
Estado en /pepper/clean_status (JSON): {state, zone, zone_name, segment, segments, zones_done,
zones_left, percent, text}; state = IDLE | CLEANING | PAUSED_DOCK | PAUSED_VISITOR | DONE.
Plan en /pepper/clean_plan (nav_msgs/Path, latched) para RViz.
"""

import json
import math
import time
import unicodedata

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from nav2_msgs.msg import SpeedLimit
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener

from .common import FreeMap, default_map, load_dock

import os

COST_BLOCKED = 99   # /global_costmap/costmap: 100 letal, 99 inscrito (Pepper no cabe)
COST_LETHAL = 100


def norm(text):
    t = unicodedata.normalize("NFD", text.lower())
    return "".join(c for c in t if unicodedata.category(c) != "Mn").replace("_", " ").strip()


class Cleaner(Node):
    def __init__(self):
        super().__init__("cleaner")
        share = get_package_share_directory("pepper_cleaning")
        cfg = self.declare_parameter("config", os.path.join(share, "config", "dirt.yaml")).value
        self.fmap = FreeMap(self.declare_parameter("map", default_map()).value)
        self.spacing = self.declare_parameter("lane_spacing", 0.40).value
        self.step_len = self.declare_parameter("waypoint_step", 0.30).value
        self.clearance = self.declare_parameter("clearance", 0.30).value
        self.edge = self.declare_parameter("edge_inset", 0.12).value          # carriles de borde
        self.people_margin = self.declare_parameter("people_margin", 0.45).value
        self.speed = self.declare_parameter("speed_percent", 50.0).value
        self.dock_when_done = self.declare_parameter("dock_when_done", True).value
        self.gap_pass = self.declare_parameter("gap_pass", True).value
        self.gap_step = self.declare_parameter("gap_step", 0.30).value
        self.brush_r = self.declare_parameter("covered_radius", 0.22).value   # algo menos que el cepillo
        with open(cfg) as f:
            self.zones = yaml.safe_load(f)["zones"]
        self.offset = load_dock()["offset"]

        self.state = "IDLE"
        self.queue = []          # zonas por limpiar
        self.zone = None
        self.segments = []       # tramos de la zona actual: [[(x, y, yaw), ...], ...]
        self.seg = 0
        self.done_zones = []
        self.goal = None
        self.goal_seq = 0
        self.dock_state = "IDLE"
        self.dirt = {}
        self.pose = None
        self.text = ""
        self.t_start = None
        self.cm = None
        self.cm_info = None
        self.retries = 0
        self.mission = []
        self.pending = None      # zonas pedidas mientras cargaba (empieza al salir de la base)
        self.covered = set()     # celdas de 0.1 m por las que pasó el cepillo (según AMCL)
        self.in_gaps = False     # la zona actual está en el repaso
        self.sparse = True       # tramos de carril: waypoints espaciados; repaso: todos los puntos
        self.brush_on = False
        self.visitor = False     # visitor_approach atiende a alguien (searching .. done)
        self.visitors = 0        # llamadas atendidas en esta misión

        self.create_subscription(String, "/pepper/clean_cmd", self._cmd_cb, 10)
        self.create_subscription(String, "/pepper/dock_status", self._dock_cb, 10)
        self.create_subscription(String, "/pepper/visitor_status", self._visitor_cb, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/pepper/dirt/status", self._dirt_cb, latched)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(0.2, self.update_pose)
        self.create_subscription(OccupancyGrid, "/global_costmap/costmap", self._costmap_cb, latched)
        self.create_subscription(OccupancyGridUpdate, "/global_costmap/costmap_updates", self._costmap_update_cb, 10)
        self.pub_brush = self.create_publisher(Bool, "/pepper/brush", 10)
        self.pub_status = self.create_publisher(String, "/pepper/clean_status", 10)
        self.pub_say = self.create_publisher(String, "/pepper/say", 10)
        self.pub_dock = self.create_publisher(String, "/pepper/dock_cmd", 10)
        self.pub_go_to = self.create_publisher(String, "/pepper/go_to", 10)
        self.pub_speed = self.create_publisher(SpeedLimit, "/speed_limit", 10)
        self.pub_plan = self.create_publisher(Path, "/pepper/clean_plan", latched)
        self.nav = ActionClient(self, NavigateThroughPoses, "navigate_through_poses")
        self.create_timer(1.0, self.publish_status)
        self.create_timer(0.5, self.watchdog)
        n = {z: len(self.plan(z)) for z in self.zones}
        self.get_logger().info(f"cleaner: zonas {n} (tramos)")

    # ------------------------------------------------------------ plan
    def plan(self, zone):
        """Tramos en zigzag de la zona: lista de listas de (x, y, yaw) en el frame map."""
        x0, x1, y0, y1 = self.zones[zone]["rect"]
        along_x = (x1 - x0) >= (y1 - y0)
        # carriles: coordenada transversal; puntos: coordenada a lo largo
        c0, c1 = (y0, y1) if along_x else (x0, x1)
        a0, a1 = (x0, x1) if along_x else (y0, y1)
        # primer y último carril pegados al borde (`edge_inset`): si AMCL se corre ~0.2 m, el borde
        # igual se limpia; los demás repartidos con separación <= lane_spacing
        span = (c1 - c0) - 2 * self.edge
        n = max(1, math.ceil(span / self.spacing)) + 1
        lanes = [c0 + self.edge + span * i / (n - 1) for i in range(n)]
        segments = []
        for k, c in enumerate(lanes):
            n = max(2, int(round((a1 - a0) / self.step_len)) + 1)
            pts = [a0 + (a1 - a0) * i / (n - 1) for i in range(n)]
            run, runs = [], []
            for a in pts:
                x, y = (a, c) if along_x else (c, a)
                if self.fmap.clear(x, y, self.clearance):
                    run.append((x, y))
                elif run:
                    runs.append(run)
                    run = []
            if run:
                runs.append(run)
            runs = [r for r in runs if len(r) >= 2]
            if k % 2:                     # zigzag: carriles impares al revés
                runs = [list(reversed(r)) for r in reversed(runs)]
            for r in runs:
                yaw = math.atan2(r[-1][1] - r[0][1], r[-1][0] - r[0][0])
                segments.append([(x, y, yaw) for x, y in r])
        return segments

    # ------------------------------------------------------------ entradas
    def update_pose(self):
        """Pose de Pepper según su localización (AMCL) y memoria de cobertura del cepillo."""
        try:
            t = self.tf_buffer.lookup_transform("map", "base_footprint", rclpy.time.Time())
        except Exception:
            return
        self.pose = (t.transform.translation.x, t.transform.translation.y)
        if self.brush_on and self.state == "CLEANING":
            r = self.brush_r
            cx, cy = int(math.floor(self.pose[0] / 0.1)), int(math.floor(self.pose[1] / 0.1))
            k = int(math.ceil(r / 0.1))
            for i in range(cx - k, cx + k + 1):
                for j in range(cy - k, cy + k + 1):
                    if math.hypot((i + 0.5) * 0.1 - self.pose[0], (j + 0.5) * 0.1 - self.pose[1]) <= r:
                        self.covered.add((i, j))

    def is_covered(self, x, y):
        return (int(math.floor(x / 0.1)), int(math.floor(y / 0.1))) in self.covered

    def gaps(self, zone):
        """Tramos del repaso: puntos libres de la zona que el cepillo no cubrió, en orden de
        vecino más cercano; se corta en tramos nuevos cuando dos puntos quedan a > 1.2 m."""
        x0, x1, y0, y1 = self.zones[zone]["rect"]
        s = self.gap_step
        pts = []
        for i in range(math.floor(x0 / s), math.ceil(x1 / s)):
            for j in range(math.floor(y0 / s), math.ceil(y1 / s)):
                x, y = (i + 0.5) * s, (j + 0.5) * s
                if (x0 <= x <= x1 and y0 <= y <= y1 and self.fmap.clear(x, y, self.clearance)
                        and not self.blocked(x, y) and not self.is_covered(x, y)):
                    pts.append((x, y))
        order, cur = [], self.pose or (x0, y0)
        while pts:
            k = min(range(len(pts)), key=lambda i: math.dist(cur, pts[i]))
            cur = pts.pop(k)
            order.append(cur)
        segments, seg = [], []
        for p in order:
            if seg and math.dist(seg[-1][:2], p) > 1.2:
                segments.append(seg)
                seg = []
            prev = seg[-1][:2] if seg else (self.pose or p)
            seg.append((p[0], p[1], math.atan2(p[1] - prev[1], p[0] - prev[0])))
        if seg:
            segments.append(seg)
        return segments

    def _costmap_cb(self, msg):
        self.cm_info = msg.info
        self.cm = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)

    def _costmap_update_cb(self, u):
        if self.cm is None:
            return
        patch = np.array(u.data, dtype=np.int8).reshape(u.height, u.width)
        self.cm[u.y:u.y + u.height, u.x:u.x + u.width] = patch

    def blocked(self, x, y):
        """El centro de Pepper no cabe en (x, y) según el costmap global (≥ inscrito), o hay a menos
        de `people_margin` un obstáculo que no está en el mapa (una persona): el láser ve sus
        piernas, pero sus brazos y su cuerpo son más anchos, y Pepper la rozaba con el brazo."""
        if self.cm is None:
            return False
        info = self.cm_info
        res = info.resolution
        c = int((x - info.origin.position.x) / res)
        r = int((y - info.origin.position.y) / res)
        if not (0 <= r < info.height and 0 <= c < info.width):
            return False
        if self.cm[max(0, r - 1):r + 2, max(0, c - 1):c + 2].max() >= COST_BLOCKED:
            return True
        k = int(math.ceil(self.people_margin / res))
        win = self.cm[max(0, r - k):r + k + 1, max(0, c - k):c + k + 1]
        for rr, cc in zip(*np.nonzero(win >= COST_LETHAL)):
            ox = info.origin.position.x + (max(0, c - k) + cc + 0.5) * res
            oy = info.origin.position.y + (max(0, r - k) + rr + 0.5) * res
            # obstáculo lejos de toda pared del mapa: si AMCL está algo corrido, el láser marca las
            # paredes un poco fuera de su sitio, y eso no debe contar como persona
            if math.hypot(ox - x, oy - y) <= self.people_margin and self.fmap.clear(ox, oy, 0.25):
                return True
        return False

    def free_runs(self, pts, min_len=2):
        runs, run = [], []
        for p in pts:
            if self.blocked(p[0], p[1]):
                if run:
                    runs.append(run)
                run = []
            else:
                run.append(p)
        if run:
            runs.append(run)
        return [r for r in runs if len(r) >= min_len]

    def _dirt_cb(self, msg):
        self.dirt = json.loads(msg.data)

    def resolve(self, text):
        t = norm(text)
        for key, z in self.zones.items():
            if t in (norm(key), norm(z.get("name", key))) or norm(key) in t or norm(z.get("name", key)) in t:
                return key
        return None

    def _cmd_cb(self, msg):
        raw = msg.data.strip()
        zones = None
        try:
            cmd = json.loads(raw)
            action, zones = cmd.get("action", "start"), cmd.get("zones")
        except (ValueError, AttributeError):
            words = raw.split(maxsplit=1)
            action = words[0].lower() if words else ""
            if len(words) > 1 and norm(words[1]) not in ("all", "todo", "todas", "museo", "el museo"):
                z = self.resolve(words[1])
                if z is None:
                    self.say(f"No conozco la zona {words[1]}.")
                    return
                zones = [z]
        if action in ("stop", "cancel"):
            self.stop("Dejo de limpiar.")
        elif action == "start":
            self.start(zones)

    def _dock_cb(self, msg):
        st = json.loads(msg.data).get("state", "IDLE")
        prev, self.dock_state = self.dock_state, st
        if self.state in ("CLEANING", "PAUSED_VISITOR") and st not in ("IDLE",) and prev == "IDLE":
            self.pause_for_dock()
        elif self.state == "PAUSED_DOCK" and st == "IDLE" and prev == "UNDOCK":
            if self.visitor:      # salió de la base para atender a alguien
                self.state = "PAUSED_VISITOR"
            else:
                self.resume()
        elif self.pending is not None and st == "IDLE" and prev == "UNDOCK":
            zones, self.pending = self.pending, None
            self.start(zones or None)
        elif self.state == "PAUSED_DOCK" and st == "FAILED":
            self.stop("No pude cargarme; dejo la limpieza.")

    def _visitor_cb(self, msg):
        try:
            phase = json.loads(msg.data).get("phase", "")
        except ValueError:
            return
        if phase in ("searching", "approaching", "facing", "reached"):
            if not self.visitor and self.state in ("CLEANING", "PAUSED_DOCK"):
                self.visitors += 1
            self.visitor = True
            if self.state == "CLEANING":
                self.cancel_nav()
                self.brush(False)
                self.set_speed(0.0)
                self.state = "PAUSED_VISITOR"
                self.text = "Atiendo a un visitante."
                self.get_logger().info(f"pausa por un visitante en {self.zone}, tramo {self.seg}")
        elif phase in ("done", "nobody", "nav_failed", "charging", "error"):
            self.visitor = False
            if self.state == "PAUSED_VISITOR":
                # deja de mirar al visitante (visitor_approach suelta el objetivo con /pepper/go_to)
                self.pub_go_to.publish(String(data=json.dumps({"cancel": True})))
                self.resume()

    # ------------------------------------------------------------ control
    def say(self, text):
        self.text = text
        self.pub_say.publish(String(data=text))
        self.get_logger().info(text)

    def brush(self, on):
        self.brush_on = on
        self.pub_brush.publish(Bool(data=on))

    def set_speed(self, percent):
        self.pub_speed.publish(SpeedLimit(percentage=True, speed_limit=float(percent)))

    def start(self, zones=None):
        if self.dock_state in ("CHARGING", "UNDOCK"):
            self.pending = zones or []
            self.say("Salgo de la base y empiezo a limpiar.")
            if self.dock_state == "CHARGING":
                self.pub_dock.publish(String(data="undock"))
            return
        if self.dock_state not in ("IDLE", "FAILED"):
            self.say("Ahora voy a la base de carga; después me lo vuelves a pedir.")
            return
        self.cancel_nav()
        self.pub_go_to.publish(String(data=json.dumps({"cancel": True})))
        self.queue = list(zones or self.zones.keys())
        self.mission = list(self.queue)
        self.covered = set()
        self.done_zones = []
        self.visitors = 0
        self.t_start = self.get_clock().now()
        names = ", ".join(self.zones[z].get("name", z) for z in self.queue)
        self.say(f"Empiezo a limpiar {names}." if len(self.queue) < 3 else "Empiezo a limpiar el museo.")
        self.next_zone()

    def next_zone(self):
        if not self.queue:
            self.finish()
            return
        if self.pose is not None:     # la más cercana primero
            def dist(z):
                x0, x1, y0, y1 = self.zones[z]["rect"]
                return math.hypot((x0 + x1) / 2 - self.pose[0], (y0 + y1) / 2 - self.pose[1])
            self.queue.sort(key=dist)
        self.zone = self.queue.pop(0)
        self.segments = self.plan(self.zone)
        # empezar por el extremo del zigzag más cercano
        if self.pose is not None and self.segments:
            first, last = self.segments[0][0], self.segments[-1][-1]
            if math.dist(self.pose, last[:2]) < math.dist(self.pose, first[:2]):
                self.segments = [[(x, y, math.atan2(-math.sin(t), -math.cos(t))) for x, y, t in reversed(s)]
                                 for s in reversed(self.segments)]
        self.seg = 0
        self.in_gaps = False
        self.sparse = True
        self.publish_plan()
        self.state = "CLEANING"
        self.get_logger().info(f"zona {self.zone}: {len(self.segments)} tramos")
        self.send_segment()

    def send_segment(self):
        if self.state != "CLEANING":
            return
        if self.seg >= len(self.segments) and self.gap_pass and not self.in_gaps:
            self.in_gaps = True
            self.sparse = False
            self.segments = self.gaps(self.zone)
            self.seg = 0
            n = sum(len(g) for g in self.segments)
            self.get_logger().info(f"repaso de {self.zone}: {n} puntos sin cubrir en {len(self.segments)} tramos")
            self.publish_plan()
        if self.seg >= len(self.segments):
            self.done_zones.append(self.zone)
            self.say(f"Terminé {self.zones[self.zone].get('name', self.zone)}.")
            self.next_zone()
            return
        if not self.nav.wait_for_server(timeout_sec=2.0):
            self.stop("Nav2 no responde.")
            return
        # lo que Pepper ve ahora (personas, objetos fuera del mapa): partir el tramo en lo que queda libre
        runs = self.free_runs(self.segments[self.seg], 2 if self.sparse else 1)
        if not runs:
            self.get_logger().info(f"tramo {self.seg}: bloqueado en el costmap, lo salto")
            self.seg += 1
            self.send_segment()
            return
        self.segments[self.seg:self.seg + 1] = runs
        pts = self.segments[self.seg]
        # waypoints cada ~0.9 m (Nav2 interpola el resto); siempre el primero y el último
        keep = max(1, int(round(0.9 / self.step_len))) if self.sparse else 1
        sparse = pts[::keep] if pts[-1] in pts[::keep] else pts[::keep] + [pts[-1]]
        goal = NavigateThroughPoses.Goal()
        for x, y, yaw in sparse:
            p = PoseStamped()
            p.header.frame_id = "map"
            p.header.stamp = self.get_clock().now().to_msg()
            p.pose.position.x, p.pose.position.y = x, y
            p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            goal.poses.append(p)
        self.set_speed(self.speed)
        self.brush(True)
        self.goal_seq += 1
        seq = self.goal_seq
        self.t_goal = time.monotonic()
        length = sum(math.dist(pts[i - 1][:2], pts[i][:2]) for i in range(1, len(pts)))
        if self.pose is not None:      # más el viaje hasta el inicio del tramo
            length += 1.5 * math.dist(self.pose, pts[0][:2])
        self.goal_timeout = 40.0 + 12.0 * length
        self.nav.send_goal_async(goal).add_done_callback(lambda f: self._accepted(f, seq))

    def _accepted(self, fut, seq):
        h = fut.result()
        if seq != self.goal_seq:
            if h.accepted:
                h.cancel_goal_async()
            return
        if not h.accepted:
            self.get_logger().warn(f"Nav2 rechazó el tramo {self.seg}")
            self.seg += 1
            self.send_segment()
            return
        self.goal = h
        h.get_result_async().add_done_callback(lambda r: self._done(r, seq))

    def _done(self, res, seq):
        if seq != self.goal_seq:
            return
        self.goal = None
        status = res.result().status
        if status != 4:
            self.get_logger().warn(f"tramo {self.seg} de {self.zone}: status {status}")
            self.retry_rest()
            return
        self.retries = 0
        self.seg += 1
        self.send_segment()

    def retry_rest(self):
        """Tramo fallido (persona en medio, atasco): una vez, reintenta lo que falta desde donde quedó
        Pepper, ya con lo que el costmap ve ahora; si vuelve a fallar, sigue con el siguiente tramo."""
        pts = self.segments[self.seg]
        if self.retries < 1 and self.pose is not None and len(pts) > 2:
            k = min(range(len(pts)), key=lambda i: math.dist(self.pose, pts[i][:2]))
            rest = pts[k + 1:]
            if len(rest) >= (2 if self.sparse else 1):
                self.retries += 1
                self.segments[self.seg] = rest
                self.get_logger().info(f"reintento el resto del tramo ({len(rest)} puntos)")
                self.send_segment()
                return
        self.retries = 0
        self.seg += 1
        self.send_segment()

    def watchdog(self):
        if self.state == "CLEANING" and self.goal is not None and time.monotonic() - self.t_goal > self.goal_timeout:
            self.get_logger().warn(f"tramo {self.seg}: timeout")
            self.goal_seq += 1
            self.goal.cancel_goal_async()
            self.goal = None
            self.retry_rest()

    def cancel_nav(self):
        self.goal_seq += 1
        if self.goal is not None:
            self.goal.cancel_goal_async()
            self.goal = None

    def pause_for_dock(self):
        self.cancel_nav()
        self.brush(False)
        self.set_speed(0.0)
        self.state = "PAUSED_DOCK"
        self.get_logger().info(f"pausa para cargar en {self.zone}, tramo {self.seg}")

    def resume(self):
        self.state = "CLEANING"
        self.say(f"Sigo limpiando {self.zones[self.zone].get('name', self.zone)}.")
        self.send_segment()

    def stop(self, text):
        self.cancel_nav()
        self.brush(False)
        self.set_speed(0.0)
        self.queue = []
        self.state = "IDLE"
        self.say(text)

    def finish(self):
        self.brush(False)
        self.set_speed(0.0)
        self.state = "DONE"
        pct = self.mission_percent()
        mins = (self.get_clock().now() - self.t_start).nanoseconds / 60e9 if self.t_start else 0
        self.say(f"Terminé de limpiar en {mins:.0f} minutos" + (f": quedó {pct:.0f} % limpio." if pct is not None else "."))
        if self.dock_when_done:
            self.pub_dock.publish(String(data="dock"))

    def mission_percent(self):
        """% limpio de las zonas de esta orden (según dirt_floor)."""
        zs = [self.dirt.get("zones", {}).get(z) for z in self.mission]
        zs = [z for z in zs if z]
        if not zs:
            return None
        return 100.0 * sum(z["cleaned"] for z in zs) / max(1, sum(z["total"] for z in zs))

    # ------------------------------------------------------------ salida
    def publish_plan(self):
        path = Path()
        path.header.frame_id = "map"
        for s in self.segments:
            for x, y, yaw in s:
                p = PoseStamped()
                p.header.frame_id = "map"
                p.pose.position.x, p.pose.position.y = x, y
                p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
                path.poses.append(p)
        self.pub_plan.publish(path)

    def publish_status(self):
        z = self.zone
        self.pub_status.publish(String(data=json.dumps({
            "state": self.state, "zone": z, "zone_name": self.zones[z].get("name", z) if z else None,
            "segment": self.seg, "segments": len(self.segments), "zones_done": self.done_zones,
            "zones_left": self.queue, "percent": self.dirt.get("percent"),
            "mission_percent": None if self.mission_percent() is None else round(self.mission_percent(), 1),
            "visitors": self.visitors, "text": self.text},
            ensure_ascii=False)))


def main():
    rclpy.init()
    node = Cleaner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.brush(False)
        node.destroy_node()
        rclpy.try_shutdown()
