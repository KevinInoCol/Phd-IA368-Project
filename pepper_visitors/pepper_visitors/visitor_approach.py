"""Cuando un visitante llama a Pepper, lo busca, se acerca a él y lo sigue mirando.

    ros2 run pepper_visitors visitor_approach
    ros2 topic pub --once /pepper/visitor_call std_msgs/String "data: hola pepper"   # llamada a mano

Corre con el entorno de yolo_ros (ultralytics en la GPU); el ejecutable `visitor_approach`
instalado es un envoltorio que lo usa. Necesita la simulación y Nav2 (AMCL en el mapa del museo):
lo lanza todo `visitors_demo.launch.py`.

Al recibir /pepper/visitor_call (std_msgs/String, de voice_listener):
  1. Búsqueda: barrido de HeadYaw (`head_yaws`). En cada posición espera a que la cabeza se
     asiente y observa `dwell` s: YOLO (persona) sobre la cámara frontal; la posición de cada
     persona sale de la cámara de profundidad (su nube, proyectada en la imagen frontal con TF,
     dentro del torso de la caja) y se lleva al frame `map`.
  2. Seguimiento entre frames: una persona es estática si casi todas sus posiciones (`static_frac`)
     quedan a menos de `static_tol` de su mediana; los actores que caminan recorren más de 1 m en
     el tiempo de observación.
     Se descarta la posición que cae sobre algo del mapa del museo vacío (/map): un visitante no
     está ahí; suele ser una persona tapada por un pedestal, cuya profundidad es la del pedestal.
  3. El barrido de cabeza deja ~64° sin ver a la espalda: gira la base 180° (Spin de Nav2) y mira
     atrás (`full_search`; si es false, sólo cuando delante no hay nadie). Si no ve a nadie
     estático a menos de `max_distance`, lo dice ("No veo a nadie cerca") en /pepper/say.
  4. Elige a la persona estática más cercana que el clasificador (visitor_cls.pt, votando con
     varios recortes) no tome por un actor ("otro"); su clase (aspecto) sirve para saludarla. Va con Nav2 (NavigateToPose) a `approach_distance` m de ella, de frente; si ese
     punto no se puede alcanzar, prueba otros ángulos alrededor de la persona.
  5. Desde que la elige, la sigue con la cabeza (Head_controller) hasta la siguiente llamada o
     hasta que place_navigator lo mande a otro lugar (/pepper/go_to).
  6. Al llegar (o si ya estaba cerca) gira la base hasta quedar de frente a la persona
     (`face_tolerance`, control proporcional en /pepper/cmd_vel): cuerpo y cabeza quedan
     alineados con ella.

Publica /pepper/say (lo que diría Pepper), /pepper/visitor_target (PointStamped, la persona
elegida), /pepper/visitor_status (std_msgs/String con JSON: `phase` = searching | approaching |
facing | reached | nobody | nav_failed | busy | error, `text` y la llamada; lo muestra la app) y,
si `log_file` no está vacío, una línea JSON por llamada (candidatos, elegida, éxito).
"""

import json
import math
import os
import threading
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, PoseStamped, Twist, Vector3
from nav2_msgs.action import NavigateToPose, Spin
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Cómo describir a cada clase del clasificador al saludar
LOOKS = {
    "male_03": "camisa a cuadros roja",
    "female_02": "vestido negro",
    "female_03": "chaqueta marrón",
}
HEAD_PIVOT_Z = 1.2   # altura aproximada del eje de HeadPitch sobre el suelo
LOOK_AT_Z = 1.5      # altura a la que mira (cara/pecho de la persona)


def quat_to_mat(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def tf_to_mat(tf):
    T = np.eye(4)
    T[:3, :3] = quat_to_mat(tf.transform.rotation)
    t = tf.transform.translation
    T[:3, 3] = (t.x, t.y, t.z)
    return T


class VisitorApproach(Node):
    def __init__(self):
        super().__init__("visitor_approach")
        share = self._share()
        self.yolo_path = self.declare_parameter("yolo_model", os.path.expanduser("~/pepper_ws/yolo11m.pt")).value
        self.cls_path = self.declare_parameter("cls_model", os.path.join(share, "models", "visitor_cls.pt")).value
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.head_yaws = list(self.declare_parameter("head_yaws", [0.0, 1.0, 2.0, -1.0, -2.0]).value)
        self.back_yaws = list(self.declare_parameter("back_yaws", [-0.5, 0.5]).value)
        self.head_pitch = self.declare_parameter("search_pitch", -0.1).value
        self.settle = self.declare_parameter("settle", 1.0).value
        self.dwell = self.declare_parameter("dwell", 2.0).value
        self.min_conf = self.declare_parameter("min_conf", 0.5).value
        self.static_tol = self.declare_parameter("static_tol", 0.12).value
        self.static_frac = self.declare_parameter("static_frac", 0.7).value
        self.max_dist = self.declare_parameter("max_distance", 6.0).value
        self.approach = self.declare_parameter("approach_distance", 1.0).value
        self.full_search = self.declare_parameter("full_search", True).value
        self.map_clearance = self.declare_parameter("map_clearance", 0.2).value
        self.face_tol = self.declare_parameter("face_tolerance", 0.05).value  # rad
        self.log_file = self.declare_parameter("log_file", "").value

        from ultralytics import YOLO
        t = time.time()
        self.yolo = YOLO(self.yolo_path)
        self.cls = YOLO(self.cls_path) if os.path.exists(self.cls_path) else None
        dummy = np.zeros((480, 640, 3), np.uint8)
        self.yolo.predict(dummy, verbose=False, device=0)
        self.get_logger().info(f"YOLO cargado en {time.time() - t:.1f} s (clasificador: "
                               f"{'sí' if self.cls else 'no, ' + self.cls_path})")

        self.bridge = CvBridge()
        self.tf = Buffer()
        TransformListener(self.tf, self)
        cb = ReentrantCallbackGroup()
        self.rgb = self.depth = self.rgb_info = self.depth_info = None
        self.lock = threading.Lock()
        self.create_subscription(Image, "/pepper/camera/front/image_raw", self._rgb_cb, qos_profile_sensor_data, callback_group=cb)
        self.create_subscription(Image, "/pepper/camera/depth/image_raw", self._depth_cb, qos_profile_sensor_data, callback_group=cb)
        self.create_subscription(CameraInfo, "/pepper/camera/front/camera_info", self._rgb_info_cb, qos_profile_sensor_data, callback_group=cb)
        self.create_subscription(CameraInfo, "/pepper/camera/depth/camera_info", self._depth_info_cb, qos_profile_sensor_data, callback_group=cb)
        self.create_subscription(String, "/pepper/visitor_call", self._call_cb, 10, callback_group=cb)
        self.create_subscription(String, "/pepper/go_to", self._go_to_cb, 10, callback_group=cb)
        self.map = None
        self.create_subscription(OccupancyGrid, "/map", self._map_cb,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL), callback_group=cb)
        self.pub_head = self.create_publisher(JointTrajectory, "/pepper/Head_controller/command", 10)
        self.pub_say = self.create_publisher(String, "/pepper/say", 10)
        self.pub_target = self.create_publisher(PointStamped, "/pepper/visitor_target", 10)
        self.pub_status = self.create_publisher(String, "/pepper/visitor_status", 10)
        self.pub_vel = self.create_publisher(Twist, "/pepper/cmd_vel", 10)   # giro final de frente
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose", callback_group=cb)
        self.spin = ActionClient(self, Spin, "spin", callback_group=cb)

        self.target = None          # (x, y) en map de la persona que mira
        self.busy = False
        self.call_id = 0
        self.create_timer(0.1, self._look_at_target, callback_group=cb)
        self.get_logger().info("Esperando llamadas en /pepper/visitor_call")

    @staticmethod
    def _share():
        try:
            from ament_index_python.packages import get_package_share_directory
            return get_package_share_directory("pepper_visitors")
        except Exception:
            return ""

    # ---------- entradas ----------
    def _rgb_cb(self, msg):
        with self.lock:
            self.rgb = msg

    def _depth_cb(self, msg):
        with self.lock:
            self.depth = msg

    def _rgb_info_cb(self, msg):
        self.rgb_info = msg

    def _depth_info_cb(self, msg):
        self.depth_info = msg

    def _map_cb(self, msg):
        g = np.array(msg.data, np.int8).reshape(msg.info.height, msg.info.width)
        self.map = (g > 50, msg.info.resolution, msg.info.origin.position.x, msg.info.origin.position.y)

    def on_map_obstacle(self, x, y):
        """¿Hay algo del museo vacío (≥ 2 celdas ocupadas) a menos de map_clearance de (x, y)?"""
        if self.map is None:
            return False
        occ, res, ox, oy = self.map
        c, r = int((x - ox) / res), int((y - oy) / res)
        k = int(math.ceil(self.map_clearance / res))
        win = occ[max(r - k, 0):r + k + 1, max(c - k, 0):c + k + 1]
        return int(win.sum()) >= 2

    def _call_cb(self, msg):
        if self.busy:
            self.get_logger().info(f"Llamada ignorada (ocupado): '{msg.data}'")
            self.status("busy", "Estoy atendiendo otra llamada.", call=msg.data)
            return
        self.busy = True
        threading.Thread(target=self._handle_call, args=(msg.data,), daemon=True).start()

    def _go_to_cb(self, msg):
        """place_navigator lleva a Pepper a otro lugar: deja de mirar al visitante."""
        if not self.busy and self.target is not None:
            self.target = None
            self.head(0.0, 0.0)

    # ---------- utilidades ----------
    def say(self, text):
        self.get_logger().info(f"Pepper: {text}")
        self.pub_say.publish(String(data=text))

    def status(self, phase, text="", **extra):
        self.pub_status.publish(String(data=json.dumps(
            {"phase": phase, "text": text, "call_id": self.call_id, **extra}, ensure_ascii=False)))

    def wait(self, sec):
        """Espera en tiempo simulado."""
        end = self.get_clock().now().nanoseconds + sec * 1e9
        while rclpy.ok() and self.get_clock().now().nanoseconds < end:
            time.sleep(0.02)

    def head(self, yaw, pitch, t=0.8):
        traj = JointTrajectory()
        traj.joint_names = ["HeadYaw", "HeadPitch"]
        p = JointTrajectoryPoint()
        p.positions = [float(np.clip(yaw, -2.08, 2.08)), float(np.clip(pitch, -0.7, 0.63))]
        p.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
        traj.points = [p]
        self.pub_head.publish(traj)

    def lookup(self, target, source, stamp=None, timeout=0.5):
        try:
            return tf_to_mat(self.tf.lookup_transform(target, source, stamp or Time(),
                                                      timeout=rclpy.duration.Duration(seconds=timeout)))
        except Exception as e:
            self.get_logger().warn(f"TF {target} <- {source}: {e}", throttle_duration_sec=2.0)
            return None

    def robot_pose(self):
        T = self.lookup(self.map_frame, "base_footprint")
        if T is None:
            return None
        return T[0, 3], T[1, 3], math.atan2(T[1, 0], T[0, 0])

    def _look_at_target(self):
        if self.target is None:
            return
        T = self.lookup("base_footprint", self.map_frame, timeout=0.05)
        if T is None:
            return
        p = T @ np.array([self.target[0], self.target[1], 0.0, 1.0])
        d = math.hypot(p[0], p[1])
        # HeadPitch positivo = mirar hacia abajo
        self.head(math.atan2(p[1], p[0]), -math.atan2(LOOK_AT_Z - HEAD_PIVOT_Z, max(d, 0.3)), t=0.3)

    # ---------- percepción ----------
    def locate(self, box, rgb_msg, depth_msg):
        """Posición (x, y) en map de la persona de la caja `box` (x1, y1, x2, y2), con la nube de la
        cámara de profundidad proyectada en la imagen frontal; None si no hay puntos."""
        if self.rgb_info is None or self.depth_info is None:
            return None
        depth = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough").astype(np.float32)
        if depth_msg.encoding == "16UC1":
            depth /= 1000.0
        Kd = np.array(self.depth_info.k).reshape(3, 3)
        Kr = np.array(self.rgb_info.k).reshape(3, 3)
        T_rd = self.lookup(rgb_msg.header.frame_id, depth_msg.header.frame_id)
        T_mr = self.lookup(self.map_frame, rgb_msg.header.frame_id, Time.from_msg(rgb_msg.header.stamp))
        if T_rd is None or T_mr is None:
            return None
        v, u = np.mgrid[0:depth.shape[0]:2, 0:depth.shape[1]:2]
        z = depth[v, u]
        ok = np.isfinite(z) & (z > 0.3) & (z < 8.0)
        u, v, z = u[ok], v[ok], z[ok]
        pts = np.stack([(u - Kd[0, 2]) / Kd[0, 0] * z, (v - Kd[1, 2]) / Kd[1, 1] * z, z, np.ones_like(z)])
        pr = T_rd @ pts
        front = pr[2] > 0.1
        pr = pr[:, front]
        uu = Kr[0, 0] * pr[0] / pr[2] + Kr[0, 2]
        vv = Kr[1, 1] * pr[1] / pr[2] + Kr[1, 2]
        x1, y1, x2, y2 = box
        w, h = x2 - x1, y2 - y1
        inside = (uu > x1 + 0.3 * w) & (uu < x2 - 0.3 * w) & (vv > y1 + 0.2 * h) & (vv < y1 + 0.6 * h)
        if inside.sum() < 8:
            return None
        sel = pr[:, inside]
        rng = np.linalg.norm(sel[:3], axis=0)
        near = rng < np.percentile(rng, 20) + 0.4   # la persona, no la pared de detrás
        pm = T_mr @ np.median(sel[:, near], axis=1)
        return float(pm[0]), float(pm[1])

    def observe(self, duration):
        """Detecta personas durante `duration` s con la cabeza quieta; devuelve las pistas
        [{'pos': (x, y), 'static': bool, 'on_map': bool, 'spread', 'n', 'crop', 'conf'}].

        Asociación uno a uno por frame (la pista más cercana a menos de 0.6 m; dos detecciones del
        mismo frame no van a la misma pista). Una pista es estática si al menos `static_frac` de sus
        puntos están a menos de `static_tol` de su mediana: así un actor que pasa al lado de una
        persona quieta y se cuela en su pista unos frames no la convierte en "en movimiento"."""
        tracks, frames, last = [], 0, None
        end = self.get_clock().now().nanoseconds + duration * 1e9
        while rclpy.ok() and self.get_clock().now().nanoseconds < end:
            with self.lock:
                rgb, depth = self.rgb, self.depth
            if rgb is None or depth is None or rgb.header.stamp == last:
                time.sleep(0.02)
                continue
            last = rgb.header.stamp
            img = self.bridge.imgmsg_to_cv2(rgb, "bgr8")
            res = self.yolo.predict(img, classes=[0], conf=self.min_conf, verbose=False, device=0)[0]
            frames += 1
            dets = []
            for b, c in zip(res.boxes.xyxy.cpu().numpy(), res.boxes.conf.cpu().numpy()):
                pos = self.locate(b, rgb, depth)
                if pos is not None:
                    x1, y1, x2, y2 = b.astype(int)
                    dets.append((pos, float(c), img[max(y1, 0):y2, max(x1, 0):x2].copy()))
            pairs = sorted((math.dist(tr["pts"][-1][0], d[0]), i, j)
                           for i, tr in enumerate(tracks) for j, d in enumerate(dets))
            used_t, used_d = set(), set()
            for dist, i, j in pairs:
                if dist > 0.6:
                    break
                if i not in used_t and j not in used_d:
                    tracks[i]["pts"].append(dets[j])
                    used_t.add(i)
                    used_d.add(j)
            tracks += [{"pts": [d]} for j, d in enumerate(dets) if j not in used_d]
        out = []
        for tr in tracks:
            pos = np.array([p for p, _, _ in tr["pts"]])
            if len(pos) < max(3, 0.4 * frames):
                continue
            med = np.median(pos, axis=0)
            d = np.linalg.norm(pos - med, axis=1)
            inl = d < self.static_tol
            static = inl.mean() >= self.static_frac
            if static:
                med = np.median(pos[inl], axis=0)
            spread = float(np.percentile(d, 100 * self.static_frac))
            best = sorted((i for i in range(len(pos)) if inl[i] or not static),
                          key=lambda i: -tr["pts"][i][1])[:5]
            on_map = self.on_map_obstacle(*med)
            out.append({"pos": (float(med[0]), float(med[1])), "static": bool(static and not on_map),
                        "on_map": on_map, "spread": spread, "n": len(pos),
                        "crops": [tr["pts"][i][2] for i in best], "conf": tr["pts"][best[0]][1]})
        return out

    def classify(self, crops):
        """Clase por votación de los recortes (los de más confianza de YOLO) y su probabilidad media."""
        crops = [c for c in crops if c is not None and c.size]
        if self.cls is None or not crops:
            return None, 0.0
        probs = np.mean([r.probs.data.cpu().numpy() for r in self.cls.predict(crops, verbose=False, device=0)], axis=0)
        k = int(np.argmax(probs))
        return self.cls.names[k], float(probs[k])

    def search(self, yaws):
        """Barrido de cabeza; personas encontradas (fusionadas entre posiciones de la cabeza)."""
        people = []
        for yaw in yaws:
            self.head(yaw, self.head_pitch)
            self.wait(self.settle)
            for p in self.observe(self.dwell):
                same = next((q for q in people if math.dist(q["pos"], p["pos"]) < 0.6), None)
                if same is None:
                    people.append(p)
                elif p["n"] > same["n"]:      # se queda con la observación más completa
                    people[people.index(same)] = p
            self.get_logger().info(f"HeadYaw {yaw:+.1f}: {len(people)} persona(s) hasta ahora")
        return people

    # ---------- acciones de Nav2 ----------
    def _run_action(self, client, goal, timeout):
        if not client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f"Acción {client._action_name} no disponible")
            return False
        fut = client.send_goal_async(goal)
        t0 = time.time()
        while not fut.done():
            time.sleep(0.05)
        handle = fut.result()
        if not handle.accepted:
            return False
        res = handle.get_result_async()
        while not res.done():
            if time.time() - t0 > timeout:
                handle.cancel_goal_async()
                return False
            time.sleep(0.1)
        return res.result().status == GoalStatus.STATUS_SUCCEEDED

    def spin_base(self, angle):
        g = Spin.Goal()
        g.target_yaw = float(angle)
        g.time_allowance = Duration(sec=30)
        return self._run_action(self.spin, g, 40.0)

    def go_to(self, x, y, yaw):
        g = NavigateToPose.Goal()
        g.pose = PoseStamped()
        g.pose.header.frame_id = self.map_frame
        g.pose.header.stamp = self.get_clock().now().to_msg()
        g.pose.pose.position.x, g.pose.pose.position.y = float(x), float(y)
        g.pose.pose.orientation.z, g.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        return self._run_action(self.nav, g, 120.0)

    # ---------- comportamiento ----------
    def _handle_call(self, text):
        self.call_id += 1
        log = {"call": self.call_id, "text": text, "t": self.get_clock().now().nanoseconds * 1e-9}
        try:
            self._behave(text, log)
        except Exception as e:  # no dejar el nodo bloqueado en "ocupado"
            self.get_logger().error(f"Error atendiendo la llamada: {e!r}")
            log["error"] = repr(e)
            self.status("error", f"Error: {e}")
        finally:
            log["duration_s"] = self.get_clock().now().nanoseconds * 1e-9 - log["t"]
            if self.log_file:
                with open(self.log_file, "a") as f:
                    f.write(json.dumps(log) + "\n")
            self.busy = False

    def _behave(self, text, log):
        self.get_logger().info(f"Llamada: '{text}'")
        self.target = None
        self.status("searching", "Busco con la cámara a quien me habló.", call=text)
        self.say("¿Sí? Déjame ver quién me llama.")
        robot = self.robot_pose()
        if robot is None:
            self.say("No sé dónde estoy (sin localización).")
            self.status("error", "Sin localización (AMCL).")
            log["result"] = "no_localization"
            return
        people = self.search(self.head_yaws)
        cands = self._candidates(people, robot)
        if self.full_search or not cands:
            self.head(0.0, self.head_pitch)
            self.wait(0.5)
            self.get_logger().info("Miro atrás: giro de la base 180°" if cands else "Nadie delante: giro de la base 180°")
            self.spin_base(math.pi)
            for p in self.search(self.back_yaws):
                if not any(math.dist(p["pos"], q["pos"]) < 0.6 for q in people):
                    people.append(p)
            cands = self._candidates(people, robot)   # distancias desde donde lo llamaron
        log["robot_start"] = robot
        log["people"] = [{k: p[k] for k in ("pos", "static", "on_map", "spread", "n", "conf")} for p in people]
        if not cands:
            self.head(0.0, 0.0)
            moving = sum(not p["static"] for p in people)
            self.say("No veo a nadie cerca." + (" Sólo gente que pasa caminando." if moving else ""))
            self.status("nobody", "No encontré a nadie quieto cerca.", people=len(people))
            log["result"] = "nobody"
            return

        # La más cercana que el clasificador no tome por alguien que camina ("otro": actores que
        # parecieron quietos unos segundos, p. ej. al girar en una esquina de su trayectoria)
        tgt = label = None
        for c in cands:
            c["class"], c["prob"] = self.classify(c["crops"])
            if c["class"] != "otro":
                tgt, label, prob = c, c["class"], c["prob"]
                break
            self.get_logger().info(f"({c['pos'][0]:.2f}, {c['pos'][1]:.2f}) parece alguien que camina: descartada")
        log["candidates"] = [{k: c.get(k) for k in ("pos", "dist", "class", "prob")} for c in cands]
        if tgt is None:
            self.head(0.0, 0.0)
            self.say("No veo a nadie cerca. Sólo gente que pasa caminando.")
            self.status("nobody", "Sólo vi gente caminando.", people=len(people))
            log["result"] = "nobody"
            return
        log["target"] = {"pos": tgt["pos"], "dist": tgt["dist"], "class": label, "prob": prob}
        self.target = tgt["pos"]
        self.pub_target.publish(PointStamped(header=self._hdr(), point=self._pt(tgt["pos"])))
        looks = LOOKS.get(label) if prob >= 0.6 else None   # con poca confianza, mejor no describirla
        self.say(f"Te veo{', la persona de ' + looks if looks else ''}. Voy hacia ti.")
        self.status("approaching", f"Persona a {tgt['dist']:.1f} m" + (f" ({looks})" if looks else "") + ".",
                    target=list(tgt["pos"]), dist=round(tgt["dist"], 2), people=len(people))
        self.get_logger().info(f"Objetivo: ({tgt['pos'][0]:.2f}, {tgt['pos'][1]:.2f}) a {tgt['dist']:.2f} m, "
                               f"clase {label} ({prob:.2f})")

        if tgt["dist"] <= self.approach + 0.3:
            ok = True     # ya está a la distancia de conversación
        else:
            ok = False
            px, py = tgt["pos"]
            base = math.atan2(robot[1] - py, robot[0] - px)    # desde la persona hacia el robot
            for k, da in enumerate((0.0, 0.5, -0.5, 1.0, -1.0, 1.6, -1.6)):
                a = base + da
                gx, gy = px + self.approach * math.cos(a), py + self.approach * math.sin(a)
                self.get_logger().info(f"Nav2: intento {k + 1} a ({gx:.2f}, {gy:.2f})")
                if self.go_to(gx, gy, a + math.pi):
                    ok = True
                    break
        if ok:
            self.status("facing", "Me pongo de frente.", target=list(tgt["pos"]))
            log["facing_error"] = self.face(tgt["pos"])
        end = self.robot_pose()
        log["robot_end"] = end
        if end:
            log["final_dist"] = math.dist(end[:2], tgt["pos"])
        log["result"] = "reached" if ok else "nav_failed"
        if ok:
            self.say("Hola, ¿en qué te puedo ayudar?")
            err = log["facing_error"]
            self.status("reached", f"A {log.get('final_dist', 0):.2f} m, de frente"
                        + (f" ({math.degrees(err):+.0f}°)." if err is not None else "."),
                        target=list(tgt["pos"]), dist=round(log.get("final_dist", 0), 2))
        else:
            self.say("Lo siento, no consigo llegar hasta ti.")
            self.status("nav_failed", "Nav2 no encontró cómo llegar.", target=list(tgt["pos"]))
        # Sigue mirando a self.target (timer) hasta la siguiente llamada

    def face(self, xy):
        """Gira la base hasta que la persona en `xy` quede de frente (Nav2 deja ±0.25 rad de
        tolerancia de orientación y, si ya estaba cerca, ni se mueve). Control proporcional sobre
        /pepper/cmd_vel: el Spin de Nav2 se pasa de largo en giros de pocos grados. Devuelve el
        error final (rad)."""
        def error():
            robot = self.robot_pose()
            if robot is None:
                return None
            return math.remainder(math.atan2(xy[1] - robot[1], xy[0] - robot[0]) - robot[2], math.tau)

        err, ok = error(), 0
        if err is not None and abs(err) > self.face_tol:
            self.get_logger().info(f"Me giro {math.degrees(err):+.0f}° hacia la persona")
        end = self.get_clock().now().nanoseconds + 15e9
        while err is not None and ok < 5 and self.get_clock().now().nanoseconds < end:
            ok = ok + 1 if abs(err) < self.face_tol else 0
            w = 0.0 if ok else math.copysign(min(max(1.0 * abs(err), 0.12), 0.6), err)
            self.pub_vel.publish(Twist(angular=Vector3(z=w)))
            self.wait(0.05)
            err = error()
        self.pub_vel.publish(Twist())
        self.wait(0.5)
        return error()

    def _candidates(self, people, robot):
        out = []
        for p in people:
            d = math.dist(p["pos"], robot[:2])
            if p["static"] and d <= self.max_dist:
                out.append(dict(p, dist=d))
        return sorted(out, key=lambda p: p["dist"])

    def _hdr(self):
        from std_msgs.msg import Header
        return Header(frame_id=self.map_frame, stamp=self.get_clock().now().to_msg())

    @staticmethod
    def _pt(xy):
        from geometry_msgs.msg import Point
        return Point(x=xy[0], y=xy[1], z=0.0)


def main():
    rclpy.init()
    node = VisitorApproach()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
