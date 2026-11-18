"""Pepper va a la mesa, encuentra un objeto con la cámara (YOLO) y lo coge con la mano derecha.

    ros2 run pepper_fetch fetch_object
    ros2 topic pub --once /pepper/fetch std_msgs/String "data: manzana"     # coger la manzana
    ros2 topic pub --once /pepper/fetch std_msgs/String "data: soltar"      # abrir la mano y soltarla

Corre con el entorno de yolo_ros (ultralytics en la GPU), como visitor_approach. Necesita la
simulación, Nav2 (AMCL) y move_group (pepper_fetch/launch/moveit.launch.py): lo lanza todo
`fetch_demo.launch.py`. La mesa y los objetos están en config/fetch.yaml.

Al recibir /pepper/fetch con el nombre de un objeto ("manzana", o cualquier alias de fetch.yaml):
  1. Va con Nav2 delante de la mesa (`approach_distance` del borde) y se pone de frente a ella.
  2. Baja la cabeza y busca el objeto con YOLO (clase COCO de fetch.yaml, p. ej. "apple") en la
     cámara frontal; su posición 3D sale de la cámara de profundidad (los puntos de la nube que caen
     en el centro de la caja), mediana de varias imágenes, en el frame odom.
  3. Se acerca con la base holonómica (/pepper/cmd_vel, control proporcional con la odometría)
     hasta tener el objeto en `grasp_at` (delante y a la derecha) sin tocar la mesa; a medio camino
     lo vuelve a localizar para corregir.
  4. Agarre: la cinemática inversa propia (arm_ik.py: brazo derecho + inclinación de la cadera,
     palma de lado contra el objeto) da las articulaciones; MoveIt planifica cada movimiento sin
     chocar con la mesa (caja en la escena de planificación) ni con el objeto:
     pre-agarre a `pregrasp` m del objeto → se inclina → acerca la palma → lo "pega" a la mano →
     cierra la mano → lo levanta `lift` m → se endereza con el brazo doblado delante del pecho.
     Pegarlo: el sistema DetachableJoint de Gazebo se añade en ese momento al modelo de Pepper
     (servicio /world/default/entity/system/add) con una unión fija muñeca–objeto. Es lo habitual
     en simulación: la física de contacto de una mano de 5 dedos con un único motor no sostiene
     un objeto de forma estable (y el modelo de Pepper en Gazebo no lleva dedos).
  5. "soltar": abre la mano y deshace la unión (/pepper/grasp/detach_<n>).

Publica /pepper/fetch_status (std_msgs/String con JSON: `phase` = going | looking | approaching |
reaching | grasping | holding | released | not_found | unreachable | nav_failed | busy | error,
`text` y datos) y lo que diría Pepper en /pepper/say.
"""

import json
import math
import os
import threading
import time
import unicodedata

import numpy as np
import rclpy
import yaml
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PointStamped, Pose, PoseStamped, Twist, Vector3
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import CollisionObject, Constraints, JointConstraint, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header, String
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from pepper_fetch.arm_ik import ARM, VARS, ArmIK

ARMS_DOWN = [1.45, -0.15, 0.0, 0.1, 1.0]      # pose arms_down del SRDF, hombro algo más abierto
RELEASE_WORDS = ("soltar", "suelta", "sueltala", "deja", "dejala", "release", "drop")


def norm(text):
    t = unicodedata.normalize("NFD", text.lower())
    return "".join(c for c in t if unicodedata.category(c) != "Mn")


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


def traj(joints, positions, t):
    m = JointTrajectory()
    m.joint_names = list(joints)
    m.points = [JointTrajectoryPoint(positions=[float(p) for p in positions],
                                     time_from_start=Duration(sec=int(t), nanosec=int((t % 1) * 1e9)))]
    return m


class FetchObject(Node):
    def __init__(self):
        super().__init__("fetch_object")
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory("pepper_fetch")
        cfg_path = self.declare_parameter("config", os.path.join(share, "config", "fetch.yaml")).value
        self.yolo_path = self.declare_parameter("yolo_model", os.path.expanduser("~/pepper_ws/yolo11m.pt")).value
        self.min_conf = self.declare_parameter("min_conf", 0.25).value
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.world = self.declare_parameter("world_name", "default").value
        self.robot_model = self.declare_parameter("robot_model", "pepper_MP").value
        self.grasp_at = list(self.declare_parameter("grasp_at", [0.33, -0.12]).value)   # objeto en base_footprint
        self.table_clear = self.declare_parameter("table_clearance", 0.29).value    # centro de Pepper–borde, mínimo
        self.pregrasp = self.declare_parameter("pregrasp", 0.06).value
        self.lift = self.declare_parameter("lift", 0.10).value
        self.look_pitch = self.declare_parameter("look_pitch", 0.45).value
        self.max_ik_err = self.declare_parameter("max_ik_error", 0.015).value
        # la palma, un poco por encima del centro del objeto: de lado, la mano mide ~8 cm de alto y
        # a la altura del centro rozaría la mesa
        self.grasp_dz = self.declare_parameter("grasp_height", 0.015).value
        self.log_file = self.declare_parameter("log_file", "").value
        with open(cfg_path) as f:
            self.cfg = yaml.safe_load(f)

        from ultralytics import YOLO
        t = time.time()
        self.yolo = YOLO(self.yolo_path)
        self.yolo.predict(np.zeros((480, 640, 3), np.uint8), verbose=False, device=0)
        self.get_logger().info(f"YOLO cargado en {time.time() - t:.1f} s")

        import gz.transport13 as gzt
        self.gz = gzt.Node()

        self.bridge = CvBridge()
        self.tf = Buffer()
        TransformListener(self.tf, self)
        cb = ReentrantCallbackGroup()
        self.rgb = self.depth = self.rgb_info = self.depth_info = None
        self.urdf = None
        self.ik = None
        self.joints = {}
        self.create_subscription(Image, "/pepper/camera/front/image_raw", lambda m: setattr(self, "rgb", m),
                                 qos_profile_sensor_data, callback_group=cb)
        self.create_subscription(Image, "/pepper/camera/depth/image_raw", lambda m: setattr(self, "depth", m),
                                 qos_profile_sensor_data, callback_group=cb)
        self.create_subscription(CameraInfo, "/pepper/camera/front/camera_info",
                                 lambda m: setattr(self, "rgb_info", m), qos_profile_sensor_data, callback_group=cb)
        self.create_subscription(CameraInfo, "/pepper/camera/depth/camera_info",
                                 lambda m: setattr(self, "depth_info", m), qos_profile_sensor_data, callback_group=cb)
        self.create_subscription(JointState, "/pepper/joint_states", self._js_cb, 10, callback_group=cb)
        self.create_subscription(String, "/pepper/robot_description", self._urdf_cb,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL), callback_group=cb)
        self.create_subscription(String, "/pepper/fetch", self._fetch_cb, 10, callback_group=cb)
        self.pub_head = self.create_publisher(JointTrajectory, "/pepper/Head_controller/command", 10)
        self.pub_pelvis = self.create_publisher(JointTrajectory, "/pepper/Pelvis_controller/command", 10)
        self.pub_hand = self.create_publisher(JointTrajectory, "/pepper/RightHand_controller/command", 10)
        self.pub_vel = self.create_publisher(Twist, "/pepper/cmd_vel", 10)
        self.pub_say = self.create_publisher(String, "/pepper/say", 10)
        self.pub_status = self.create_publisher(String, "/pepper/fetch_status", 10)
        self.pub_target = self.create_publisher(PointStamped, "/pepper/fetch_target", 10)
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose", callback_group=cb)
        self.move = ActionClient(self, MoveGroup, "move_action", callback_group=cb)
        self.scene = self.create_client(ApplyPlanningScene, "apply_planning_scene", callback_group=cb)

        self.busy = False
        self.edge_odom = None       # borde de la mesa medido (odom)
        self.held = None            # {"model", "label", "n"} del objeto en la mano
        self.grasps = 0             # uniones creadas (cada una con sus topics)
        self.get_logger().info("Esperando pedidos en /pepper/fetch")

    # ---------- entradas ----------
    def _js_cb(self, m):
        self.joints.update(zip(m.name, m.position))

    def _urdf_cb(self, m):
        if self.ik is None:
            self.urdf = m.data
            self.ik = ArmIK(m.data)
            self.get_logger().info("Cinemática del brazo lista (URDF de /pepper/robot_description)")

    def _fetch_cb(self, msg):
        if self.busy:
            self.status("busy", "Estoy ocupado con otro pedido.")
            return
        self.busy = True
        threading.Thread(target=self._handle, args=(msg.data,), daemon=True).start()

    def _handle(self, text):
        log = {"text": text, "t": self.now()}
        try:
            words = norm(text)
            if any(w in words.split() for w in RELEASE_WORDS):
                self.release(log)
            else:
                self.fetch(words, log)
        except Exception as e:
            self.get_logger().error(f"Error: {e!r}")
            log["error"] = repr(e)
            self.status("error", f"Error: {e}")
        finally:
            log["duration_s"] = round(self.now() - log["t"], 1)
            if self.log_file:
                with open(self.log_file, "a") as f:
                    f.write(json.dumps(log, ensure_ascii=False) + "\n")
            self.busy = False

    # ---------- utilidades ----------
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def wait(self, sec):
        end = self.now() + sec
        while rclpy.ok() and self.now() < end:
            time.sleep(0.02)

    def say(self, text):
        self.get_logger().info(f"Pepper: {text}")
        self.pub_say.publish(String(data=text))

    def status(self, phase, text="", **extra):
        self.get_logger().info(f"[{phase}] {text}")
        self.pub_status.publish(String(data=json.dumps({"phase": phase, "text": text, **extra}, ensure_ascii=False)))

    def lookup(self, target, source, stamp=None, timeout=0.5):
        try:
            return tf_to_mat(self.tf.lookup_transform(target, source, stamp or Time(),
                                                      timeout=rclpy.duration.Duration(seconds=timeout)))
        except Exception as e:
            self.get_logger().warn(f"TF {target} <- {source}: {e}", throttle_duration_sec=2.0)
            return None

    def pose2d(self, frame):
        T = self.lookup(frame, "base_footprint")
        return None if T is None else (T[0, 3], T[1, 3], math.atan2(T[1, 0], T[0, 0]))

    def head(self, yaw, pitch, t=1.0):
        self.pub_head.publish(traj(["HeadYaw", "HeadPitch"], [yaw, pitch], t))

    def hand(self, opening, t=1.0):
        self.pub_hand.publish(traj(["RHand"], [opening], t))

    def pelvis(self, pitch, t=1.5):
        self.pub_pelvis.publish(traj(["HipRoll", "HipPitch", "KneePitch"], [0.0, pitch, 0.0], t))

    def _run_action(self, client, goal, timeout):
        if not client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(f"Acción {client._action_name} no disponible")
            return None
        fut = client.send_goal_async(goal)
        while not fut.done():
            time.sleep(0.05)
        handle = fut.result()
        if not handle.accepted:
            return None
        res = handle.get_result_async()
        t0 = time.time()
        while not res.done():
            if time.time() - t0 > timeout:
                handle.cancel_goal_async()
                return None
            time.sleep(0.05)
        return res.result()

    # ---------- geometría de la mesa ----------
    def table(self):
        t = self.cfg["table"]
        cx, cy = t["center"]
        yaw = math.radians(t.get("yaw_deg", 0.0))
        side = {"south": -math.pi / 2, "north": math.pi / 2, "east": 0.0, "west": math.pi}[t.get("approach_side", "south")]
        a = yaw + side                                  # dirección del centro de la mesa hacia Pepper
        half = (t["size"][0] if side in (0.0, math.pi) else t["size"][1]) / 2
        edge = (cx + half * math.cos(a), cy + half * math.sin(a))
        return {"center": (cx, cy), "yaw": yaw, "out": a, "half": half, "edge": edge,
                "size": t["size"], "height": t["height"], "approach": t.get("approach_distance", 0.6)}

    def go_to_table(self):
        tb = self.table()
        d = tb["half"] + tb["approach"]
        x, y = tb["center"][0] + d * math.cos(tb["out"]), tb["center"][1] + d * math.sin(tb["out"])
        yaw = tb["out"] + math.pi
        g = NavigateToPose.Goal()
        g.pose = PoseStamped()
        g.pose.header.frame_id = self.map_frame
        g.pose.header.stamp = self.get_clock().now().to_msg()
        g.pose.pose.position.x, g.pose.pose.position.y = x, y
        g.pose.pose.orientation.z, g.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        r = self._run_action(self.nav, g, 180.0)
        if r is None or r.status != GoalStatus.STATUS_SUCCEEDED:
            return False
        if not self.turn_to(yaw):
            return False
        # Nav2 da por llegado a ±0.25 m: si quedó pegado a la mesa (p. ej. tras coger algo) la cámara
        # no ve el tablero; se coloca con la base holonómica en el punto exacto
        self.drive_map(x, y)
        return True

    def drive_map(self, x, y, tol=0.03, vmax=0.15, timeout=20.0):
        """Mueve la base (sin girar) hasta (x, y) del mapa, con AMCL."""
        end, ok = self.now() + timeout, 0
        while self.now() < end and ok < 5:
            p = self.pose2d(self.map_frame)
            if p is None:
                break
            dx, dy = x - p[0], y - p[1]
            ok = ok + 1 if math.hypot(dx, dy) < tol else 0
            c, s_ = math.cos(p[2]), math.sin(p[2])
            vx, vy = c * dx + s_ * dy, -s_ * dx + c * dy
            k = 0.0 if ok else 1.0
            self.pub_vel.publish(Twist(linear=Vector3(x=float(np.clip(k * vx, -vmax, vmax)),
                                                      y=float(np.clip(k * vy, -vmax, vmax)))))
            self.wait(0.05)
        self.pub_vel.publish(Twist())
        self.wait(0.3)

    def turn_to(self, yaw, tol=0.03):
        """Gira en el sitio hasta `yaw` (map), control proporcional sobre cmd_vel."""
        end, ok = self.now() + 15.0, 0
        while self.now() < end and ok < 5:
            p = self.pose2d(self.map_frame)
            if p is None:
                return False
            err = math.remainder(yaw - p[2], math.tau)
            ok = ok + 1 if abs(err) < tol else 0
            w = 0.0 if ok else math.copysign(min(max(1.2 * abs(err), 0.08), 0.5), err)
            self.pub_vel.publish(Twist(angular=Vector3(z=w)))
            self.wait(0.05)
        self.pub_vel.publish(Twist())
        self.wait(0.3)
        return ok >= 5

    # ---------- percepción ----------
    def detect(self, yolo_class, frames=6, timeout=8.0):
        """Mediana de la posición 3D (odom) del objeto de clase `yolo_class` más seguro en varias
        imágenes; None si no se ve. Devuelve también la confianza media."""
        pts, confs, seen, end = [], [], set(), self.now() + timeout
        while len(pts) < frames and self.now() < end:
            rgb, depth = self.rgb, self.depth
            if rgb is None or depth is None or id(rgb) in seen:
                time.sleep(0.05)
                continue
            seen.add(id(rgb))
            img = self.bridge.imgmsg_to_cv2(rgb, "bgr8")
            r = self.yolo.predict(img, conf=self.min_conf, verbose=False, device=0)[0]
            boxes = [(float(b.conf), [float(v) for v in b.xyxy[0]]) for b in r.boxes
                     if r.names[int(b.cls)] == yolo_class]
            if not boxes:
                continue
            conf, box = max(boxes)
            p = self.locate(box, rgb, depth)
            if p is not None:
                pts.append(p)
                confs.append(conf)
        if len(pts) < max(2, frames // 2):
            return None, 0.0
        P = np.array(pts)
        med = np.median(P, axis=0)
        P = P[np.linalg.norm(P - med, axis=1) < 0.05]
        return (np.median(P, axis=0) if len(P) else med), float(np.mean(confs))

    def locate(self, box, rgb_msg, depth_msg, frame="odom"):
        """Centro (x, y, z) en `frame` del objeto de la caja: puntos de la nube de profundidad
        proyectados en la imagen frontal dentro del centro de la caja; el más cercano (el objeto,
        no la mesa de detrás) y se suma medio radio hacia el fondo (la cámara ve su cara delantera)."""
        if self.rgb_info is None or self.depth_info is None:
            return None
        depth = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough").astype(np.float32)
        if depth_msg.encoding == "16UC1":
            depth /= 1000.0
        Kd = np.array(self.depth_info.k).reshape(3, 3)
        Kr = np.array(self.rgb_info.k).reshape(3, 3)
        T_rd = self.lookup(rgb_msg.header.frame_id, depth_msg.header.frame_id)
        T_fr = self.lookup(frame, rgb_msg.header.frame_id, Time.from_msg(rgb_msg.header.stamp))
        if T_rd is None or T_fr is None:
            return None
        v, u = np.mgrid[0:depth.shape[0], 0:depth.shape[1]]
        z = depth[v, u]
        ok = np.isfinite(z) & (z > 0.15) & (z < 3.0)
        u, v, z = u[ok], v[ok], z[ok]
        pts = np.stack([(u - Kd[0, 2]) / Kd[0, 0] * z, (v - Kd[1, 2]) / Kd[1, 1] * z, z, np.ones_like(z)])
        pr = T_rd @ pts
        pr = pr[:, pr[2] > 0.1]
        uu = Kr[0, 0] * pr[0] / pr[2] + Kr[0, 2]
        vv = Kr[1, 1] * pr[1] / pr[2] + Kr[1, 2]
        x1, y1, x2, y2 = box
        w, h = x2 - x1, y2 - y1
        inside = (uu > x1 + 0.3 * w) & (uu < x2 - 0.3 * w) & (vv > y1 + 0.3 * h) & (vv < y2 - 0.3 * h)
        if inside.sum() < 5:
            return None
        sel = pr[:, inside]
        rng = np.linalg.norm(sel[:3], axis=0)
        near = rng < np.percentile(rng, 30) + 0.03
        c = np.median(sel[:, near], axis=1)
        c[:3] += c[:3] / np.linalg.norm(c[:3]) * 0.02          # de la superficie hacia el centro
        p = T_fr @ c
        return p[:3]

    def table_edge(self, frames=5):
        """Borde de la mesa más cercano, medido con la cámara de profundidad: los puntos a la altura
        del tablero (±1.5 cm) delante de Pepper; el borde = percentil 3 de su x en base_footprint.
        Devuelve ese punto del borde (en la línea de avance de Pepper) en odom, o None."""
        h = self.table()["height"]
        xs, seen, end = [], set(), self.now() + 5.0
        while len(xs) < frames and self.now() < end:
            d = self.depth
            if d is None or self.depth_info is None or id(d) in seen:
                time.sleep(0.05)
                continue
            seen.add(id(d))
            depth = self.bridge.imgmsg_to_cv2(d, "passthrough").astype(np.float32)
            if d.encoding == "16UC1":
                depth /= 1000.0
            K = np.array(self.depth_info.k).reshape(3, 3)
            T = self.lookup("base_footprint", d.header.frame_id, Time.from_msg(d.header.stamp))
            if T is None:
                continue
            v, u = np.mgrid[0:depth.shape[0]:2, 0:depth.shape[1]:2]
            z = depth[v, u]
            ok = np.isfinite(z) & (z > 0.15) & (z < 3.0)
            u, v, z = u[ok], v[ok], z[ok]
            P = T @ np.stack([(u - K[0, 2]) / K[0, 0] * z, (v - K[1, 2]) / K[1, 1] * z, z, np.ones_like(z)])
            sel = (np.abs(P[2] - h) < 0.015) & (np.abs(P[1]) < 0.30) & (P[0] > 0.15) & (P[0] < 1.5)
            if sel.sum() > 30:
                xs.append(float(np.percentile(P[0, sel], 3)))
        if len(xs) < 2:
            return None
        Tob = self.lookup("odom", "base_footprint")
        return (Tob @ np.array([float(np.median(xs)), 0.0, 0.0, 1.0]))[:3]

    # ---------- base ----------
    def drive_to(self, obj_odom, target, tol=0.008, vmax=0.12, timeout=40.0):
        """Mueve la base (holonómica) hasta tener el punto `obj_odom` en `target` (x, y) del frame
        base_footprint, sin girar."""
        end, ok = self.now() + timeout, 0
        while self.now() < end and ok < 5:
            T = self.lookup("base_footprint", "odom")
            if T is None:
                return False
            p = T @ np.array([*obj_odom[:2], 0.0, 1.0])
            ex, ey = p[0] - target[0], p[1] - target[1]
            ok = ok + 1 if math.hypot(ex, ey) < tol else 0
            k = 0.0 if ok else 1.0
            vx = float(np.clip(k * ex, -vmax, vmax))
            vy = float(np.clip(k * ey, -vmax, vmax))
            if not ok:   # velocidad mínima para vencer el limitador
                s = math.hypot(vx, vy)
                if 0 < s < 0.02:
                    vx, vy = vx / s * 0.02, vy / s * 0.02
            self.pub_vel.publish(Twist(linear=Vector3(x=vx, y=vy)))
            self.wait(0.05)
        self.pub_vel.publish(Twist())
        self.wait(0.5)
        return ok >= 5

    def to_base(self, p_odom):
        T = self.lookup("base_footprint", "odom")
        return (T @ np.array([*p_odom[:3], 1.0]))[:3]

    # ---------- MoveIt ----------
    def set_scene(self, obj_base=None, radius=0.0):
        """Escena de planificación: la mesa (caja hasta el suelo) y, si se da, el objeto (esfera)."""
        tb = self.table()
        T = self.lookup("base_footprint", self.map_frame)
        c = T @ np.array([*tb["center"], 0.0, 1.0])
        yaw_b = tb["yaw"] + math.atan2(T[1, 0], T[0, 0])
        objs = []
        box = CollisionObject()
        box.header.frame_id = "base_footprint"
        box.id = "table"
        box.operation = CollisionObject.ADD
        box.primitives = [SolidPrimitive(type=SolidPrimitive.BOX,
                                         dimensions=[tb["size"][0], tb["size"][1], tb["height"]])]
        bp = Pose()
        bp.position.x, bp.position.y, bp.position.z = c[0], c[1], tb["height"] / 2
        bp.orientation.z, bp.orientation.w = math.sin(yaw_b / 2), math.cos(yaw_b / 2)
        if self.edge_odom is not None:
            # Pepper está de frente a la mesa: la cara cercana de la caja en el borde medido
            e = self.to_base(self.edge_odom)
            depth_x = tb["size"][1] if tb["out"] % math.pi else tb["size"][0]
            bp.position.x = e[0] + depth_x / 2
            bp.orientation.z, bp.orientation.w = 0.0, 1.0
            box.primitives[0].dimensions = [depth_x, tb["size"][0] if tb["out"] % math.pi else tb["size"][1],
                                            tb["height"]]
        box.primitive_poses = [bp]
        objs.append(box)
        ob = CollisionObject()
        ob.header.frame_id = "base_footprint"
        ob.id = "object"
        if obj_base is not None:
            ob.operation = CollisionObject.ADD
            ob.primitives = [SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[radius + 0.01])]
            op = Pose()
            op.position.x, op.position.y, op.position.z = map(float, obj_base)
            op.orientation.w = 1.0
            ob.primitive_poses = [op]
        else:
            ob.operation = CollisionObject.REMOVE
        objs.append(ob)
        ps = PlanningScene()
        ps.is_diff = True
        ps.world.collision_objects = objs
        if not self.scene.wait_for_service(timeout_sec=10.0):
            return False
        fut = self.scene.call_async(ApplyPlanningScene.Request(scene=ps))
        while not fut.done():
            time.sleep(0.02)
        return fut.result().success

    def clamp_arm(self, margin=0.12):
        """Si alguna articulación del brazo quedó fuera de sus límites (pasa con el brazo colgando
        pegado al cuerpo: RShoulderRoll ~0.0 > −0.009), la devuelve dentro con el controlador;
        si no, MoveIt rechaza el estado de partida ("Start state out of bounds")."""
        cur = [self.joints.get(k) for k in ARM]
        if None in cur:
            return
        lim = [self.ik.chain.limits[k] for k in ARM]
        fixed = [min(max(v, lo + margin), hi - margin) for v, (lo, hi) in zip(cur, lim)]
        if any(abs(a - b) > 1e-6 for a, b in zip(cur, fixed)):
            self.get_logger().info("Brazo fuera de sus límites: lo devuelvo dentro")
            if not hasattr(self, "pub_arm"):
                self.pub_arm = self.create_publisher(JointTrajectory, "/pepper/RightArm_controller/command", 10)
            self.pub_arm.publish(traj(ARM, fixed, 0.5))
            self.wait(1.0)

    def move_arm(self, q, speed=0.4):
        g = MoveGroup.Goal()
        g.request.group_name = "right_arm"
        g.request.num_planning_attempts = 5
        g.request.allowed_planning_time = 5.0
        g.request.max_velocity_scaling_factor = speed
        g.request.max_acceleration_scaling_factor = speed
        c = Constraints()
        c.joint_constraints = [JointConstraint(joint_name=k, position=q[k], tolerance_above=0.01,
                                               tolerance_below=0.01, weight=1.0) for k in ARM]
        g.request.goal_constraints = [c]
        # OMPL es aleatorio: a veces la trayectoria suavizada roza un obstáculo y ValidateSolution la
        # rechaza (INVALID_MOTION_PLAN); otro intento suele salir bien
        for attempt in range(3):
            self.wait(0.5)                     # que el brazo se asiente
            self.clamp_arm()
            r = self._run_action(self.move, g, 60.0)
            code = r.result.error_code.val if r is not None else None
            if code == 1:
                return True
            self.get_logger().warn(f"MoveIt: código {code} (intento {attempt + 1})")
        return False

    def lean_to(self, pitch):
        self.pelvis(pitch, 1.5)
        self.wait(2.0)

    # ---------- Gazebo: unir y soltar ----------
    def entity_id(self, name):
        from gz.msgs10.empty_pb2 import Empty
        from gz.msgs10.scene_pb2 import Scene
        ok, rep = self.gz.request(f"/world/{self.world}/scene/info", Empty(), Empty, Scene, 5000)
        if ok:
            for m in rep.model:
                if m.name == name:
                    return m.id
        return None

    def attach(self, model):
        """Une `model` a la muñeca derecha con un DetachableJoint añadido ahora al modelo de Pepper."""
        from gz.msgs10.boolean_pb2 import Boolean
        from gz.msgs10.entity_plugin_v_pb2 import EntityPlugin_V
        eid = self.entity_id(self.robot_model)
        if eid is None:
            return None
        self.grasps += 1
        n = f"{os.getpid()}_{self.grasps}"     # topics únicos aunque se reinicie el nodo
        req = EntityPlugin_V()
        req.entity.id = eid
        p = req.plugins.add()
        p.name = "gz::sim::systems::DetachableJoint"
        p.filename = "gz-sim-detachable-joint-system"
        p.innerxml = (f"<parent_link>r_wrist</parent_link><child_model>{model}</child_model>"
                      f"<child_link>link</child_link><detach_topic>/pepper/grasp/detach_{n}</detach_topic>"
                      f"<attach_topic>/pepper/grasp/attach_{n}</attach_topic>"
                      f"<output_topic>/pepper/grasp/state_{n}</output_topic>")
        # la respuesta tarda (se añade en el siguiente paso de la simulación): se comprueba aparte
        self.gz.request(f"/world/{self.world}/entity/system/add", req, EntityPlugin_V, Boolean, 10000)
        return n

    def detach(self, n):
        from gz.msgs10.empty_pb2 import Empty
        pub = self.gz.advertise(f"/pepper/grasp/detach_{n}", Empty)
        time.sleep(0.5)
        pub.publish(Empty())
        time.sleep(0.3)

    def model_z(self, name):
        """Altura del modelo en Gazebo (comprobación del agarre)."""
        from gz.msgs10.pose_v_pb2 import Pose_V
        got = {}
        ev = threading.Event()

        def cb(msg):
            for p in msg.pose:
                if p.name == name:
                    got["p"] = (p.position.x, p.position.y, p.position.z)
                    ev.set()
        topic = f"/world/{self.world}/pose/info"
        self.gz.subscribe(Pose_V, topic, cb)
        ev.wait(3.0)
        self.gz.unsubscribe(topic)
        return got.get("p")

    # ---------- comportamiento ----------
    def find_object(self, words):
        aliases = self.cfg.get("aliases", {})
        for o in self.cfg["objects"]:
            names = [norm(a) for a in aliases.get(o["yolo_class"], [])] + [norm(o["label"]), o["yolo_class"]]
            if any(n in words.split() for n in names):
                return o
        return None

    def fetch(self, words, log):
        obj = self.find_object(words)
        if obj is None:
            self.status("not_found", "No sé qué objeto es ése.")
            self.say("No sé qué objeto es ése. Puedo traer una manzana.")
            return
        log["object"] = obj["label"]
        if self.held:
            self.say(f"Ya tengo {self.held['label']} en la mano.")
            self.status("holding", f"Ya tengo {self.held['label']} en la mano.")
            return
        if self.ik is None:
            self.status("error", "Aún no tengo el modelo del robot.")
            return
        art = "la" if obj["label"].endswith("a") else "el"
        if abs(self.joints.get("HipRoll", 0.0)) > 0.2:
            # visto 1 vez en 40 intentos: el torso cae de lado hasta el tope de HipRoll (0.51 rad) y
            # el motor (10.2 N·m en el modelo) no lo puede volver a levantar; la cámara ve torcido
            self.status("error", "Me quedé inclinado de lado y no me puedo enderezar.")
            self.say("Me quedé inclinado de lado y no me puedo enderezar. Necesito ayuda.")
            log["result"] = "tilted"
            return
        self.say(f"Voy a la mesa por {art} {obj['label']}.")
        self.status("going", "Voy a la mesa.")
        self.hand(0.9)
        self.edge_odom = None
        if not self.go_to_table():
            self.status("nav_failed", "No pude llegar a la mesa.")
            self.say("No puedo llegar a la mesa.")
            log["result"] = "nav_failed"
            return

        # 1) mirar la mesa y localizar el objeto
        self.status("looking", f"Busco {art} {obj['label']} en la mesa.")
        p, conf = None, 0.0
        for yaw in (0.0, -0.35, 0.35):
            self.head(yaw, self.look_pitch)
            self.wait(1.5)
            p, conf = self.detect(obj["yolo_class"])
            if p is not None:
                break
        if p is None:
            self.status("not_found", f"No veo ninguna {obj['label']} en la mesa.")
            self.say(f"No veo {art} {obj['label']} en la mesa.")
            log["result"] = "not_found"
            return
        log["detected_odom"] = [round(float(v), 3) for v in p]
        log["conf"] = round(conf, 2)
        self.pub_target.publish(PointStamped(header=Header(frame_id="odom", stamp=self.get_clock().now().to_msg()),
                                             point=Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))))
        self.get_logger().info(f"{obj['label']} en odom {np.round(p, 3)} (YOLO {conf:.2f})")

        # 2) acercarse: el objeto delante a la derecha; sin pasar de `table_clear` del borde
        self.status("approaching", f"Me acerco a {art} {obj['label']}.")
        target = list(self.grasp_at)
        tb = self.table()
        # borde de la mesa con la cámara de profundidad (el del mapa lleva el error de AMCL, ~10 cm)
        self.head(0.0, self.look_pitch)
        self.wait(1.0)
        self.edge_odom = self.table_edge()
        if self.edge_odom is not None:
            to_edge = self.to_base(p)[0] - self.to_base(self.edge_odom)[0]
            log["edge_source"] = "depth"
        else:
            Tm = self.lookup(self.map_frame, "odom")
            pm = Tm @ np.array([*p[:3], 1.0])
            to_edge = (pm[0] - tb["edge"][0]) * -math.cos(tb["out"]) + (pm[1] - tb["edge"][1]) * -math.sin(tb["out"])
            log["edge_source"] = "map"
        # distancia del objeto al borde de la mesa (hacia Pepper)
        target[0] = max(target[0], to_edge + self.table_clear)
        log["object_from_edge"] = round(float(to_edge), 3)
        mid = [target[0] + 0.15, target[1]]          # a medio camino se vuelve a mirar
        self.head(0.0, 0.6)
        self.drive_to(p, mid)
        self.wait(1.0)
        p2, _ = self.detect(obj["yolo_class"], frames=4, timeout=5.0)
        if p2 is not None and np.linalg.norm(p2[:2] - p[:2]) < 0.15:
            p = p2
            log["refined_odom"] = [round(float(v), 3) for v in p]
        self.drive_to(p, target)
        ob = self.to_base(p)
        log["object_base"] = [round(float(v), 3) for v in ob]
        self.get_logger().info(f"{obj['label']} en base_footprint {np.round(ob, 3)}")

        # 3) agarre
        self.status("reaching", "Estiro el brazo.", object_base=log["object_base"])
        r = obj.get("radius", 0.035)
        og = ob + np.array([0.0, 0.0, self.grasp_dz])
        q_pre, e_pre = self.ik.solve(og, r, approach=self.pregrasp)
        q_grasp, e_grasp = self.ik.solve(og, r, approach=0.0, seeds=[[q_pre[k] for k in VARS]], lean=q_pre["HipPitch"])
        log["ik_error_cm"] = [round(e_pre * 100, 2), round(e_grasp * 100, 2)]
        if max(e_pre, e_grasp) > self.max_ik_err:
            self.status("unreachable", f"No alcanzo {art} {obj['label']} (me falta {max(e_pre, e_grasp) * 100:.0f} cm).")
            self.say(f"No alcanzo {art} {obj['label']}.")
            log["result"] = "unreachable"
            self.head(0.0, 0.0)
            return
        self.set_scene(ob, r)
        self.hand(0.95)
        self.lean_to(q_pre["HipPitch"])
        if not self.move_arm(q_pre):
            return self.abort(log, "No encuentro cómo mover el brazo sin chocar.", "unreachable")
        self.set_scene(None)                       # el objeto deja de ser obstáculo: la palma va a tocarlo
        if not self.move_arm(q_grasp, speed=0.2):
            return self.abort(log, "No encuentro cómo acercar la mano.", "unreachable")
        self.status("grasping", "Lo cojo.")
        before = self.model_z(obj["model"])
        n = self.attach(obj["model"])
        self.hand(0.25)
        self.wait(1.0)
        # 4) levantar y llevar
        up = og + np.array([0.0, 0.0, self.lift])
        q_up, e_up = self.ik.solve(up, r, approach=0.0, seeds=[[q_grasp[k] for k in VARS]], lean=q_grasp["HipPitch"])
        self.move_arm(q_up, speed=0.2)
        self.lean_to(0.0)
        # delante del pecho pero separado del cuerpo (más cerca, la mano se aprieta contra la barriga
        # y empuja RShoulderRoll a su límite)
        q_carry, _ = self.ik.solve([0.30, -0.22, 0.88], r, approach=0.0, lean=0.0, side=False,
                                   seeds=[[0.0] + [q_up[k] for k in ARM]])
        self.set_scene(None)
        self.move_arm(q_carry, speed=0.3)
        self.head(0.0, 0.25)
        self.wait(1.0)
        pos = self.model_z(obj["model"])
        # en la mano si subió con ella (en la pose de llevarla queda 8–13 cm más alta que en la mesa)
        rise = pos[2] - before[2] if pos and before else None
        held = rise is not None and rise > 0.04
        log["lift_m"] = round(rise, 3) if rise is not None else None
        log["object_world_after"] = [round(v, 3) for v in pos] if pos else None
        log["result"] = "holding" if held else "dropped"
        if held:
            self.held = {"model": obj["model"], "label": obj["label"], "n": n}
            self.status("holding", f"Tengo {art} {obj['label']}.", grasp=n)
            self.say(f"¡Ya tengo {art} {obj['label']}!")
        else:
            if n:
                self.detach(n)
            self.status("error", f"Se me cayó {art} {obj['label']}.")
            self.say(f"Vaya, se me cayó {art} {obj['label']}.")

    def abort(self, log, text, phase):
        self.status(phase, text)
        self.say(text)
        log["result"] = phase
        self.retract()

    def retract(self):
        """Brazo abajo (pose arms_down del SRDF) y de pie."""
        self.set_scene(None)
        self.move_arm(dict(zip(ARM, ARMS_DOWN)), speed=0.3)
        self.lean_to(0.0)
        self.head(0.0, 0.0)

    def release(self, log):
        if not self.held:
            self.status("released", "No tengo nada en la mano.")
            return
        self.hand(0.95)
        self.wait(0.8)
        self.detach(self.held["n"])
        log["result"] = "released"
        self.say(f"Aquí tienes {'la' if self.held['label'].endswith('a') else 'el'} {self.held['label']}.")
        self.held = None
        self.wait(1.0)
        self.retract()
        self.status("released", "Lo solté.")


def main():
    rclpy.init()
    node = FetchObject()
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
