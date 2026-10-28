"""Lleva a Pepper a los lugares con nombre del museo ("vamos a la mesa", "corre a la entrada").

    ros2 run pepper_visitors place_navigator
    ros2 topic pub --once /pepper/go_to std_msgs/String "data: la mesa"                       # caminando
    ros2 topic pub --once /pepper/go_to std_msgs/String '{data: "{\"place\": \"mesa\", \"run\": true}"}'
    ros2 topic pub --once /pepper/go_to std_msgs/String '{data: "{\"cancel\": true}"}'

Lee los lugares de `places_file` (por defecto pepper_museum/maps/museum_places.yaml: nombre,
alias y pose en el frame map) y espera órdenes en /pepper/go_to (std_msgs/String): el nombre o un
alias del lugar (sin importar tildes, mayúsculas ni artículos), o JSON {"place", "run"} /
{"cancel": true}. Manda la pose a Nav2 (NavigateToPose); una orden nueva reemplaza a la anterior.
Camina al `walk_speed` % (40) de la velocidad máxima de Nav2 y corre al 100 % (nav2_msgs/SpeedLimit en
/speed_limit; al terminar quita el límite para no frenar a visitor_approach).

Publica /pepper/places (std_msgs/String con JSON, transient local: la lista de lugares; el bridge
de la app se la pasa a Gemini), /pepper/place_status (JSON: `phase` = going | arrived | failed |
canceled | unknown, `place`, `name`, `text`), /pepper/say al llegar o fallar, y
/pepper/places/markers (MarkerArray para RViz: flecha y nombre de cada lugar).
"""

import json
import math
import os
import unicodedata

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import SpeedLimit
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

ARTICLES = ("el ", "la ", "los ", "las ", "al ", "a ", "del ", "de ")


def norm(text):
    """Minúsculas, sin tildes ni signos ni artículos al principio."""
    t = unicodedata.normalize("NFD", text.lower())
    t = "".join(c for c in t if unicodedata.category(c) != "Mn")
    t = " ".join("".join(c if c.isalnum() else " " for c in t).split())
    while t.startswith(ARTICLES):
        t = t.split(" ", 1)[1] if " " in t else ""
    return t


def to(name):
    """«a» + nombre con contracción: a el cartel -> al cartel."""
    return "al " + name[3:] if name.startswith("el ") else "a " + name


def default_places_file():
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory("pepper_museum"), "maps", "museum_places.yaml")
    except Exception:
        return ""


class PlaceNavigator(Node):
    def __init__(self):
        super().__init__("place_navigator")
        self.places_file = self.declare_parameter("places_file", default_places_file()).value
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.walk_speed = self.declare_parameter("walk_speed", 40.0).value   # % de la máxima
        with open(self.places_file) as f:
            self.places = yaml.safe_load(f)["places"]
        self.lookup = {}
        for key, p in self.places.items():
            for alias in [key.replace("_", " "), p["name"], *p.get("aliases", [])]:
                self.lookup[norm(alias)] = key

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_places = self.create_publisher(String, "/pepper/places", latched)
        self.pub_markers = self.create_publisher(MarkerArray, "/pepper/places/markers", latched)
        self.pub_status = self.create_publisher(String, "/pepper/place_status", 10)
        self.pub_say = self.create_publisher(String, "/pepper/say", 10)
        self.pub_speed = self.create_publisher(SpeedLimit, "/speed_limit", 10)
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.create_subscription(String, "/pepper/go_to", self._go_to_cb, 10)
        self.goal = None      # (handle, key) del objetivo en curso
        self.seq = 0          # cada orden invalida las respuestas de las anteriores

        self.pub_places.publish(String(data=json.dumps(
            [{"key": k, "name": p["name"], "aliases": p.get("aliases", []), "about": p.get("about", ""),
              "pose": p["pose"]} for k, p in self.places.items()], ensure_ascii=False)))
        self.pub_markers.publish(self._markers())
        self.get_logger().info(f"{len(self.places)} lugares de {self.places_file}; "
                               "esperando órdenes en /pepper/go_to")

    # ---------- órdenes ----------
    def resolve(self, text):
        """Clave del lugar nombrado en `text`, o None: alias exacto o, si no, el más largo contenido."""
        t = norm(text)
        if t in self.lookup:
            return self.lookup[t]
        hits = [a for a in self.lookup if a and f" {a} " in f" {t} "]
        return self.lookup[max(hits, key=len)] if hits else None

    def _go_to_cb(self, msg):
        try:
            cmd = json.loads(msg.data)
        except ValueError:
            cmd = None
        if not isinstance(cmd, dict):
            cmd = {"place": msg.data}
        if cmd.get("cancel"):
            self.cancel()
            return
        key = self.resolve(str(cmd.get("place", "")))
        if key is None:
            self.status("unknown", f"No conozco el lugar «{cmd.get('place', '')}».", place=cmd.get("place"))
            return
        self.go(key, bool(cmd.get("run", False)))

    def go(self, key, run):
        self.cancel(quiet=True)
        self.seq += 1
        seq, p = self.seq, self.places[key]
        x, y, yaw_deg = p["pose"]
        yaw = math.radians(yaw_deg)
        g = NavigateToPose.Goal()
        g.pose = PoseStamped()
        g.pose.header.frame_id = self.map_frame
        g.pose.header.stamp = self.get_clock().now().to_msg()
        g.pose.pose.position.x, g.pose.pose.position.y = float(x), float(y)
        g.pose.pose.orientation.z, g.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        if not self.nav.wait_for_server(timeout_sec=3.0):
            self.finish(seq, key, "failed", "Nav2 no está disponible.")
            return
        self.set_speed(100.0 if run else self.walk_speed)
        self.get_logger().info(f"{'Corriendo' if run else 'Caminando'} a {key} ({x}, {y}, {yaw_deg}°)")
        self.status("going", f"{'Corro' if run else 'Voy'} {to(p['name'])}.", place=key, run=run)
        self.nav.send_goal_async(g).add_done_callback(lambda fut: self._accepted(fut, seq, key))

    def _accepted(self, fut, seq, key):
        handle = fut.result()
        if seq != self.seq:   # llegó otra orden mientras Nav2 aceptaba esta
            handle.cancel_goal_async()
            return
        if not handle.accepted:
            self.finish(seq, key, "failed", f"Nav2 rechazó ir {to(self.places[key]['name'])}.")
            return
        self.goal = (handle, key)
        handle.get_result_async().add_done_callback(lambda res: self._done(res, seq, key))

    def _done(self, res, seq, key):
        status = res.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.finish(seq, key, "arrived", f"Llegué {to(self.places[key]['name'])}.")
        elif status == GoalStatus.STATUS_CANCELED:
            self.finish(seq, key, "canceled", "")
        else:
            self.finish(seq, key, "failed", f"No pude llegar {to(self.places[key]['name'])}.")

    def cancel(self, quiet=False):
        """Cancela el objetivo en curso; sin `quiet` también uno que Nav2 aún no aceptó."""
        if not quiet:
            self.seq += 1
            self.set_speed(0.0)
        if self.goal is None:
            return
        handle, key = self.goal
        self.goal = None
        handle.cancel_goal_async()
        if not quiet:
            self.status("canceled", "Me detengo.", place=key)

    def finish(self, seq, key, phase, text):
        if seq != self.seq:   # respuesta de una orden ya reemplazada
            return
        self.goal = None
        self.set_speed(0.0)
        if phase == "canceled":
            return
        self.status(phase, text, place=key)
        self.get_logger().info(text)
        self.pub_say.publish(String(data=text))

    # ---------- salidas ----------
    def set_speed(self, percent):
        """Límite de velocidad de Nav2 en % de la máxima; 0 = sin límite."""
        msg = SpeedLimit(percentage=True, speed_limit=float(percent))
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_speed.publish(msg)

    def status(self, phase, text, place=None, **extra):
        name = self.places[place]["name"] if place in self.places else None
        self.pub_status.publish(String(data=json.dumps(
            {"phase": phase, "place": place, "name": name, "text": text, **extra}, ensure_ascii=False)))

    def _markers(self):
        arr = MarkerArray()
        for i, (key, p) in enumerate(self.places.items()):
            x, y, yaw_deg = p["pose"]
            yaw = math.radians(yaw_deg)
            for kind in (Marker.ARROW, Marker.TEXT_VIEW_FACING):
                m = Marker()
                m.header.frame_id = self.map_frame
                m.ns, m.id, m.type = "places", 2 * i + (kind == Marker.TEXT_VIEW_FACING), kind
                m.pose.position.x, m.pose.position.y = float(x), float(y)
                if kind == Marker.ARROW:
                    m.pose.orientation.z, m.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
                    m.scale.x, m.scale.y, m.scale.z = 0.6, 0.12, 0.12
                    m.color.r, m.color.g, m.color.b, m.color.a = 0.9, 0.45, 0.1, 1.0
                else:
                    m.pose.position.z = 0.6
                    m.pose.orientation.w = 1.0
                    m.text, m.scale.z = key.replace("_", " "), 0.35
                    m.color.r = m.color.g = m.color.b = m.color.a = 1.0
                arr.markers.append(m)
        return arr


def main():
    rclpy.init()
    node = PlaceNavigator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
