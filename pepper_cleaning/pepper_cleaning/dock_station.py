"""Base de carga simulada: emula el haz infrarrojo y los contactos, como la base de la escena del curso.

El receptor IR está en el frente de la base de Pepper (`receiver_offset` m delante del centro). La
base emite un abanico de `beam_half_angle` grados y `beam_range` m hacia `facing_deg` (config/dock.yaml).
Cuando el receptor está dentro del abanico y mira hacia la base (±`receiver_fov`):

    /pepper/dock/signal          std_msgs/Float32   intensidad = 1 − d / beam_range   (sólo si hay señal)
    /pepper/dock/relative_angle  std_msgs/Float32   rumbo de la base visto desde Pepper [rad], 0 = al frente

Igual que el `charging_base_node` del curso, **sólo publica cuando hay señal**: si no llegan mensajes,
se perdió. Además, siempre a 10 Hz:

    /pepper/dock/contact         std_msgs/Bool      el frente de Pepper toca los contactos (d ≤ contact_tol)
    /pepper/dock/markers         visualization_msgs/MarkerArray   base y haz en RViz (frame map)

La posición del receptor sale de /pepper/odom_groundtruth (mundo): es la física del sensor, no la
localización del robot (que usa AMCL).
"""

import math

import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, ColorRGBA, Float32
from visualization_msgs.msg import Marker, MarkerArray

from .common import load_dock, wrap, yaw_of


class DockStation(Node):
    def __init__(self):
        super().__init__("dock_station")
        self.dock = load_dock(self.declare_parameter("config", "").value)
        self.range = self.declare_parameter("beam_range", 2.5).value
        self.half = math.radians(self.declare_parameter("beam_half_angle", 20.0).value)
        self.fov = math.radians(self.declare_parameter("receiver_fov", 90.0).value)
        self.offset = self.declare_parameter("receiver_offset", 0.24).value
        self.contact_tol = self.declare_parameter("contact_tol", 0.07).value
        self.pose = None
        self.contact = False
        self.create_subscription(Odometry, "/pepper/odom_groundtruth", self._gt_cb, 10)
        self.pub_signal = self.create_publisher(Float32, "/pepper/dock/signal", 10)
        self.pub_angle = self.create_publisher(Float32, "/pepper/dock/relative_angle", 10)
        self.pub_contact = self.create_publisher(Bool, "/pepper/dock/contact", 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_markers = self.create_publisher(MarkerArray, "/pepper/dock/markers", latched)
        self.create_timer(0.1, self.step)
        self._markers()
        cx, cy = self.dock["contact"]
        self.get_logger().info(f"dock_station: base en ({cx:.2f}, {cy:.2f}) del mapa, "
                               f"haz {math.degrees(self.half):.0f}° × {self.range} m")

    def _gt_cb(self, msg):
        ox, oy = self.dock["offset"]
        p = msg.pose.pose
        self.pose = (p.position.x + ox, p.position.y + oy, yaw_of(p.orientation))

    def sense(self):
        """(intensidad o None, rumbo de la base desde Pepper, distancia receptor-contacto)."""
        x, y, th = self.pose
        rx, ry = x + self.offset * math.cos(th), y + self.offset * math.sin(th)
        cx, cy = self.dock["contact"]
        dx, dy = rx - cx, ry - cy
        d = math.hypot(dx, dy)
        bearing = wrap(math.atan2(-dy, -dx) - th)          # hacia la base, en el frame de Pepper
        in_beam = d <= self.range and (d < 0.05 or abs(wrap(math.atan2(dy, dx) - self.dock["facing"])) <= self.half)
        if in_beam and abs(bearing) <= self.fov:
            return max(0.0, 1.0 - d / self.range), bearing, d
        return None, bearing, d

    def step(self):
        if self.pose is None:
            return
        strength, bearing, d = self.sense()
        if strength is not None:
            self.pub_signal.publish(Float32(data=float(strength)))
            self.pub_angle.publish(Float32(data=float(bearing)))
        contact = d <= self.contact_tol and abs(bearing) < math.radians(35)
        if contact != self.contact:
            self.get_logger().info("contacto con la base" if contact else "fuera de la base")
            self.contact = contact
        self.pub_contact.publish(Bool(data=contact))

    def _markers(self):
        cx, cy = self.dock["contact"]
        f = self.dock["facing"]
        arr = MarkerArray()
        beam = Marker(ns="dock", id=0, type=Marker.TRIANGLE_LIST, action=Marker.ADD)
        beam.header.frame_id = "map"
        beam.scale.x = beam.scale.y = beam.scale.z = 1.0
        beam.color = ColorRGBA(r=1.0, g=0.2, b=0.2, a=0.25)
        beam.pose.orientation.w = 1.0
        n = 12
        for i in range(n):
            a0 = f - self.half + 2 * self.half * i / n
            a1 = f - self.half + 2 * self.half * (i + 1) / n
            beam.points += [Point(x=cx, y=cy, z=0.02),
                            Point(x=cx + self.range * math.cos(a0), y=cy + self.range * math.sin(a0), z=0.02),
                            Point(x=cx + self.range * math.cos(a1), y=cy + self.range * math.sin(a1), z=0.02)]
        arr.markers.append(beam)
        box = Marker(ns="dock", id=1, type=Marker.CUBE, action=Marker.ADD)
        box.header.frame_id = "map"
        box.pose.position.x = cx - 0.05 * math.cos(f)
        box.pose.position.y = cy - 0.05 * math.sin(f)
        box.pose.position.z = 0.16
        box.pose.orientation.z, box.pose.orientation.w = math.sin(f / 2), math.cos(f / 2)
        box.scale.x, box.scale.y, box.scale.z = 0.10, 0.50, 0.32
        box.color = ColorRGBA(r=0.1, g=0.6, b=1.0, a=0.9)
        arr.markers.append(box)
        txt = Marker(ns="dock", id=2, type=Marker.TEXT_VIEW_FACING, action=Marker.ADD)
        txt.header.frame_id = "map"
        txt.pose.position.x, txt.pose.position.y, txt.pose.position.z = cx, cy, 0.6
        txt.pose.orientation.w = 1.0
        txt.scale.z = 0.25
        txt.color = ColorRGBA(r=0.1, g=0.6, b=1.0, a=1.0)
        txt.text = "base de carga"
        arr.markers.append(txt)
        self.pub_markers.publish(arr)


def main():
    rclpy.init()
    node = DockStation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
