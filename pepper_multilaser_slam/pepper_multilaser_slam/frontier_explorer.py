#!/usr/bin/env python3
"""Exploración autónoma por fronteras: Pepper mapea solo, sin ruta ni ground truth.

    ros2 launch pepper_multilaser_slam explore.launch.py      # Nav2 (sin AMCL) + este nodo
    ros2 run pepper_multilaser_slam frontier_explorer          # sólo el nodo (Nav2 y SLAM aparte)

Lee el mapa que va construyendo slam_toolbox (/map) y su pose (TF map → base_footprint):

  1. Frontera = celda libre con alguna vecina desconocida, lejos (`clearance`) de los obstáculos.
     Las celdas de frontera se agrupan (8-conectividad); los grupos de menos de
     `min_frontier_size` celdas se ignoran (ruido de los rayos dispersos de Pepper).
  2. De cada grupo se toma como objetivo su celda más cercana al centroide. Coste =
     distancia − `size_weight`·tamaño [m]: cerca y grande primero.
  3. NavigateToPose al objetivo, mirando hacia la frontera. Cada `replan_period` s se comprueba
     si esa frontera ya se descubrió (no queda frontera a menos de 0.5 m): entonces se cancela y
     se elige otra. Un objetivo que falla o tarda más de `goal_timeout` s entra en una lista negra.
  4. Sin fronteras (o pasado `max_time`): vuelve al punto de partida (`return_home`), que cierra
     el lazo del SLAM, y sale con código 0, como route_driver.

Al empezar espera a que Nav2 esté activo, gira 360° (Spin: los costmaps ven alrededor) y avanza
0.5 m (DriveOnHeading; si no puede, retrocede con BackUp): slam_toolbox no añade scans al mapa
mientras el robot sólo gira, y con un único scan Karto deja casi todo desconocido (cada celda
necesita 2 lecturas), así que sin moverse no hay fronteras. Las fronteras a menos de
`min_goal_distance` se ignoran (los láseres no ven a menos de 0.3 m: queda un anillo desconocido
alrededor del robot). Si al principio sigue sin haber fronteras repite el empujón (giro de 90° y
0.5 m), hasta 6 veces. Antes de dar la exploración por terminada espera otra actualización del mapa.

El ground truth no se usa en ninguna decisión (sólo lo registra trajectory_recorder aparte).
"""

import math
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import BackUp, DriveOnHeading, NavigateToPose, Spin
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from scipy import ndimage
from tf2_ros import Buffer, TransformException, TransformListener


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__("frontier_explorer")
        self.min_size = int(self.declare_parameter("min_frontier_size", 12).value)
        self.clearance = float(self.declare_parameter("clearance", 0.35).value)
        self.size_weight = float(self.declare_parameter("size_weight", 0.02).value)
        self.goal_timeout = float(self.declare_parameter("goal_timeout", 60.0).value)
        self.replan_period = float(self.declare_parameter("replan_period", 2.0).value)
        self.blacklist_radius = float(self.declare_parameter("blacklist_radius", 0.6).value)
        self.max_time = float(self.declare_parameter("max_time", 1200.0).value)
        self.return_home = bool(self.declare_parameter("return_home", True).value)
        self.min_goal_dist = float(self.declare_parameter("min_goal_distance", 0.8).value)
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.base_frame = self.declare_parameter("base_frame", "base_footprint").value

        self.map = None
        self.create_subscription(
            OccupancyGrid, "/map", self.map_cb,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.spin_client = ActionClient(self, Spin, "spin")
        self.drive_client = ActionClient(self, DriveOnHeading, "drive_on_heading")
        self.backup_client = ActionClient(self, BackUp, "backup")
        self.bt_state = self.create_client(GetState, "/bt_navigator/get_state")
        self.blacklist = []
        self.home = None
        self.goals = 0
        self.failed = 0

    def map_cb(self, msg):
        self.map = msg

    # ---------- utilidades ----------
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def wait(self, sec):
        end = self.now() + sec
        while rclpy.ok() and self.now() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def robot_pose(self):
        try:
            t = self.tf.lookup_transform(self.map_frame, self.base_frame, Time())
        except TransformException:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw

    # ---------- fronteras ----------
    def frontier_mask(self):
        """Máscara de celdas de frontera del mapa actual (y su resolución y origen)."""
        m = self.map
        g = np.array(m.data, np.int16).reshape(m.info.height, m.info.width)
        res = m.info.resolution
        free, unknown, occ = g == 0, g < 0, g > 50
        near_unknown = ndimage.binary_dilation(unknown, structure=np.ones((3, 3), bool))
        # Lejos de los obstáculos: transformada de distancia a la celda ocupada más cercana
        dist = ndimage.distance_transform_edt(~occ) * res if occ.any() else np.full(g.shape, np.inf)
        return free & near_unknown & (dist > self.clearance), res, m.info.origin.position.x, m.info.origin.position.y

    def frontiers(self):
        """[(x, y, tamaño)] en el frame map, de los grupos de frontera válidos."""
        if self.map is None:
            return []
        front, res, ox, oy = self.frontier_mask()
        labels, n = ndimage.label(front, structure=np.ones((3, 3), bool))
        if n == 0:
            return []
        out = []
        sizes = ndimage.sum(front, labels, range(1, n + 1))
        coms = ndimage.center_of_mass(front, labels, range(1, n + 1))
        for k, (size, (cr, cc)) in enumerate(zip(sizes, coms), start=1):
            if size < self.min_size:
                continue
            rr, cc_ = np.nonzero(labels == k)
            i = int(np.argmin((rr - cr) ** 2 + (cc_ - cc) ** 2))
            out.append((ox + (cc_[i] + 0.5) * res, oy + (rr[i] + 0.5) * res, int(size)))
        return out

    def still_frontier(self, gx, gy, radius=0.6):
        """¿Quedan celdas de frontera a menos de `radius` del objetivo? (No el representante del
        grupo: en grupos grandes se desplaza con cada actualización del mapa.)"""
        front, res, ox, oy = self.frontier_mask()
        c, r, k = int((gx - ox) / res), int((gy - oy) / res), int(radius / res)
        return bool(front[max(r - k, 0):r + k + 1, max(c - k, 0):c + k + 1].any())

    def choose(self, robot):
        best = None
        for x, y, size in self.frontiers():
            if math.hypot(x - robot[0], y - robot[1]) < self.min_goal_dist:
                continue
            if any(math.hypot(x - bx, y - by) < self.blacklist_radius for bx, by in self.blacklist):
                continue
            cost = math.hypot(x - robot[0], y - robot[1]) - self.size_weight * size
            if best is None or cost < best[0]:
                best = (cost, x, y, size)
        return best

    # ---------- Nav2 ----------
    def go(self, x, y, yaw, check_frontier):
        """Navega a (x, y, yaw). 'ok', 'explored' (la frontera desapareció), 'failed' o 'timeout'."""
        g = NavigateToPose.Goal()
        g.pose = PoseStamped()
        g.pose.header.frame_id = self.map_frame
        g.pose.header.stamp = self.get_clock().now().to_msg()
        g.pose.pose.position.x, g.pose.pose.position.y = float(x), float(y)
        g.pose.pose.orientation.z, g.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        fut = self.nav.send_goal_async(g)
        while rclpy.ok() and not fut.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        handle = fut.result()
        if not handle.accepted:
            return "failed"
        res = handle.get_result_async()
        t0 = last = self.now()
        while rclpy.ok() and not res.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.now() - t0 > self.goal_timeout:
                handle.cancel_goal_async()
                self.wait(1.0)
                return "timeout"
            if check_frontier and self.now() - last > self.replan_period:
                last = self.now()
                if not self.still_frontier(x, y):
                    handle.cancel_goal_async()
                    self.wait(0.5)
                    return "explored"
        return "ok" if res.result().status == GoalStatus.STATUS_SUCCEEDED else "failed"

    def nav2_active(self):
        if not self.bt_state.service_is_ready():
            return False
        fut = self.bt_state.call_async(GetState.Request())
        while rclpy.ok() and not fut.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        return fut.result() is not None and fut.result().current_state.label == "active"

    def behavior(self, client, goal):
        """Ejecuta un behavior de Nav2 (Spin, DriveOnHeading, BackUp); True si terminó bien."""
        fut = client.send_goal_async(goal)
        while rclpy.ok() and not fut.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        if not fut.result().accepted:
            return False
        res = fut.result().get_result_async()
        while rclpy.ok() and not res.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        return res.result().status == GoalStatus.STATUS_SUCCEEDED

    def bootstrap(self, turn=2 * math.pi):
        """Gira `turn` y avanza (o retrocede) 0.5 m para que el SLAM empiece a llenar el mapa."""
        g = Spin.Goal()
        g.target_yaw = float(turn)
        g.time_allowance = Duration(sec=40)
        self.behavior(self.spin_client, g)
        for client, cls, speed in ((self.drive_client, DriveOnHeading, 0.15), (self.backup_client, BackUp, -0.15)):
            d = cls.Goal()
            d.target.x = 0.5 if speed > 0 else -0.5
            d.speed = abs(speed) if cls is BackUp else speed
            d.time_allowance = Duration(sec=15)
            if self.behavior(client, d):
                self.get_logger().info(f"Arranque: {'avanza' if speed > 0 else 'retrocede'} 0.5 m")
                break
        self.wait(3.0)   # que slam_toolbox publique el mapa

    # ---------- bucle ----------
    def run(self):
        self.get_logger().info("Esperando mapa, TF y Nav2...")
        while rclpy.ok() and (self.map is None or self.robot_pose() is None):
            rclpy.spin_once(self, timeout_sec=0.2)
        self.nav.wait_for_server()
        self.spin_client.wait_for_server()
        self.drive_client.wait_for_server()
        while rclpy.ok() and not self.nav2_active():
            self.wait(1.0)
        self.home = self.robot_pose()
        t_start = self.now()
        self.get_logger().info(f"Exploración desde ({self.home[0]:.2f}, {self.home[1]:.2f}): arranque")
        self.bootstrap()
        rechecked = False
        boots = 1
        while rclpy.ok() and self.now() - t_start < self.max_time:
            self.wait(0.5)   # que llegue el mapa actualizado
            robot = self.robot_pose()
            best = self.choose(robot) if robot else None
            if best is None and self.goals == 0 and boots < 6:
                # Todavía no hay mapa suficiente: otro empujón en otra dirección
                self.get_logger().info("Aún sin fronteras al empezar: giro 90° y avanzo")
                self.bootstrap(math.pi / 2)
                boots += 1
                continue
            if best is None:
                if rechecked:
                    break
                self.get_logger().info("Sin fronteras: espero otra actualización del mapa")
                self.wait(4.0)
                rechecked = True
                continue
            rechecked = False
            _, x, y, size = best
            yaw = math.atan2(y - robot[1], x - robot[0])
            self.goals += 1
            self.get_logger().info(f"Objetivo {self.goals}: frontera de {size} celdas en ({x:.2f}, {y:.2f}), "
                                   f"a {math.hypot(x - robot[0], y - robot[1]):.1f} m")
            r = self.go(x, y, yaw, check_frontier=True)
            if r in ("failed", "timeout"):
                self.failed += 1
                self.blacklist.append((x, y))
            self.get_logger().info(f"  → {r}")
        elapsed = self.now() - t_start
        why = "sin fronteras" if self.now() - t_start < self.max_time else "tiempo máximo"
        self.get_logger().info(f"Exploración terminada ({why}): {self.goals} objetivos, {self.failed} fallidos, "
                               f"{elapsed:.0f} s")
        if self.return_home and self.home:
            self.get_logger().info("Vuelta al inicio")
            self.get_logger().info(f"  → {self.go(*self.home, check_frontier=False)}")
        self.get_logger().info("Recorrido terminado")


def main():
    rclpy.init()
    node = FrontierExplorer()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
