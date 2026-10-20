#!/usr/bin/env python3
"""Recorrido repetible (oficina o museo) para comparar métodos de SLAM.

Guía a Pepper por una lista de puntos (coordenadas del mundo de Gazebo) usando el ground truth
(/pepper/odom_groundtruth), de modo que todos los experimentos siguen exactamente la misma ruta.
El SLAM no ve el ground truth: sólo recibe láseres y odometría ruidosa (/pepper/odom).

Pepper gira hasta mirar al siguiente punto y avanza de frente (el láser frontal mira hacia
donde va, como haría el robot real). La ruta da la vuelta a la sala inferior, cruza la puerta,
recorre la sala superior y vuelve al punto de partida para provocar un cierre de lazo.
Parámetro `route`: office (por defecto) | museum.

Al terminar publica velocidad cero y sale con código 0.
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

# Mundo simple_office_with_people.world: sala inferior y ∈ [-5, 5], sala superior y ∈ [5, 10],
# puerta en x ≈ 1.5. Pepper aparece en (-0.5, 1) mirando a +x.
OFFICE_ROUTE = [
    (-0.5, -1.5), (-1.2, -3.8), (-1.5, -1.2), (-3.4, -1.0), (-2.8, 2.8), (-1.0, 3.8),
    (1.5, 3.6), (1.5, 6.2), (-1.5, 7.0), (-3.6, 7.6), (0.5, 7.6), (3.0, 8.0), (3.2, 6.2),
    (1.5, 6.2), (1.5, 3.6), (2.8, 2.5), (2.8, -1.5), (0.5, -1.0), (-0.5, 1.0),
]

# Museo ISCAS (pepper_museum): Pepper aparece en (-1.5, 1). Sube a la sala de arriba a la
# izquierda, cruza por arriba del bloque central, baja por x = 7 (por donde caminan actores),
# rodea la vitrina baja de la derecha, vuelve por el pasillo inferior y sube por x = -1.7 hasta el
# inicio (cierre de lazo). ~58 m; a ≥ 0.75 m de las paredes (maps/museum_walls.txt) y ≥ 1 m de
# todas las personas estáticas de museum_with_people, museum_with_people_moving y museum_test_people.
MUSEUM_ROUTE = [
    (-2.0, 3.0), (-2.0, 7.0), (-5.5, 7.0), (-2.0, 7.0), (1.0, 4.3), (5.0, 4.3), (7.0, 3.8),
    (7.0, -6.5), (9.3, -6.7), (9.3, -9.0), (-1.7, -9.0), (-1.7, -0.5), (-1.5, 1.0),
]
ROUTES = {"office": OFFICE_ROUTE, "museum": MUSEUM_ROUTE}


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class RouteDriver(Node):
    def __init__(self):
        super().__init__("route_driver")
        self.v_max = self.declare_parameter("linear_speed", 0.3).value
        self.w_max = self.declare_parameter("angular_speed", 0.6).value
        self.tol = self.declare_parameter("goal_tolerance", 0.15).value
        self.route = list(ROUTES[self.declare_parameter("route", "office").value])
        self.i = 0
        self.pose = None
        self.done = False
        self.pub = self.create_publisher(Twist, "/pepper/cmd_vel", 1)
        self.create_subscription(Odometry, "/pepper/odom_groundtruth", self.gt_cb, 10)
        self.create_timer(0.05, self.step)
        self.get_logger().info(f"route_driver: {len(self.route)} puntos")

    def gt_cb(self, msg):
        p = msg.pose.pose
        yaw = math.atan2(2.0 * (p.orientation.w * p.orientation.z + p.orientation.x * p.orientation.y),
                         1.0 - 2.0 * (p.orientation.y ** 2 + p.orientation.z ** 2))
        self.pose = (p.position.x, p.position.y, yaw)

    def step(self):
        if self.pose is None or self.done:
            return
        x, y, yaw = self.pose
        gx, gy = self.route[self.i]
        dx, dy = gx - x, gy - y
        dist = math.hypot(dx, dy)
        if dist < self.tol:
            self.get_logger().info(f"punto {self.i + 1}/{len(self.route)} ({gx:.1f}, {gy:.1f})")
            self.i += 1
            if self.i == len(self.route):
                self.pub.publish(Twist())
                self.done = True
                self.get_logger().info("Recorrido terminado")
            return
        heading_err = wrap(math.atan2(dy, dx) - yaw)
        cmd = Twist()
        cmd.angular.z = max(-self.w_max, min(self.w_max, 1.5 * heading_err))
        # Avanza sólo cuando ya mira (más o menos) al objetivo; frena al llegar
        if abs(heading_err) < 0.3:
            cmd.linear.x = min(self.v_max, 0.6 * dist) * math.cos(heading_err)
        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = RouteDriver()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
