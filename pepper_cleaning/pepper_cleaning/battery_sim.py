"""Batería simulada de Pepper.

Se descarga con el tiempo y más rápido cuando la base se mueve; se carga cuando Pepper toca los
contactos de la base (/pepper/dock/contact, de dock_station):

    consumo [%/s] = drain_idle + drain_per_mps · |v|        (v de /pepper/odom_groundtruth)
    carga   [%/s] = charge_rate                             (mientras hay contacto)

Publica /pepper/battery (sensor_msgs/BatteryState, `percentage` en 0..1, `power_supply_status`
CHARGING / DISCHARGING / FULL) a 2 Hz. Para pruebas, /pepper/battery/set (std_msgs/Float32, en %)
fija el nivel:

    ros2 topic pub -w 1 --once /pepper/battery/set std_msgs/Float32 "data: 18.0"

Los valores por defecto son para la demo (en el robot real la batería dura horas): con la base en
movimiento a ~0.3 m/s se gasta ~1 % cada 5 s.
"""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Float32


class BatterySim(Node):
    def __init__(self):
        super().__init__("battery_sim")
        self.level = self.declare_parameter("initial", 100.0).value
        self.drain_idle = self.declare_parameter("drain_idle", 0.02).value
        self.drain_mps = self.declare_parameter("drain_per_mps", 0.6).value
        self.charge_rate = self.declare_parameter("charge_rate", 2.0).value
        self.contact = False
        self.speed = 0.0
        self.last_xy = None
        self.last_t = None
        self.create_subscription(Odometry, "/pepper/odom_groundtruth", self._gt_cb, 10)
        self.create_subscription(Bool, "/pepper/dock/contact", self._contact_cb, 10)
        self.create_subscription(Float32, "/pepper/battery/set", self._set_cb, 10)
        self.pub = self.create_publisher(BatteryState, "/pepper/battery", 10)
        self.prev = self.get_clock().now()
        self.create_timer(0.5, self.step)
        self.get_logger().info(f"battery_sim: {self.level:.0f} %")

    def _gt_cb(self, msg):
        p = msg.pose.pose.position
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_xy is not None and t > self.last_t:
            v = math.hypot(p.x - self.last_xy[0], p.y - self.last_xy[1]) / (t - self.last_t)
            self.speed = 0.8 * self.speed + 0.2 * v
        self.last_xy, self.last_t = (p.x, p.y), t

    def _contact_cb(self, msg):
        self.contact = msg.data

    def _set_cb(self, msg):
        self.level = min(100.0, max(0.0, msg.data))
        self.get_logger().info(f"batería fijada en {self.level:.0f} %")

    def step(self):
        now = self.get_clock().now()
        dt = (now - self.prev).nanoseconds * 1e-9
        self.prev = now
        if dt <= 0 or dt > 5:            # reloj simulado reiniciado o en pausa
            dt = 0.0
        if self.contact:
            self.level = min(100.0, self.level + self.charge_rate * dt)
        else:
            self.level = max(0.0, self.level - (self.drain_idle + self.drain_mps * self.speed) * dt)
        msg = BatteryState()
        msg.header.stamp = now.to_msg()
        msg.percentage = self.level / 100.0
        msg.present = True
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        if self.contact:
            msg.power_supply_status = (BatteryState.POWER_SUPPLY_STATUS_FULL if self.level >= 99.9
                                       else BatteryState.POWER_SUPPLY_STATUS_CHARGING)
        else:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        msg.voltage = 21.0 + 4.2 * self.level / 100.0      # Pepper: batería de litio de 25.2 V
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = BatterySim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
