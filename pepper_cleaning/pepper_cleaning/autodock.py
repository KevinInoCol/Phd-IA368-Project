"""Autodocking de Pepper: la tarea del curso (Sección 2, autodocking.py) llevada al museo.

Máquina de estados:

  IDLE        nada que hacer (Pepper libre para limpiar, ir a lugares o atender visitantes)
  TO_PREDOCK  Nav2 (NavigateToPose) hasta delante de la base (`predock` de config/dock.yaml)
  SEEK        gira en el lugar hasta encontrar el haz IR (máx. 2 vueltas; si no, vuelve a TO_PREDOCK)
  APPROACH    sigue el haz: gira según el ángulo relativo y avanza más lento cuanto más fuerte la señal
  LOST        perdió la señal en la aproximación: gira hacia el último ángulo visto (`lost_timeout` s);
              si no la recupera, retrocede y vuelve a TO_PREDOCK
  CHARGING    contacto con la base: quieto, cargando hasta `full_battery` %
  UNDOCK      retrocede `undock_distance` m y queda libre (IDLE)
  FAILED      `max_attempts` intentos sin acoplarse

Entradas: /pepper/battery (BatteryState), /pepper/dock/signal y /relative_angle (Float32, sólo con
señal), /pepper/dock/contact (Bool), /pepper/dock_cmd (String: "dock" | "undock" | "cancel").
Salidas: /pepper/cmd_vel (sólo en SEEK/APPROACH/LOST/UNDOCK), /pepper/dock_status (String JSON
{state, battery, charging, attempts, text}), /pepper/say.

Con `auto_dock` (true), el docking se lanza solo cuando la batería baja del umbral

    umbral = max(low_battery, reserve + path_factor · drain_per_meter · d)

con d la distancia en línea recta de Pepper (su localización, TF map → base_footprint) a la base:
lejos de la base se vuelve antes, para llegar con `reserve` % y no con lo justo; con
`auto_undock` (true), si el docking lo lanzó la batería baja, al llegar a `full_battery` % sale de
la base (para que cleaner siga limpiando). Si se pidió con "dock", se queda en la base. Al empezar el docking cancela
cualquier ir-a-lugar (/pepper/go_to {cancel}).

    ros2 topic pub -w 1 --once /pepper/dock_cmd std_msgs/String "data: dock"
"""

import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Float32, String
from tf2_ros import Buffer, TransformListener

from .common import load_dock

SIGNAL_TIMEOUT = 0.6     # s sin mensajes de señal = señal perdida (la base sólo publica con señal)


class AutoDock(Node):
    def __init__(self):
        super().__init__("autodock")
        self.dock = load_dock(self.declare_parameter("config", "").value)
        self.low = self.declare_parameter("low_battery", 20.0).value
        self.full = self.declare_parameter("full_battery", 95.0).value
        self.auto_dock = self.declare_parameter("auto_dock", True).value
        self.auto_undock = self.declare_parameter("auto_undock", True).value
        self.v_far = self.declare_parameter("approach_speed", 0.15).value
        self.v_near = self.declare_parameter("final_speed", 0.05).value
        self.k_ang = self.declare_parameter("angle_gain", 1.2).value
        self.w_seek = self.declare_parameter("seek_speed", 0.4).value
        self.lost_timeout = self.declare_parameter("lost_timeout", 4.0).value
        self.undock_dist = self.declare_parameter("undock_distance", 0.8).value
        self.max_attempts = self.declare_parameter("max_attempts", 4).value
        self.reserve = self.declare_parameter("reserve", 10.0).value
        self.drain_per_m = self.declare_parameter("drain_per_meter", 0.6).value   # % por metro (battery_sim)
        self.path_factor = self.declare_parameter("path_factor", 1.4).value      # camino real / línea recta

        self.state = "IDLE"
        self.t_state = time.monotonic()
        self.battery = None
        self.charging = False
        self.contact = False
        self.signal = None
        self.angle = 0.0
        self.t_signal = 0.0
        self.attempts = 0
        self.goal = None
        self.seq = 0
        self.text = ""
        self.retry_at = None
        self.auto_started = False   # docking lanzado por batería baja
        self.t_sent = 0.0

        self.create_subscription(BatteryState, "/pepper/battery", self._battery_cb, 10)
        self.create_subscription(Float32, "/pepper/dock/signal", self._signal_cb, 10)
        self.create_subscription(Float32, "/pepper/dock/relative_angle", self._angle_cb, 10)
        self.create_subscription(Bool, "/pepper/dock/contact", self._contact_cb, 10)
        self.create_subscription(String, "/pepper/dock_cmd", self._cmd_cb, 10)
        self.pub_vel = self.create_publisher(Twist, "/pepper/cmd_vel", 10)
        self.pub_status = self.create_publisher(String, "/pepper/dock_status", 10)
        self.pub_say = self.create_publisher(String, "/pepper/say", 10)
        self.pub_go_to = self.create_publisher(String, "/pepper/go_to", 10)
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(0.05, self.step)
        self.create_timer(1.0, self.publish_status)
        self.get_logger().info(f"autodock: docking automático bajo {self.low:.0f} %, sale a {self.full:.0f} %")

    # ---------------------------------------------------------------- entradas
    def _battery_cb(self, msg):
        self.battery = msg.percentage * 100.0
        self.charging = msg.power_supply_status in (BatteryState.POWER_SUPPLY_STATUS_CHARGING,
                                                    BatteryState.POWER_SUPPLY_STATUS_FULL)

    def _signal_cb(self, msg):
        self.signal = msg.data
        self.t_signal = time.monotonic()

    def _angle_cb(self, msg):
        self.angle = msg.data

    def _contact_cb(self, msg):
        self.contact = msg.data

    def _cmd_cb(self, msg):
        cmd = msg.data.strip().lower()
        if cmd == "dock":
            self.start_docking("Voy a la base de carga.")
        elif cmd == "undock":
            if self.state == "CHARGING" or self.contact:
                self.set_state("UNDOCK", "Salgo de la base de carga.")
            else:
                self.get_logger().info("undock: no estoy en la base")
        elif cmd in ("cancel", "stop"):
            if self.state not in ("IDLE", "CHARGING"):
                self.cancel_nav()
                self.stop()
                self.set_state("IDLE", "Cancelo el docking.")

    # ---------------------------------------------------------------- ayuda
    def threshold(self):
        """% de batería con el que hay que volver: más lejos de la base, antes."""
        try:
            t = self.tf_buffer.lookup_transform("map", "base_footprint", rclpy.time.Time())
        except Exception:
            return self.low
        d = math.dist((t.transform.translation.x, t.transform.translation.y), self.dock["contact"])
        return max(self.low, self.reserve + self.path_factor * self.drain_per_m * d)

    def has_signal(self):
        return time.monotonic() - self.t_signal < SIGNAL_TIMEOUT

    def elapsed(self):
        return time.monotonic() - self.t_state

    def set_state(self, state, say=""):
        if state != self.state:
            self.get_logger().info(f"{self.state} -> {state}" + (f": {say}" if say else ""))
        self.state = state
        self.t_state = time.monotonic()
        self.text = say
        if say:
            self.pub_say.publish(String(data=say))
        self.publish_status()

    def publish_status(self):
        self.pub_status.publish(String(data=json.dumps({
            "state": self.state, "battery": None if self.battery is None else round(self.battery, 1),
            "charging": self.charging, "contact": self.contact, "attempts": self.attempts,
            "text": self.text}, ensure_ascii=False)))

    def cmd(self, vx=0.0, wz=0.0):
        t = Twist()
        t.linear.x, t.angular.z = float(vx), float(wz)
        self.pub_vel.publish(t)

    def stop(self):
        self.cmd(0.0, 0.0)

    def start_docking(self, say):
        if self.state not in ("IDLE", "FAILED", "UNDOCK"):
            return
        self.auto_started = False
        if self.contact:
            self.set_state("CHARGING", "Ya estoy en la base de carga.")
            return
        self.pub_go_to.publish(String(data=json.dumps({"cancel": True})))
        self.attempts = 0
        self.to_predock(say)

    def to_predock(self, say=""):
        self.attempts += 1
        if self.attempts > self.max_attempts:
            self.stop()
            self.set_state("FAILED", "No pude acoplarme a la base de carga.")
            return
        self.set_state("TO_PREDOCK", say)
        if not self.nav.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("Nav2 no responde: busco la señal desde aquí")
            self.set_state("SEEK")
            return
        x, y, yaw = self.dock["predock"]
        g = NavigateToPose.Goal()
        g.pose = PoseStamped()
        g.pose.header.frame_id = "map"
        g.pose.header.stamp = self.get_clock().now().to_msg()
        g.pose.pose.position.x, g.pose.pose.position.y = x, y
        g.pose.pose.orientation.z, g.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.seq += 1
        seq = self.seq
        self.t_sent = time.monotonic()
        self.nav.send_goal_async(g).add_done_callback(lambda f: self._accepted(f, seq))

    def _accepted(self, fut, seq):
        handle = fut.result()
        if not handle.accepted:
            # Nav2 todavía está con otro objetivo (p. ej. un tramo de limpieza que se está cancelando)
            if seq == self.seq and self.state == "TO_PREDOCK":
                self.get_logger().warn("Nav2 rechazó el objetivo; reintento")
                self.retry_at = time.monotonic() + 1.5
            return
        self.goal = handle
        handle.get_result_async().add_done_callback(lambda r: self._nav_done(r, seq))

    def _nav_done(self, res, seq):
        self.goal = None
        if seq != self.seq or self.state != "TO_PREDOCK":
            return
        status = res.result().status
        self.get_logger().info(f"Nav2 a la base: status {status}")
        if status == 6 and time.monotonic() - self.t_sent < 2.0:
            # abortado al instante: Nav2 seguía con otro objetivo (el tramo de limpieza que se está
            # cancelando, "another navigator is processing"); reintentar en 1 s
            self.retry_at = time.monotonic() + 1.0
            return
        # éxito o no, se busca el haz: Nav2 puede fallar a pocos cm del objetivo
        self.set_state("SEEK")

    def cancel_nav(self):
        self.seq += 1
        if self.goal is not None:
            self.goal.cancel_goal_async()
            self.goal = None

    # ---------------------------------------------------------------- bucle
    def step(self):
        b = self.battery
        if self.state == "IDLE":
            if self.auto_dock and b is not None and not self.contact and b < self.threshold():
                self.start_docking(f"Me queda {b:.0f} % de batería. Voy a cargarme.")
                self.auto_started = True
            elif self.contact and self.charging and b is not None and b < self.full:
                self.set_state("CHARGING")
            return

        if self.state in ("TO_PREDOCK",):
            if self.retry_at and time.monotonic() > self.retry_at:
                self.retry_at = None
                self.attempts -= 1
                self.to_predock()
                return
            if self.contact:
                self.cancel_nav()
                self.set_state("CHARGING", "Estoy en la base. Cargando.")
            return

        if self.state == "SEEK":
            if self.contact:
                self.stop()
                self.set_state("CHARGING", "Estoy en la base. Cargando.")
            elif self.has_signal():
                self.set_state("APPROACH")
            elif self.elapsed() > 2 * 2 * math.pi / self.w_seek:      # dos vueltas sin señal
                self.stop()
                self.to_predock()
            else:
                self.cmd(0.0, self.w_seek)
            return

        if self.state == "APPROACH":
            if self.contact:
                self.stop()
                self.set_state("CHARGING", "Estoy en la base. Cargando.")
            elif not self.has_signal():
                self.set_state("LOST")
            else:
                a = self.angle
                w = max(-0.6, min(0.6, self.k_ang * a))
                if abs(a) > 0.35:           # muy desalineado: girar en el lugar
                    v = 0.0
                else:
                    s = self.signal or 0.0  # 0 lejos .. 1 en la base
                    v = self.v_near if s > 0.85 else self.v_far
                    v *= max(0.2, 1.0 - abs(a) / 0.35)
                self.cmd(v, w)
            return

        if self.state == "LOST":
            if self.contact:
                self.stop()
                self.set_state("CHARGING", "Estoy en la base. Cargando.")
            elif self.has_signal():
                self.set_state("APPROACH")
            elif self.elapsed() < self.lost_timeout:
                self.cmd(0.0, math.copysign(0.35, self.angle or 1.0))
            elif self.elapsed() < self.lost_timeout + 2.0:
                self.cmd(-0.12, 0.0)        # retrocede un poco antes de replanificar
            else:
                self.stop()
                self.to_predock()
            return

        if self.state == "CHARGING":
            if not self.contact:
                # se separó de la base (empujón, teleoperación): reintentar
                self.set_state("IDLE", "Me separé de la base.")
            elif self.auto_undock and self.auto_started and b is not None and b >= self.full:
                self.set_state("UNDOCK", f"Batería al {b:.0f} %. Salgo de la base.")
            return

        if self.state == "UNDOCK":
            if self.elapsed() < self.undock_dist / 0.15:
                self.cmd(-0.15, 0.0)
            else:
                self.stop()
                self.set_state("IDLE")
            return


def main():
    rclpy.init()
    node = AutoDock()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        rclpy.try_shutdown()
