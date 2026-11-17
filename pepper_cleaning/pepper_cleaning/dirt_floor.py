"""Piso sucio que se limpia al pasar por encima (como la escena dirty_evaluation3 del curso).

Cada zona de config/dirt.yaml se llena de baldosas sucias de `tile` m sobre las celdas libres del
mapa. En Gazebo cada zona es UNA malla (OBJ, 2 mm sobre el piso, sin colisión) con una baldosa por
celda sucia y la textura atlas de tools/make_dirt_textures.py: con un modelo por baldosa (~450) el
renderizado de los sensores bajaba el factor de tiempo real de ~0.93 a ~0.35. Cuando el cepillo
está encendido (/pepper/brush, std_msgs/Bool) y el centro de una baldosa queda a menos de
`brush_radius` del centro de Pepper (/pepper/odom_groundtruth: es la física del cepillo, no la
localización), la baldosa se marca limpia; a lo sumo una vez por segundo se regenera la malla de
cada zona que cambió (archivo nuevo, porque Gazebo cachea las mallas por nombre) y se reemplaza el
modelo: aparece el piso limpio.

Publica:
    /pepper/dirt/status   std_msgs/String (JSON) {total, cleaned, percent, zones: {zona: {total, cleaned, percent}}}
    /pepper/dirt/grid     nav_msgs/OccupancyGrid (map, resolución `tile`): 100 sucio, 0 limpio, −1 sin suciedad
    /pepper/dirt/tiles    std_msgs/String (JSON, latched) las baldosas sucias de cada zona [[x, y], ...]
                          (para el planificador de cobertura)
Servicio-tópico /pepper/dirt/reset (std_msgs/Empty): vuelve a ensuciar todo.

Parámetros: `world` (para excluir a sus personas estáticas), `map` (yaml del mapa), `config`,
`brush_always` (false: limpia sólo con /pepper/brush = true).
"""

import json
import math
import os
import shutil
import tempfile
import threading

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.empty_pb2 import Empty as Empty_gz
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.scene_pb2 import Scene
from gz.transport13 import Node as GzNode
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, Empty, String

from .common import FreeMap, default_map, load_dock

N_TEX = 4   # el atlas tiene N_TEX × N_TEX baldosas (tools/make_dirt_textures.py)

# Material: el del ground_plane del museo (ambient/specular 0.8) con metalness 0 y roughness 0.5
# (el PBR de SDF trae metalness 0.5 por defecto y oscurece). La textura va en sRGB y ya trae el
# difuso 0.8 del piso mezclado (tools/make_dirt_textures.py), por eso aquí el difuso es 1: lo limpio
# de la baldosa se ve igual que el piso (196/255 en la cámara inferior, medido).
ZONE_SDF = """<?xml version="1.0"?>
<sdf version="1.9"><model name="{name}"><static>true</static>
<pose>0 0 0.002 0 0 0</pose>
<link name="l"><visual name="v"><cast_shadows>false</cast_shadows>
<geometry><mesh><uri>file://{mesh}</uri></mesh></geometry>
<material><ambient>0.8 0.8 0.8 1</ambient><diffuse>1 1 1 1</diffuse><specular>0.8 0.8 0.8 1</specular>
<pbr><metal><albedo_map>{tex}</albedo_map><metalness>0.0</metalness><roughness>0.5</roughness></metal></pbr>
</material></visual></link></model></sdf>"""


def static_people(world):
    """(x, y) en el mundo de las personas estáticas del .sdf del mundo."""
    from pepper_visitors.museum import people
    path = os.path.join(get_package_share_directory("pepper_museum"), "worlds", world + ".sdf")
    return [(x, y) for _, _, x, y, _ in people(path)]


class DirtFloor(Node):
    def __init__(self):
        super().__init__("dirt_floor")
        share = get_package_share_directory("pepper_cleaning")
        cfg_path = self.declare_parameter("config", os.path.join(share, "config", "dirt.yaml")).value
        map_path = self.declare_parameter("map", default_map()).value
        self.world = self.declare_parameter("world", "museum_test_people").value
        self.brush_always = self.declare_parameter("brush_always", False).value
        with open(cfg_path) as f:
            self.cfg = yaml.safe_load(f)
        self.dock = load_dock()
        self.tile = self.cfg["tile"]
        self.brush_r = self.cfg["brush_radius"]
        self.tex_dir = os.path.join(share, "models", "dirt", "materials", "textures")
        self.tiles = self.make_tiles(FreeMap(map_path))   # (i, j) -> {x, y, zone, dirty}
        self.zones = {z: v.get("name", z) for z, v in self.cfg["zones"].items()}
        self.pose = None
        self.brush = self.brush_always
        self.lock = threading.Lock()
        self.gz = GzNode()
        self.mesh_dir = tempfile.mkdtemp(prefix="pepper_dirt_")
        self.version = 0
        self.models = {}            # zona -> nombre del modelo en Gazebo
        self.changed = set(self.zones)

        self.create_subscription(Odometry, "/pepper/odom_groundtruth", self._gt_cb, 10)
        self.create_subscription(Bool, "/pepper/brush", self._brush_cb, 10)
        self.create_subscription(Empty, "/pepper/dirt/reset", lambda _: self.reset(), 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_status = self.create_publisher(String, "/pepper/dirt/status", latched)
        self.pub_grid = self.create_publisher(OccupancyGrid, "/pepper/dirt/grid", latched)
        self.pub_tiles = self.create_publisher(String, "/pepper/dirt/tiles", latched)
        self.spawned = False
        self.cleared_old = False
        self.create_timer(2.0, self._spawn_once)
        self.create_timer(1.0, self.refresh)
        self.create_timer(0.1, self.step)
        self.create_timer(1.0, self.publish)
        self.get_logger().info(f"dirt_floor: {len(self.tiles)} baldosas sucias en {len(self.zones)} zonas "
                               f"({len(self.tiles) * self.tile ** 2:.1f} m²)")

    # ------------------------------------------------------------ baldosas
    def make_tiles(self, fmap):
        ox, oy = self.dock["offset"]
        people = [(x + ox, y + oy) for x, y in static_people(self.world)]
        self.get_logger().info(f"mundo {self.world}: {len(people)} personas estáticas excluidas del piso sucio")
        dx, dy = self.dock["contact"]
        pm, dm, wm = self.cfg["person_margin"], self.cfg["dock_margin"], self.cfg["wall_margin"]
        s = self.tile
        tiles = {}
        for zone, z in self.cfg["zones"].items():
            x0, x1, y0, y1 = z["rect"]
            for i in range(math.floor(x0 / s), math.ceil(x1 / s)):
                for j in range(math.floor(y0 / s), math.ceil(y1 / s)):
                    x, y = (i + 0.5) * s, (j + 0.5) * s
                    if (i, j) in tiles or not fmap.clear(x, y, s / 2 + wm):
                        continue
                    if any(math.hypot(x - px, y - py) < pm for px, py in people):
                        continue
                    if math.hypot(x - dx, y - dy) < dm:
                        continue
                    tiles[(i, j)] = {"x": x, "y": y, "zone": zone, "dirty": True}
        return tiles

    def _spawn_once(self):
        if self.spawned:
            return
        if not self.cleared_old:
            self.cleared_old = self.remove_old()
            return                    # el borrado se aplica en el siguiente paso de Gazebo
        self.spawned = True
        self.refresh()
        self.get_logger().info(f"piso sucio creado en Gazebo ({len(self.zones)} mallas)")

    def remove_old(self):
        """Borra el piso sucio de una corrida anterior del nodo (misma simulación)."""
        ok, scene = self.gz.request("/world/default/scene/info", Empty_gz(), Empty_gz, Scene, 5000)
        if not ok:
            return False
        old = [m.name for m in scene.model if m.name.startswith("dirt_")]
        for name in old:
            self.remove(name)
        if old:
            self.get_logger().info(f"borrados {len(old)} modelos de suciedad de una corrida anterior")
        return True

    def write_mesh(self, zone, path):
        """OBJ con una baldosa (2 triángulos) por celda sucia de la zona; UV = trozo del atlas.
        Coordenadas del mundo (el modelo va en el origen). Devuelve el número de baldosas."""
        ox, oy = self.dock["offset"]
        h, q = self.tile / 2, 1.0 / N_TEX
        v, vt, f = [], [], []
        for (i, j), t in self.tiles.items():
            if t["zone"] != zone or not t["dirty"]:
                continue
            x, y = t["x"] - ox, t["y"] - oy
            u0, v0 = (i % N_TEX) * q, (j % N_TEX) * q
            k = len(v)
            v += [(x - h, y - h), (x + h, y - h), (x + h, y + h), (x - h, y + h)]
            vt += [(u0, v0), (u0 + q, v0), (u0 + q, v0 + q), (u0, v0 + q)]
            f += [(k + 1, k + 2, k + 3), (k + 1, k + 3, k + 4)]
        if not f:
            return 0
        with open(path, "w") as out:
            out.write("".join(f"v {a:.4f} {b:.4f} 0\n" for a, b in v))
            out.write("".join(f"vt {a:.4f} {b:.4f}\n" for a, b in vt))
            out.write("vn 0 0 1\n")
            out.write("".join(f"f {a}/{a}/1 {b}/{b}/1 {c}/{c}/1\n" for a, b, c in f))
        return len(f) // 2

    def refresh(self):
        """Reemplaza en Gazebo la malla de cada zona que cambió (primero crea la nueva, después
        borra la vieja, para que no parpadee)."""
        if not self.spawned:
            return
        with self.lock:
            zones, self.changed = self.changed, set()
        tex = os.path.join(self.tex_dir, "dirt_atlas.png")
        for zone in zones:
            self.version += 1
            name = f"dirt_{zone}_{self.version}"
            mesh = os.path.join(self.mesh_dir, name + ".obj")
            n = self.write_mesh(zone, mesh)
            old = self.models.pop(zone, None)
            if n:
                ef = EntityFactory()
                ef.sdf = ZONE_SDF.format(name=name, mesh=mesh, tex=tex)
                ok, rep = self.gz.request("/world/default/create", ef, EntityFactory, Boolean, 5000)
                if ok and rep.data:
                    self.models[zone] = name
                else:
                    self.get_logger().warn(f"no pude crear {name}; reintento")
                    with self.lock:
                        self.changed.add(zone)
                    if old:
                        self.models[zone] = old
                    continue
            if old and not self.remove(old):
                self.get_logger().warn(f"no pude borrar {old}")

    def remove(self, name):
        e = Entity()
        e.name = name
        e.type = Entity.MODEL
        ok, rep = self.gz.request("/world/default/remove", e, Entity, Boolean, 2000)
        return ok and rep.data

    def reset(self):
        with self.lock:
            n = 0
            for t in self.tiles.values():
                if not t["dirty"]:
                    t["dirty"] = True
                    n += 1
                    self.changed.add(t["zone"])
        self.get_logger().info(f"piso ensuciado de nuevo ({n} baldosas)")
        self.publish()

    def destroy_node(self):
        shutil.rmtree(self.mesh_dir, ignore_errors=True)
        super().destroy_node()

    # ------------------------------------------------------------ entradas
    def _gt_cb(self, msg):
        ox, oy = self.dock["offset"]
        p = msg.pose.pose.position
        self.pose = (p.x + ox, p.y + oy)

    def _brush_cb(self, msg):
        self.brush = msg.data or self.brush_always

    # ------------------------------------------------------------ bucle
    def step(self):
        if self.pose is None or not self.brush or not self.spawned:
            return
        x, y = self.pose
        s, r = self.tile, self.brush_r
        cleaned = False
        with self.lock:
            for i in range(math.floor((x - r) / s), math.floor((x + r) / s) + 1):
                for j in range(math.floor((y - r) / s), math.floor((y + r) / s) + 1):
                    t = self.tiles.get((i, j))
                    if t and t["dirty"] and math.hypot(t["x"] - x, t["y"] - y) <= r:
                        t["dirty"] = False
                        self.changed.add(t["zone"])
                        cleaned = True
        if cleaned:
            self.publish()

    def publish(self):
        zones = {}
        for t in self.tiles.values():
            z = zones.setdefault(t["zone"], {"name": self.zones[t["zone"]], "total": 0, "cleaned": 0})
            z["total"] += 1
            z["cleaned"] += not t["dirty"]
        for z in zones.values():
            z["percent"] = round(100.0 * z["cleaned"] / max(1, z["total"]), 1)
        total = len(self.tiles)
        cleaned = sum(not t["dirty"] for t in self.tiles.values())
        self.pub_status.publish(String(data=json.dumps({
            "total": total, "cleaned": cleaned, "percent": round(100.0 * cleaned / max(1, total), 1),
            "brush": self.brush, "zones": zones}, ensure_ascii=False)))
        self.pub_tiles.publish(String(data=json.dumps({
            z: [[round(t["x"], 3), round(t["y"], 3)] for t in self.tiles.values() if t["zone"] == z and t["dirty"]]
            for z in self.zones})))
        self.pub_grid.publish(self.grid())

    def grid(self):
        s = self.tile
        ii = [i for i, _ in self.tiles]
        jj = [j for _, j in self.tiles]
        i0, j0 = min(ii), min(jj)
        w, h = max(ii) - i0 + 1, max(jj) - j0 + 1
        data = [-1] * (w * h)
        for (i, j), t in self.tiles.items():
            data[(j - j0) * w + (i - i0)] = 100 if t["dirty"] else 0
        g = OccupancyGrid()
        g.header.frame_id = "map"
        g.header.stamp = self.get_clock().now().to_msg()
        g.info.resolution = float(s)
        g.info.width, g.info.height = w, h
        g.info.origin.position.x, g.info.origin.position.y = i0 * s, j0 * s
        g.info.origin.position.z = 0.01
        g.info.origin.orientation.w = 1.0
        g.data = data
        return g


def main():
    rclpy.init()
    node = DirtFloor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
