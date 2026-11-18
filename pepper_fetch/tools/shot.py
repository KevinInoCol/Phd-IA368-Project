"""Foto de la escena desde una cámara puesta en Gazebo (para depurar y documentar el agarre): crea
la cámara en (cx, cy, cz) mirando al punto (tx, ty, tz), toma una imagen y la borra.

    python3 tools/shot.py <salida.png> cx cy cz tx ty tz [--fov 0.8] [--world]

Coordenadas en el frame map (= mundo + (1.5, −1)); con --world, en el mundo de Gazebo.
"""

import argparse
import math
import sys
import threading
import time

import numpy as np
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.image_pb2 import Image as GzImage
from gz.transport13 import Node
from PIL import Image

OFFSET = (1.5, -1.0)
CAM_SDF = """<?xml version="1.0"?>
<sdf version="1.9"><model name="{name}"><static>true</static>
<pose>{x} {y} {z} 0 {pitch} {yaw}</pose>
<link name="l"><sensor name="cam" type="camera">
<topic>{topic}</topic><update_rate>2</update_rate><always_on>true</always_on>
<camera><horizontal_fov>{fov}</horizontal_fov><image><width>1280</width><height>960</height></image>
<clip><near>0.05</near><far>50</far></clip></camera>
</sensor></link></model></sdf>"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    for k in ("cx", "cy", "cz", "tx", "ty", "tz"):
        ap.add_argument(k, type=float)
    ap.add_argument("--fov", type=float, default=0.8)
    ap.add_argument("--world", action="store_true")
    a = ap.parse_args()
    ox, oy = (0.0, 0.0) if a.world else OFFSET
    dx, dy, dz = a.tx - a.cx, a.ty - a.cy, a.tz - a.cz
    yaw, pitch = math.atan2(dy, dx), -math.atan2(dz, math.hypot(dx, dy))
    name = f"shot_cam_{int(time.time() * 1000)}"
    topic = f"/{name}/image"
    node = Node()
    got = threading.Event()
    frame = {}

    def cb(msg):
        if not got.is_set():
            frame["img"] = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, -1)[..., :3]
            got.set()

    node.subscribe(GzImage, topic, cb)
    ef = EntityFactory()
    ef.sdf = CAM_SDF.format(name=name, x=a.cx - ox, y=a.cy - oy, z=a.cz, pitch=pitch, yaw=yaw,
                            topic=topic, fov=a.fov)
    ok, rep = node.request("/world/default/create", ef, EntityFactory, Boolean, 5000)
    if not (ok and rep.data):
        print("no pude crear la cámara")
        return 1
    try:
        if not got.wait(30):
            print("sin imagen de la cámara")
            return 1
        time.sleep(1.0)
        got.clear()
        got.wait(10)
        Image.fromarray(np.ascontiguousarray(frame["img"])).save(a.out)
        print(a.out)
    finally:
        e = Entity()
        e.name, e.type = name, Entity.MODEL
        node.request("/world/default/remove", e, Entity, Boolean, 2000)
    return 0


if __name__ == "__main__":
    sys.exit(main())
