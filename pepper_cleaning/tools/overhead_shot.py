"""Foto cenital de una zona del museo (para la documentación): crea una cámara en Gazebo mirando
hacia abajo, toma una imagen y la borra.

    python3 tools/overhead_shot.py <salida.png> <x_map> <y_map> [--height 5] [--fov 1.0] [--yaw 0]

(x, y) en el frame map (= mundo + (1.5, −1)). La cámara se crea con el sistema de sensores que ya
tiene el mundo del museo (lo usan las cámaras de Pepper); la imagen llega por gz-transport.
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
<pose>{x} {y} {z} 0 1.5708 {yaw}</pose>
<link name="l"><sensor name="cam" type="camera">
<topic>{topic}</topic><update_rate>2</update_rate><always_on>true</always_on>
<camera><horizontal_fov>{fov}</horizontal_fov><image><width>1280</width><height>960</height></image>
<clip><near>0.1</near><far>50</far></clip></camera>
</sensor></link></model></sdf>"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("x", type=float)
    ap.add_argument("y", type=float)
    ap.add_argument("--height", type=float, default=5.0)
    ap.add_argument("--fov", type=float, default=1.0)
    ap.add_argument("--yaw", type=float, default=0.0, help="grados; 0 = +x del mapa hacia arriba de la imagen")
    a = ap.parse_args()
    name = f"overhead_cam_{int(time.time())}"
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
    ef.sdf = CAM_SDF.format(name=name, x=a.x - OFFSET[0], y=a.y - OFFSET[1], z=a.height,
                            yaw=math.radians(a.yaw), topic=topic, fov=a.fov)
    ok, rep = node.request("/world/default/create", ef, EntityFactory, Boolean, 5000)
    if not (ok and rep.data):
        print("no pude crear la cámara")
        return 1
    try:
        if not got.wait(30):
            print("sin imagen de la cámara")
            return 1
        time.sleep(1.0)            # una imagen más: la primera puede salir sin todas las mallas
        got.clear()
        got.wait(10)
        # la cámara mira hacia abajo con +x del modelo hacia arriba de la imagen
        Image.fromarray(np.ascontiguousarray(frame["img"])).save(a.out)
        print(a.out)
    finally:
        e = Entity()
        e.name, e.type = name, Entity.MODEL
        node.request("/world/default/remove", e, Entity, Boolean, 2000)
    return 0


if __name__ == "__main__":
    sys.exit(main())
