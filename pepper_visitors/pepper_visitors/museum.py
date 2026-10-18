"""Lo que se sabe del museo y de la cámara de Pepper, compartido por la captura y el etiquetado.

Sin dependencias de ROS: el etiquetado corre en el entorno de yolo_ros (ultralytics).
"""

import math
import os
import xml.etree.ElementTree as ET

import numpy as np

# Clase de cada modelo de persona (las copias de un modelo son idénticas: la clase es el modelo)
CLASSES = {
    "citizen_extras_male_03": "male_03",
    "citizen_extras_female_02": "female_02",
    "citizen_extras_female_03_fixed": "female_03",
}
OTHER = "otro"  # actores que caminan (y cualquier persona que no sea un visitante estático)

# Cámara frontal (/pepper/camera/front/camera_info y TF base_footprint → CameraTop_optical_frame):
# 640×480, f = 585.76 px, a 0.048 m delante y 1.153 m sobre base_footprint, mirando al frente.
W, H = 640, 480
F = 585.756
CX, CY = 320.0, 240.0
CAM_X, CAM_Z = 0.048, 1.153
PERSON_H = 1.70


def people(world_sdf):
    """Personas estáticas del mundo: [(nombre, clase, x, y, yaw)]."""
    out = []
    for inc in ET.parse(world_sdf).getroot().find("world").findall("include"):
        cls = CLASSES.get(inc.findtext("uri").replace("model://", ""))
        if cls:
            p = [float(v) for v in inc.findtext("pose").split()]
            out.append((inc.findtext("name"), cls, p[0], p[1], p[5]))
    return out


def actor_paths(world_sdf):
    """Waypoints (x, y) de cada actor."""
    return [[tuple(float(v) for v in wp.findtext("pose").split()[:2]) for wp in a.iter("waypoint")]
            for a in ET.parse(world_sdf).getroot().find("world").findall("actor")]


class Walls:
    """Rejilla de paredes del museo a la altura de los láseres (pepper_museum/maps/museum_walls.txt)."""

    def __init__(self, path):
        with open(path) as f:
            self.res = float(f.readline().split()[2])
            self.cells = {tuple(int(v) for v in l.split()) for l in f}

    def _cell(self, x, y):
        return math.floor(x / self.res), math.floor(y / self.res)

    def clear(self, x, y, radius):
        cx, cy = self._cell(x, y)
        r = int(math.ceil(radius / self.res))
        return not any((cx + i, cy + j) in self.cells for i in range(-r, r + 1) for j in range(-r, r + 1)
                       if i * i + j * j <= r * r)

    def line_of_sight(self, x0, y0, x1, y1, margin=0.3):
        """Sin pared en el segmento, salvo los últimos `margin` m (la base de la persona)."""
        d = math.hypot(x1 - x0, y1 - y0)
        n = max(2, int(d / (self.res / 2)))
        for i in range(n):
            t = i / n
            if t * d > d - margin:
                break
            if self._cell(x0 + t * (x1 - x0), y0 + t * (y1 - y0)) in self.cells:
                return False
        return True


def project(px, py, rx, ry, ryaw):
    """Caja esperada (u_centro, v_arriba, v_abajo, distancia) de una persona en (px, py) vista por
    Pepper en (rx, ry, ryaw); None si queda detrás de la cámara."""
    dx, dy = px - rx, py - ry
    xf = math.cos(ryaw) * dx + math.sin(ryaw) * dy - CAM_X  # adelante
    yl = -math.sin(ryaw) * dx + math.cos(ryaw) * dy          # izquierda
    if xf < 0.3:
        return None
    u = CX - F * yl / xf
    return u, CY - F * (PERSON_H - CAM_Z) / xf, CY + F * CAM_Z / xf, xf


def museum_share(*parts):
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory("pepper_museum"), *parts)


def rng(seed):
    return np.random.default_rng(seed)
