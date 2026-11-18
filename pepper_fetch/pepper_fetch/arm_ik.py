"""Cinemática del brazo derecho de Pepper (con la inclinación de la cadera) para coger un objeto.

El brazo de Pepper tiene 5 articulaciones (RShoulderPitch, RShoulderRoll, RElbowYaw, RElbowRoll,
RWristYaw) y el IK de MoveIt (KDL) sólo resuelve la posición de la muñeca (`position_only_ik`): no
deja elegir cómo queda la mano. Aquí se resuelve con mínimos cuadrados (scipy) sobre la cadena del
URDF (base_footprint → muñeca), con HipPitch como sexta variable (Pepper se inclina hacia delante
para llegar a la mesa), pidiendo además:
  - que el objeto quede pegado a la palma (punto `palm` de la muñeca, normal `palm_axis`),
  - que la palma mire de lado (normal horizontal: agarre lateral, como lo hace el Pepper real),
  - que el antebrazo vaya más o menos horizontal y hacia delante,
  - inclinarse lo menos posible.
La trayectoria (sin chocar con la mesa) la planifica luego MoveIt con estas articulaciones como
objetivo.
"""

import math
import xml.etree.ElementTree as ET

import numpy as np
from scipy.optimize import least_squares

ARM = ["RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw"]
VARS = ["HipPitch"] + ARM
# Mano derecha en el frame r_wrist (malla RWristYaw: x 0..0.093 m hacia los dedos): el centro de la
# palma y hacia dónde mira. En Gazebo la mano es una pieza (el modelo con gazebo:=true no lleva dedos).
# El sensor táctil "Hand/Touch/Back" (dorso de la mano) está en +z de r_wrist: la palma mira a −z.
PALM = np.array([0.060, 0.0, 0.0])
PALM_AXIS = np.array([0.0, 0.0, -1.0])
PALM_HALF = 0.025
ELBOW_OUT = 0.21                            # |y| mínimo del codo (el torso mide ~0.28 m de ancho)                           # de la línea media de la mano a la palma


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _axis_angle(axis, a):
    x, y, z = axis
    c, s, t = math.cos(a), math.sin(a), 1 - math.cos(a)
    return np.array([[t * x * x + c, t * x * y - s * z, t * x * z + s * y],
                     [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
                     [t * x * z - s * y, t * y * z + s * x, t * z * z + c]])


class Chain:
    """Cadena de articulaciones del URDF entre `root` y `tip`."""

    def __init__(self, urdf_xml, root="base_footprint", tip="r_wrist"):
        robot = ET.fromstring(urdf_xml)
        by_child = {}
        for j in robot.findall("joint"):
            by_child[j.find("child").get("link")] = j
        joints, link = [], tip
        while link != root:
            j = by_child[link]
            o = j.find("origin")
            xyz = np.array([float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()])
            rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
            ax = j.find("axis")
            axis = np.array([float(v) for v in ax.get("xyz").split()]) if ax is not None else np.array([1.0, 0, 0])
            lim = j.find("limit")
            joints.append({"name": j.get("name"), "type": j.get("type"), "xyz": xyz, "R": _rpy(*rpy),
                           "axis": axis / np.linalg.norm(axis),
                           "lower": float(lim.get("lower")) if lim is not None and lim.get("lower") else -math.pi,
                           "upper": float(lim.get("upper")) if lim is not None and lim.get("upper") else math.pi})
            link = j.find("parent").get("link")
        self.joints = joints[::-1]
        self.limits = {j["name"]: (j["lower"], j["upper"]) for j in self.joints}

    def fk(self, q):
        """T (4x4) de la punta en `root` para las posiciones `q` (dict nombre → rad; las que falten, 0)."""
        T = np.eye(4)
        for j in self.joints:
            A = np.eye(4)
            A[:3, :3], A[:3, 3] = j["R"], j["xyz"]
            T = T @ A
            if j["type"] in ("revolute", "continuous"):
                R = np.eye(4)
                R[:3, :3] = _axis_angle(j["axis"], q.get(j["name"], 0.0))
                T = T @ R
        return T


class ArmIK:
    def __init__(self, urdf_xml, lean_max=0.45):
        self.chain = Chain(urdf_xml)
        self.elbow = Chain(urdf_xml, tip="RForeArm")       # el codo
        self.lo = np.array([self.chain.limits[n][0] for n in VARS])
        self.hi = np.array([self.chain.limits[n][1] for n in VARS])
        # HipPitch negativo = inclinarse hacia delante; no más de lean_max ni hacia atrás
        self.lo[0], self.hi[0] = -lean_max, 0.0
        # dejar margen respecto a los límites de las articulaciones
        # 0.10 rad de margen: con la manzana en la mano el brazo cede un poco y, si una articulación
        # queda en su tope, MoveIt rechaza el estado de partida del siguiente movimiento
        self.lo[1:] += 0.10
        self.hi[1:] -= 0.10
        # RShoulderRoll llega hasta −0.009: con el brazo pegado al cuerpo el controlador lo deja en
        # ~0.0 y MoveIt rechaza el estado de partida ("Start state out of bounds")
        i = VARS.index("RShoulderRoll")
        self.hi[i] = min(self.hi[i], -0.15)

    def palm(self, x, fixed=None):
        """Punto y normal de la palma y eje del antebrazo (en base_footprint)."""
        q = dict(fixed or {})
        q.update(dict(zip(VARS, x)))
        T = self.chain.fk(q)
        R, p = T[:3, :3], T[:3, 3]
        return p + R @ PALM, R @ PALM_AXIS, R[:, 0]

    def solve(self, obj, radius, approach=0.0, fixed=None, seeds=None, lean=None, side=True):
        """Articulaciones (dict) para que el objeto (centro `obj` en base_footprint, radio `radius`)
        quede contra la palma; con `approach` > 0 la mano queda separada esa distancia (pre-agarre).
        `lean` fija la inclinación (HipPitch); `side`=False no exige palma de lado (para llevarlo).
        Devuelve (q, error de posición [m])."""
        obj = np.asarray(obj, float)
        gap = PALM_HALF + radius + approach

        def res(x):
            p, n, fwd = self.palm(x, fixed)
            r = list((p + gap * n - obj) * 10.0)          # el objeto en la normal de la palma
            r.append(1.5 * n[2] if side else 0.0)          # palma de lado (normal horizontal)
            r.append(0.8 * fwd[2])                         # antebrazo horizontal
            r.append(0.6 * max(0.0, -fwd[0]))              # y hacia delante
            r.append(0.4 * x[0])                           # inclinarse poco
            # el codo separado del costado (|y| > ELBOW_OUT): pegado, el brazo se apoya en el cuerpo
            # y el contacto lleva RShoulderRoll a su límite (MoveIt ya no puede planificar)
            q = dict(fixed or {})
            q.update(dict(zip(VARS, x)))
            ey = self.elbow.fk(q)[1, 3]
            r.append(10.0 * max(0.0, ey + ELBOW_OUT))
            return np.array(r)

        lo, hi = self.lo.copy(), self.hi.copy()
        if lean is not None:
            lo[0], hi[0] = lean - 1e-4, lean + 1e-4
        best = None
        seeds = seeds or [[-0.2, 0.2, -0.3, 1.2, 0.8, 1.0], [-0.3, -0.1, -0.1, 1.6, 0.4, 1.5],
                          [-0.1, 0.4, -0.6, 0.8, 1.2, 0.3], [-0.35, 0.0, -0.2, 1.4, 0.2, -0.5]]
        for s in seeds:
            s = np.clip(s, lo + 1e-6, hi - 1e-6)
            r = least_squares(res, s, bounds=(lo, hi), xtol=1e-8, ftol=1e-8, max_nfev=400)
            p, n, _ = self.palm(r.x, fixed)
            err = float(np.linalg.norm(p + gap * n - obj))
            if best is None or (err, r.cost) < (best[1], best[2]):
                best = (r.x, err, r.cost)
        return dict(zip(VARS, map(float, best[0]))), best[1]
