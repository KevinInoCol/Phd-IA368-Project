#!/usr/bin/env python3
"""¿Deja estelas la gente en el mapa? Celdas ocupadas que no están en el mapa de referencia.

    python3 tools/trails.py <ref.yaml> <mundo.sdf> <exp1> <exp2> ... [--out fig.png] [--json out.json]
                            [--titles "título 1" "título 2" ...]

Cada <exp> es un directorio con map.yaml (run_experiment.sh). Una celda ocupada del mapa es
"extra" si está a más de 15 cm de toda celda ocupada de la referencia (el museo vacío mapeado con
el hokuyo). Las extra se reparten en:
  - actores: a menos de 0.5 m de la trayectoria de algún actor (estelas de gente que camina)
  - personas: a menos de 0.5 m de una persona estática (obstáculos reales, no son error)
  - otras: errores del SLAM (paredes dobladas o desplazadas, ruido)
Frame map = mundo + (1.5, −1) (Pepper aparece en (−1.5, 1)). La figura dibuja, por experimento,
la referencia en gris, las extra coloreadas y las trayectorias de los actores.
"""

import argparse
import json
import os
import re

import numpy as np
import yaml
from PIL import Image
from scipy.ndimage import distance_transform_edt

OFFSET = np.array([1.5, -1.0])   # mundo -> map
TOL, NEAR = 0.15, 0.5


def load_map(path):
    with open(path) as f:
        meta = yaml.safe_load(f)
    img = np.array(Image.open(os.path.join(os.path.dirname(path), meta["image"])))
    res, (ox, oy) = meta["resolution"], meta["origin"][:2]
    r, c = np.nonzero(img < 100)
    h = img.shape[0]
    return {"img": img, "res": res, "origin": (ox, oy),
            "occ": np.column_stack([ox + (c + 0.5) * res, oy + (h - r - 0.5) * res])}


def dist_to(ref, pts):
    img, res, (ox, oy) = ref["img"], ref["res"], ref["origin"]
    dt = distance_transform_edt(~(img < 100)) * res
    h, w = img.shape
    c = np.floor((pts[:, 0] - ox) / res).astype(int)
    r = h - 1 - np.floor((pts[:, 1] - oy) / res).astype(int)
    d = np.full(len(pts), np.inf)
    ok = (c >= 0) & (c < w) & (r >= 0) & (r < h)
    d[ok] = dt[r[ok], c[ok]]
    return d


def world_people(sdf):
    s = open(sdf).read()
    people = [tuple(map(float, p.split()[:2]))
              for n, p in re.findall(r"<include>\s*<name>(.*?)</name>.*?<pose>(.*?)</pose>", s, re.S)
              if "museum" not in n]
    actors = [np.array([list(map(float, q.split()[:2]))
                        for q in re.findall(r"<waypoint>.*?<pose>(.*?)</pose>", a, re.S)])
              for a in re.findall(r"<actor.*?</actor>", s, re.S)]
    return np.array(people).reshape(-1, 2) + OFFSET, [a + OFFSET for a in actors]


def seg_dist(pts, path):
    """Distancia de cada punto a la polilínea `path` (cerrada: los actores van en bucle)."""
    d = np.full(len(pts), np.inf)
    loop = np.vstack([path, path[:1]])
    for a, b in zip(loop[:-1], loop[1:]):
        ab = b - a
        t = np.clip(((pts - a) @ ab) / max(ab @ ab, 1e-9), 0, 1)
        d = np.minimum(d, np.linalg.norm(pts - (a + t[:, None] * ab), axis=1))
    return d


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("ref")
    ap.add_argument("world")
    ap.add_argument("experiments", nargs="+")
    ap.add_argument("--out", default=None)
    ap.add_argument("--json", default=None)
    ap.add_argument("--titles", nargs="*", default=None, help="títulos de los paneles de la figura")
    args = ap.parse_args()

    ref = load_map(args.ref)
    people, actors = world_people(args.world)
    res = {}
    for exp in args.experiments:
        m = load_map(os.path.join(exp, "map.yaml"))
        extra = m["occ"][dist_to(ref, m["occ"]) > TOL]
        da = np.min([seg_dist(extra, a) for a in actors], axis=0) if actors and len(extra) else np.full(len(extra), np.inf)
        dp = np.min(np.linalg.norm(extra[:, None] - people[None], axis=2), axis=1) if len(people) and len(extra) else np.full(len(extra), np.inf)
        on_people = dp < NEAR
        on_actor = (da < NEAR) & ~on_people
        cell = m["res"] ** 2
        res[os.path.basename(os.path.normpath(exp))] = {
            "occ_m2": len(m["occ"]) * cell, "extra_m2": len(extra) * cell,
            "actores_m2": on_actor.sum() * cell, "personas_m2": on_people.sum() * cell,
            "otras_m2": (~on_actor & ~on_people).sum() * cell,
            "_pts": (extra, on_actor, on_people)}
        r = res[os.path.basename(os.path.normpath(exp))]
        print(f"{os.path.basename(os.path.normpath(exp)):24s} ocupado {r['occ_m2']:6.2f} m²  extra {r['extra_m2']:5.2f}  "
              f"actores {r['actores_m2']:5.2f}  personas {r['personas_m2']:5.2f}  otras {r['otras_m2']:5.2f}")

    if args.json:
        with open(args.json, "w") as f:
            json.dump({k: {kk: float(vv) for kk, vv in v.items() if not kk.startswith("_")}
                       for k, v in res.items()}, f, indent=2)
    if args.out:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        n = len(res)
        fig, axs = plt.subplots(1, n, figsize=(5.2 * n, 6), squeeze=False)
        titles = args.titles or list(res)
        for ax, title, (name, r) in zip(axs[0], titles, res.items()):
            ax.scatter(*ref["occ"].T, s=0.3, c="0.7")
            for a in actors:
                lp = np.vstack([a, a[:1]])
                ax.plot(*lp.T, "-", c="tab:green", lw=0.8, alpha=0.6)
            extra, on_a, on_p = r["_pts"]
            ax.scatter(*extra[on_a].T, s=1.5, c="tab:red", label="junto a actores")
            ax.scatter(*extra[on_p].T, s=1.5, c="tab:blue", label="personas estáticas")
            ax.scatter(*extra[~on_a & ~on_p].T, s=1.5, c="k", label="otras")
            if len(people):
                ax.plot(*people.T, "o", mfc="none", c="tab:blue", ms=6)
            ax.set_title(title, fontsize=11)
            ax.set_aspect("equal")
            ax.set_xticks([])
            ax.set_yticks([])
        axs[0][0].legend(fontsize=7, loc="lower left", markerscale=4)
        fig.tight_layout()
        fig.savefig(args.out, dpi=120, bbox_inches="tight")


if __name__ == "__main__":
    main()
