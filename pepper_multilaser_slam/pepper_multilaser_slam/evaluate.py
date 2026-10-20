#!/usr/bin/env python3
"""Evaluación offline de los experimentos de SLAM.

    ros2 run pepper_multilaser_slam evaluate <dir_exp1> <dir_exp2> ... [--ref mapa.yaml] [--out dir]

Las repeticiones se nombran <config>_r<k> (p. ej. acc4_r1, acc4_r2): la tabla agrupada da
media ± desviación por configuración y las figuras usan la primera repetición de cada una.

Cada directorio de experimento contiene trajectory.csv (trajectory_recorder) y map.yaml/.pgm
(map_saver_cli). Métricas:

Trayectoria (frente al ground truth de Gazebo; la estimación se alinea con la pose inicial,
porque el frame map nace en la pose de arranque del robot):
  - ATE RMSE [m]: error de posición cuadrático medio a lo largo del recorrido
  - error final [m] y error de orientación RMSE [°]
  - lo mismo para la odometría sola (sin SLAM), como referencia

Mapa (frente a un mapa de referencia en el mismo frame, por defecto el mapa del hokuyo de
pepper_gazebo_plugin/map/office.yaml):
  - precisión: distancia media de cada celda ocupada a la pared de referencia más cercana y % a
    menos de 10 cm
  - completitud: % de celdas ocupadas de referencia con una celda ocupada a menos de 10 cm
  - área libre conocida [m²]
"""

import argparse
import json
import math
import os
import re

import numpy as np
import yaml
from PIL import Image
from scipy.ndimage import distance_transform_edt

TOL = 0.10  # m


def wrap(a):
    return np.arctan2(np.sin(a), np.cos(a))


def load_traj(path):
    d = np.genfromtxt(path, delimiter=",", names=True)
    return {k: d[k] for k in d.dtype.names}


def align_to_first(gt_x, gt_y, gt_yaw, x, y, yaw):
    """Lleva la estimación al frame del mundo haciendo coincidir la primera pose."""
    dth = gt_yaw[0] - yaw[0]
    c, s = math.cos(dth), math.sin(dth)
    xr = c * (x - x[0]) - s * (y - y[0]) + gt_x[0]
    yr = s * (x - x[0]) + c * (y - y[0]) + gt_y[0]
    return xr, yr, wrap(yaw + dth)


def traj_metrics(tr, prefix):
    x, y, yaw = align_to_first(tr["gt_x"], tr["gt_y"], tr["gt_yaw"],
                               tr[prefix + "_x"], tr[prefix + "_y"], tr[prefix + "_yaw"])
    err = np.hypot(x - tr["gt_x"], y - tr["gt_y"])
    yerr = np.degrees(wrap(yaw - tr["gt_yaw"]))
    return {
        "ate_rmse_m": float(np.sqrt(np.mean(err ** 2))),
        "ate_max_m": float(err.max()),
        "final_err_m": float(err[-1]),
        "yaw_rmse_deg": float(np.sqrt(np.mean(yerr ** 2))),
    }, (x, y, err)


def load_map(yaml_path):
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)
    img = np.array(Image.open(os.path.join(os.path.dirname(yaml_path), meta["image"])))
    res = meta["resolution"]
    ox, oy = meta["origin"][:2]
    occ = img < 100            # trinary: 0 ocupado, 254 libre, 205 desconocido
    free = img > 250
    rows, cols = np.nonzero(occ)
    h = img.shape[0]
    pts = np.column_stack([ox + (cols + 0.5) * res, oy + (h - rows - 0.5) * res])
    return {"img": img, "res": res, "origin": (ox, oy), "occ_pts": pts,
            "free_area_m2": float(free.sum() * res * res)}


def nearest_dist(ref, pts):
    """Distancia de cada punto a la celda ocupada más cercana de `ref` (transformada de distancia)."""
    img = ref["img"]
    dt = distance_transform_edt(~(img < 100)) * ref["res"]
    ox, oy = ref["origin"]
    h, w = img.shape
    cols = np.floor((pts[:, 0] - ox) / ref["res"]).astype(int)
    rows = h - 1 - np.floor((pts[:, 1] - oy) / ref["res"]).astype(int)
    inside = (rows >= 0) & (rows < h) & (cols >= 0) & (cols < w)
    d = np.full(len(pts), np.inf)
    d[inside] = dt[rows[inside], cols[inside]]
    return d


def map_metrics(m, ref):
    to_ref = nearest_dist(ref, m["occ_pts"])
    to_res = nearest_dist(m, ref["occ_pts"])
    finite = np.isfinite(to_ref)
    return {
        "precision_mean_dist_m": float(np.mean(np.minimum(to_ref[finite], 1.0))),
        "precision_pct_within_10cm": float(100 * np.mean(to_ref <= TOL)),
        "completeness_pct_within_10cm": float(100 * np.mean(to_res <= TOL)),
        "occupied_cells": int(len(m["occ_pts"])),
        "free_area_m2": m["free_area_m2"],
    }


def default_ref():
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory("pepper_gazebo_plugin"), "map", "office.yaml")
    except Exception:
        return None


def group_of(name):
    return re.sub(r"_r\d+$", "", name)


def grouped_table(results):
    groups = {}
    for n, r in results.items():
        groups.setdefault(group_of(n), []).append(r)
    cols = [("ATE RMSE SLAM [m]", lambda r: r["slam"]["ate_rmse_m"], 3),
            ("Error final [m]", lambda r: r["slam"]["final_err_m"], 3),
            ("Yaw RMSE [°]", lambda r: r["slam"]["yaw_rmse_deg"], 2),
            ("ATE odometría [m]", lambda r: r["odom"]["ate_rmse_m"], 3),
            ("Precisión mapa (% ≤10 cm)", lambda r: r["map"]["precision_pct_within_10cm"], 1),
            ("Completitud (% ≤10 cm)", lambda r: r["map"]["completeness_pct_within_10cm"], 1),
            ("Área libre [m²]", lambda r: r["map"]["free_area_m2"], 1)]
    lines = ["| Configuración | n | " + " | ".join(c[0] for c in cols) + " |",
             "|---|---|" + "---|" * len(cols)]
    for g, rs in groups.items():
        cells = []
        for _, f, d in cols:
            try:
                v = np.array([f(r) for r in rs])
            except KeyError:
                cells.append("—")
                continue
            cells.append(f"{v.mean():.{d}f} ± {v.std(ddof=1):.{d}f}" if len(v) > 1 else f"{v[0]:.{d}f}")
        lines.append(f"| {g} | {len(rs)} | " + " | ".join(cells) + " |")
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("experiments", nargs="+")
    ap.add_argument("--ref", default=default_ref(), help="mapa de referencia (yaml)")
    ap.add_argument("--out", default=None, help="carpeta de salida (por defecto, la del 1er exp.)")
    args = ap.parse_args()

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    out = args.out or os.path.dirname(os.path.abspath(args.experiments[0]))
    os.makedirs(out, exist_ok=True)
    ref = load_map(args.ref) if args.ref else None

    results = {}
    fig_t, ax_t = plt.subplots(figsize=(7, 9))
    fig_e, ax_e = plt.subplots(figsize=(9, 4))
    gt_drawn = False
    plotted = set()
    for exp in args.experiments:
        name = os.path.basename(os.path.normpath(exp))
        tr = load_traj(os.path.join(exp, "trajectory.csv"))
        slam, (sx, sy, serr) = traj_metrics(tr, "slam")
        odom, (oxx, oyy, oerr) = traj_metrics(tr, "odom")
        r = {"samples": int(len(tr["t"])), "duration_s": float(tr["t"][-1] - tr["t"][0]),
             "slam": slam, "odom": odom}
        map_yaml = os.path.join(exp, "map.yaml")
        if ref is not None and os.path.exists(map_yaml):
            r["map"] = map_metrics(load_map(map_yaml), ref)
        results[name] = r

        if group_of(name) in plotted:
            continue
        plotted.add(group_of(name))
        t = tr["t"] - tr["t"][0]
        if not gt_drawn:
            ax_t.plot(tr["gt_x"], tr["gt_y"], "k-", lw=2.5, label="ground truth")
            ax_t.plot(oxx, oyy, "--", color="gray", lw=1, label="solo odometría")
            ax_e.plot(t, oerr, "--", color="gray", lw=1, label="solo odometría")
            gt_drawn = True
        ax_t.plot(sx, sy, lw=1.3, label=group_of(name))
        ax_e.plot(t, serr, lw=1.3, label=group_of(name))

    ax_t.set_aspect("equal")
    ax_t.set_xlabel("x [m]")
    ax_t.set_ylabel("y [m]")
    ax_t.set_title("Trayectorias (frame del mundo)")
    ax_t.legend(fontsize=8)
    ax_t.grid(alpha=0.3)
    fig_t.tight_layout()
    fig_t.savefig(os.path.join(out, "trajectories.png"), dpi=130)
    ax_e.set_xlabel("t [s]")
    ax_e.set_ylabel("error de posición [m]")
    ax_e.set_title("Error de posición frente al ground truth")
    ax_e.legend(fontsize=8)
    ax_e.grid(alpha=0.3)
    fig_e.tight_layout()
    fig_e.savefig(os.path.join(out, "position_error.png"), dpi=130)

    # Mapas lado a lado
    maps, seen = [], set()
    for n, e in zip(results, args.experiments):
        if group_of(n) not in seen and os.path.exists(os.path.join(e, "map.yaml")):
            seen.add(group_of(n))
            maps.append((n, os.path.join(e, "map.yaml")))
    if maps:
        ncol = min(4, len(maps))
        nrow = math.ceil(len(maps) / ncol)
        # Encuadre: las paredes del mapa de referencia (o del primer mapa) con 0.5 m de margen
        occ = (ref or load_map(maps[0][1]))["occ_pts"]
        (x0, y0), (x1, y1) = occ.min(0) - 0.5, occ.max(0) + 0.5
        aspect = (y1 - y0) / (x1 - x0)
        fig_m, axs = plt.subplots(nrow, ncol, figsize=(3.6 * ncol, 3.6 * aspect * nrow + 0.4 * nrow),
                                  squeeze=False)
        axs = axs.ravel()
        for ax in axs[len(maps):]:
            ax.axis("off")
        for ax, (n, y) in zip(axs, maps):
            m = load_map(y)
            h, w = m["img"].shape
            ox, oy = m["origin"]
            ax.imshow(m["img"], cmap="gray", vmin=0, vmax=255,
                      extent=[ox, ox + w * m["res"], oy, oy + h * m["res"]])
            ax.set_title(group_of(n), fontsize=10)
            ax.set_xlim(x0, x1)
            ax.set_ylim(y0, y1)
        fig_m.tight_layout()
        fig_m.savefig(os.path.join(out, "maps.png"), dpi=130)

    with open(os.path.join(out, "summary.json"), "w") as f:
        json.dump(results, f, indent=2)

    lines = ["| Experimento | ATE RMSE SLAM [m] | Error final [m] | Yaw RMSE [°] | ATE odometría [m] |"
             " Precisión mapa (% ≤10 cm) | Completitud (% ≤10 cm) | Área libre [m²] |",
             "|---|---|---|---|---|---|---|---|"]
    for n, r in results.items():
        mm = r.get("map", {})
        lines.append(
            f"| {n} | {r['slam']['ate_rmse_m']:.3f} | {r['slam']['final_err_m']:.3f} | "
            f"{r['slam']['yaw_rmse_deg']:.2f} | {r['odom']['ate_rmse_m']:.3f} | "
            f"{mm.get('precision_pct_within_10cm', float('nan')):.1f} | "
            f"{mm.get('completeness_pct_within_10cm', float('nan')):.1f} | "
            f"{mm.get('free_area_m2', float('nan')):.1f} |")
    table = "\n".join(lines)
    gtable = grouped_table(results)
    with open(os.path.join(out, "summary.md"), "w") as f:
        f.write("## Por configuración (media ± desviación)\n\n" + gtable +
                "\n\n## Por corrida\n\n" + table + "\n")
    print(gtable)
    print()
    print(table)


if __name__ == "__main__":
    main()
