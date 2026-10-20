#!/usr/bin/env python3
"""Evalúa las llamadas registradas por visitor_approach (log_file) frente al ground truth del mundo.

    python3 tools/eval_approach.py <calls.jsonl> <mundo.sdf> [--max-distance 6]

Frame map = mundo + (1.5, −1) (Pepper aparece en (−1.5, 1) en museum.launch.py). Por llamada:
  - personas estáticas detectadas: emparejadas con la persona real más cercana (< 0.6 m) →
    error de posición; sin pareja = falso estático (un actor que pareció quieto)
  - personas reales a menos de --max-distance del robot y con línea de vista (paredes de
    maps/museum_walls.txt): cuántas encontró
  - la elegida, ¿es la real más cercana con línea de vista?; distancia final a ella
  - "sobre el mapa": detecciones descartadas por caer sobre el museo vacío (persona tapada)
"""

import argparse
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
from pepper_visitors.museum import Walls, people  # noqa: E402

OFFSET = (1.5, -1.0)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("calls")
    ap.add_argument("world")
    ap.add_argument("--max-distance", type=float, default=6.0)
    args = ap.parse_args()

    walls = Walls(os.path.join(HERE, "..", "..", "pepper_museum", "maps", "museum_walls.txt"))
    gt = [(n, c, x + OFFSET[0], y + OFFSET[1]) for n, c, x, y, _ in people(args.world)]
    errs, rows = [], []
    for line in open(args.calls):
        d = json.loads(line)
        rx, ry = d["robot_start"][:2]
        wx, wy = rx - OFFSET[0], ry - OFFSET[1]
        near = sorted((math.dist((rx, ry), (x, y)), n, c, x, y) for n, c, x, y in gt
                      if math.dist((rx, ry), (x, y)) <= args.max_distance
                      and walls.line_of_sight(wx, wy, x - OFFSET[0], y - OFFSET[1]))
        statics = [p for p in d.get("people", []) if p["static"]]
        found, false_static = set(), 0
        for p in statics:
            dist, n = min((math.dist(p["pos"], (x, y)), n) for n, _, x, y in gt)
            if dist < 0.6:
                errs.append(dist)
                found.add(n)
            else:
                false_static += 1
        moving = sum(not p["static"] and not p.get("on_map") for p in d.get("people", []))
        on_map = sum(bool(p.get("on_map")) for p in d.get("people", []))
        chosen = ok = None
        if d.get("target"):
            dist, chosen = min((math.dist(d["target"]["pos"], (x, y)), n) for n, _, x, y in gt)
            ok = bool(near) and chosen == near[0][1]
        rows.append({
            "start": (round(wx, 1), round(wy, 1)), "result": d["result"],
            "cerca_visibles": len(near), "encontradas": len([n for _, n, *_ in near if n in found]),
            "falsos_estaticos": false_static, "en_movimiento": moving, "sobre_mapa": on_map,
            "elegida": chosen, "clase": (d.get("target") or {}).get("class"),
            "mas_cercana": near[0][1] if near else None, "correcta": ok,
            "dist_final": round(d.get("final_dist", float("nan")), 2), "t": round(d["duration_s"], 1)})
    print("| Inicio (mundo) | Resultado | Cerca y visibles | Encontradas | Falsos estáticos | Andando | "
          "Sobre el mapa | Elegida | Clase | ¿La más cercana? | Dist. final [m] | Tiempo [s] |")
    print("|---|---|---|---|---|---|---|---|---|---|---|---|")
    for r in rows:
        print(f"| {r['start']} | {r['result']} | {r['cerca_visibles']} | {r['encontradas']} | "
              f"{r['falsos_estaticos']} | {r['en_movimiento']} | {r['sobre_mapa']} | {r['elegida']} | {r['clase']} | "
              f"{'sí' if r['correcta'] else 'no' if r['correcta'] is not None else '—'} | {r['dist_final']} | {r['t']} |")
    e = np.array(errs)
    reached = [r for r in rows if r["result"] == "reached"]
    print(f"\nLlamadas: {len(rows)}; llegó: {len(reached)}; eligió la más cercana: "
          f"{sum(bool(r['correcta']) for r in rows)}; error de posición de las estáticas: "
          f"media {e.mean():.2f} m, máx {e.max():.2f} m (n = {len(e)})" if len(e) else "")
    if reached:
        print(f"Distancia final: {np.mean([r['dist_final'] for r in reached]):.2f} m de media; "
              f"tiempo {np.mean([r['t'] for r in rows]):.0f} s de media")


if __name__ == "__main__":
    main()
