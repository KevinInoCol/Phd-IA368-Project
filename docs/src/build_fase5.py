"""Pestaña 5 (limpieza) de proyecto.html = fase5.tpl.html + resultados + imágenes.

    python3 build_fase5.py            (desde docs/src)

Lee results/limpieza/*/summary.json (corridas finales test_r*, moving_r*, e historial v1/v2),
calcula los KPI y la tabla, incrusta img/fase5_*.jpg (@@nombre → data URI) e inserta o reemplaza la
pestaña entre los marcadores <!-- fase5 --> y <!-- /fase5 --> de ../proyecto.html (también el botón
de la pestaña y el id en el script de pestañas). Se puede volver a correr sin duplicar nada.
"""

import base64
import glob
import json
import os
import re
import statistics

import yaml

VISIT_CAPTION = ("Llamada en mitad de la sala oeste, en el museo con gente caminando: Pepper dejó el zigzag, "
                 "eligió a la visitante quieta del vestido negro (arriba) y quedó a 1.1 m, de frente. El de verde es "
                 "un actor que camina y no lo eligió. Abajo, una franja que todavía le falta limpiar.")
HERE = os.path.dirname(os.path.abspath(__file__))
RES = os.path.join(HERE, "..", "..", "results", "limpieza")
PAGE = os.path.join(HERE, "..", "proyecto.html")
ZONES = ["sala_oeste", "esquina_base", "pasillo_sur", "sala_norte", "entrada"]


def load(name):
    p = os.path.join(RES, name, "summary.json")
    return json.load(open(p)) if os.path.exists(p) else None


def row(label, s, cls=""):
    z = s["zones"]
    cells = [f'<td>{label}</td>', f'<td class="num {cls}">{s["percent"]:.1f} %</td>']
    cells += [f'<td class="num">{z[k]:.0f} %</td>' for k in ZONES]
    cells += [f'<td class="num">{s["duration_min"]:.1f} min</td>', f'<td class="num">{s["path_m"]:.0f} m</td>',
              f'<td class="num">{s["dock_cycles"]}</td>']
    return "          <tr>" + "".join(cells) + "</tr>"


ZONE_NAMES = {"sala_oeste": "sala oeste", "esquina_base": "esquina de la base", "pasillo_sur": "pasillo sur",
              "sala_norte": "sala norte", "entrada": "entrada"}
CFG = os.path.join(HERE, "..", "..", "pepper_cleaning", "config", "dirt.yaml")


def visitor_calls():
    """Filas de la tabla de llamadas (results/limpieza/visitantes_r*/calls.jsonl) y sus números."""
    rects = {k: z["rect"] for k, z in yaml.safe_load(open(CFG))["zones"].items()}
    rows, calls = [], []
    for d in sorted(glob.glob(os.path.join(RES, "visitantes_r*"))):
        f = os.path.join(d, "calls.jsonl")
        if not os.path.exists(f):
            continue
        run = os.path.basename(d).replace("visitantes_", "")
        for c in map(json.loads, open(f)):
            calls.append(c)
            if c.get("dock_state") == "CHARGING":
                where = f"en la base ({c.get('battery', 0):.0f} %)"
            else:
                # "limpiando" sólo si está dentro de la zona que limpia (si no, va de camino a ella)
                x, y = c.get("gt_call") or (None, None)
                z = c.get("zone")
                x0, x1, y0, y1 = rects.get(z, (0, 0, 0, 0))
                inside = x is not None and x0 <= x <= x1 and y0 <= y <= y1
                where = f"limpiando {ZONE_NAMES.get(z, z)}" if inside else f"de camino a {ZONE_NAMES.get(z, z)}"
            talk = {"chat": "se despide", "silent": "nadie habla"}.get(c["kind"], "—")
            res = {"reached_and_talked": "se acerca y conversa", "nobody": "nadie quieto cerca",
                   "charging": "no va: batería baja", "nav_failed": "no puede llegar",
                   "error": "error"}.get(c.get("result", "").strip(), c.get("result", ""))
            if c.get("result") == "reached_and_talked" and c["kind"] != "chat":   # la prueba no simula charla
                res, talk = "se acerca; cierra a los 60 s", "nadie habla"
            reach = f'{c["dt_reached"]:.0f} s' if c.get("dt_reached") is not None else "—"
            dist = f'{c["gt_dist"]:.2f} m' if c.get("gt_dist") is not None else "—"
            if c.get("dt_resumed") is not None:
                back = f'{c["dt_resumed"]:.0f} s'
            elif c.get("dock_after") in ("TO_PREDOCK", "SEEK", "APPROACH"):
                back = "va a cargar (batería) y sigue después"
            elif c.get("result") == "charging":
                back = "sigue cargando"
            else:
                back = "—"
            cls = ' class="new"' if c.get("dt_reached") is not None else ""
            rows.append(f'          <tr><td>{run} · {c["n"]}</td><td>{where}</td><td>{talk}</td>'
                        f'<td{cls}>{res}</td><td class="num">{reach}</td><td class="num">{dist}</td>'
                        f'<td>{back}</td></tr>')
    return rows, calls


def main():
    finals = sorted(glob.glob(os.path.join(RES, "test_r*")))
    runs = [(os.path.basename(p), load(os.path.basename(p))) for p in finals]
    runs = [(n, s) for n, s in runs if s and s.get("finished")]
    moving = [(os.path.basename(p), load(os.path.basename(p))) for p in sorted(glob.glob(os.path.join(RES, "moving_r*")))]
    moving = [(n, s) for n, s in moving if s and s.get("finished")]
    rows = []
    v1, v2 = load("historial_v1_sin_repaso"), load("historial_v2_repaso_mapa_laser")
    if v1:
        rows.append(row("v1 · sólo zigzag (mapa del láser)", v1))
    if v2:
        rows.append(row("v2 · + repaso (mapa del láser)", v2))
    for n, s in runs:
        rows.append(row(f"final · {n.replace('test_', 'corrida ')}", s, "new"))
    if len(runs) > 1:
        mean = {"percent": statistics.mean(s["percent"] for _, s in runs),
                "zones": {k: statistics.mean(s["zones"][k] for _, s in runs) for k in ZONES},
                "duration_min": statistics.mean(s["duration_min"] for _, s in runs),
                "path_m": statistics.mean(s["path_m"] for _, s in runs),
                "dock_cycles": sum(s["dock_cycles"] for _, s in runs)}
        rows.append(row(f"<b>final · media de {len(runs)}</b>", mean, "new"))
    for n, s in moving:
        rows.append(row(f"final · con gente caminando ({n.replace('moving_', '')})", s, "new"))

    allf = runs + moving
    pct = statistics.mean(s["percent"] for _, s in runs) if runs else 0
    docks = sum(s["dock_cycles"] for _, s in allf)
    mins = statistics.mean(s["duration_min"] for _, s in runs) if runs else 0
    bad = [s["amcl_rmse_by_zone_m"][k] for _, s in runs for k in ("sala_norte", "entrada")
           if "amcl_rmse_by_zone_m" in s and k in s["amcl_rmse_by_zone_m"]]
    good = [s["amcl_rmse_by_zone_m"][k] for _, s in runs for k in ("sala_oeste", "esquina_base", "pasillo_sur")
            if "amcl_rmse_by_zone_m" in s and k in s["amcl_rmse_by_zone_m"]]

    vrows, calls = visitor_calls()
    found = [c for c in calls if c.get("target")]
    reached = [c for c in found if c.get("dt_reached") is not None]
    cleaning = [c for c in calls if c.get("clean_state") == "CLEANING"]
    paused = [c["dt_paused"] for c in cleaning if c.get("dt_paused") is not None]
    back = [c for c in cleaning if c.get("dt_resumed") is not None or c.get("dock_after") == "TO_PREDOCK"]
    dists = [c["gt_dist"] for c in reached if c.get("gt_dist") is not None]
    vsum = f"{len(calls)} llamadas:"
    vsum += (f" se acercó a la persona elegida en {len(reached)} de {len(found)}"
             f" ({min(dists):.2f}–{max(dists):.2f} m), y en {len(back)} de {len(cleaning)} llamadas en mitad de la"
             f" limpieza volvió a limpiar." if dists else "")
    vsum += (" Cuando no encontró a nadie, lo habían llamado cerca de la base o en un pasillo, donde no hay"
             " gente quieta en ese mundo. La llamada a 0.45 m fue porque ya estaba limpiando junto a la persona y no"
             " se apartó. Ahora, si está a menos de 0.75 m, retrocede a 1 m (prueba aparte: de 0.50 a 1.11 m).")
    html = open(os.path.join(HERE, "fase5.tpl.html"), encoding="utf-8").read()
    subs = {"KPI_PCT": f"{pct:.0f} %", "KPI_RUNS": str(len(runs)), "KPI_DOCKS": str(docks),
            "ALL_RUNS": str(len(allf)),
            "KPI_MIN": f"{mins:.0f}", "RESULTS_ROWS": "\n".join(rows),
            "AMCL_BAD": f"{statistics.mean(bad):.2f}" if bad else "—",
            "AMCL_GOOD": f"{statistics.mean(good):.2f}" if good else "—",
            "VISIT_ROWS": "\n".join(vrows), "VISIT_SUMMARY": vsum,
            "VISIT_PAUSE": f"{statistics.median(paused):.1f}" if paused else "—",
            "VISIT_CAPTION": VISIT_CAPTION}

    def sub(m):
        k = m.group(1)
        if k in subs:
            return subs[k]
        path = os.path.join(HERE, "img", f"fase5_{k}.jpg")
        return "data:image/jpeg;base64," + base64.b64encode(open(path, "rb").read()).decode()

    html = re.sub(r"@@(\w+)", sub, html)
    page = open(PAGE, encoding="utf-8").read()
    block = "<!-- fase5 -->\n" + html + "<!-- /fase5 -->\n"
    if "<!-- fase5 -->" in page:
        page = re.sub(r"<!-- fase5 -->.*?<!-- /fase5 -->\n", lambda _: block, page, flags=re.S)
    else:
        page = page.replace("<footer>", block + "<footer>", 1)
    btn = ('<button role="tab" id="tab-fase5" data-tab="fase5" aria-controls="fase5">'
           '<span class="tn">5</span>Limpieza y carga</button>')
    if 'id="tab-fase5"' not in page:
        page = re.sub(r'(<button role="tab" id="tab-fase4"[^\n]*</button>)', lambda m: m.group(1) + "\n      " + btn, page)
    page = page.replace('var ids = ["fase1", "fase2", "fase3", "fase4"];',
                        'var ids = ["fase1", "fase2", "fase3", "fase4", "fase5"];')
    page = page.replace("pepper_multilaser_slam · pepper_museum · pepper_visitors · pepper_app ·",
                        "pepper_multilaser_slam · pepper_museum · pepper_visitors · pepper_cleaning · pepper_app ·")
    open(PAGE, "w", encoding="utf-8").write(page)
    print(f"fase5: {len(runs)} corridas finales + {len(moving)} con gente caminando; "
          f"limpio {pct:.1f} %, recargas {docks}; página {len(page) // 1024} KB")


if __name__ == "__main__":
    main()
