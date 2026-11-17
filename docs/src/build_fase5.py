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

    html = open(os.path.join(HERE, "fase5.tpl.html"), encoding="utf-8").read()
    subs = {"KPI_PCT": f"{pct:.0f} %", "KPI_RUNS": str(len(runs)), "KPI_DOCKS": str(docks),
            "ALL_RUNS": str(len(allf)),
            "KPI_MIN": f"{mins:.0f}", "RESULTS_ROWS": "\n".join(rows),
            "AMCL_BAD": f"{statistics.mean(bad):.2f}" if bad else "—",
            "AMCL_GOOD": f"{statistics.mean(good):.2f}" if good else "—"}

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
