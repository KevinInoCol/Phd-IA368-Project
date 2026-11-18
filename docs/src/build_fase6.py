"""Pestaña 6 (coger la manzana) de proyecto.html = fase6.tpl.html + resultados + imágenes.

    python3 build_fase6.py            (desde docs/src)

Lee results/fetch/corrida*/trials.jsonl (tools/fetch_trials.py de pepper_fetch), calcula los KPI y la
tabla, incrusta img/fase6_*.jpg (@@nombre → data URI) e inserta o reemplaza la pestaña entre los
marcadores <!-- fase6 --> y <!-- /fase6 --> de ../proyecto.html (también el botón de la pestaña y el
id en el script de pestañas). Se puede volver a correr sin duplicar nada.
"""

import base64
import glob
import json
import os
import re
import statistics

HERE = os.path.dirname(os.path.abspath(__file__))
RES = os.path.join(HERE, "..", "..", "results", "fetch")
PAGE = os.path.join(HERE, "..", "proyecto.html")
# qué falló en cada serie (y si quedó corregido): se escribe a mano tras mirar los registros
FAILS = {
    "corrida1": "el nodo dio la manzana por caída (comprobación demasiado estricta) y la soltó: corregido",
    "corrida2": "MoveIt usaba el octomap de la cámara (MoveItConfigsBuilder carga sensors_3d.yaml): obstáculos "
                "falsos pegados a Pepper; corregido",
    "corrida3": "al levantarla, el codo quedó en su tope con el peso de la manzana: margen de 0.10 rad en la IK",
    "corrida4": "el torso cayó de lado hasta el tope de HipRoll y el motor (10.2 N·m) no lo levanta: la cámara "
                "veía torcido. Pendiente; ahora se detecta y lo dice",
}
RESULT = {"holding": "en la mano", "not_found": "no la ve", "unreachable": "no alcanza",
          "nav_failed": "no llega a la mesa", "error": "error", "timeout": "sin respuesta"}


def main():
    recs, rows = [], []
    for d in sorted(glob.glob(os.path.join(RES, "corrida*"))):
        rs = [json.loads(line) for line in open(os.path.join(d, "trials.jsonl"))]
        recs += rs
        name = os.path.basename(d)
        good = [r for r in rs if r.get("success")]
        e = [r["vision_err_cm"] for r in rs if r.get("vision_err_cm") is not None]
        bad = [f'intento {r["n"]} ({RESULT.get(r["phase"], r["phase"])})' for r in rs if not r.get("success")]
        fail = (", ".join(bad) + ": " + FAILS.get(name, "")) if bad else "—"
        rows.append(f'            <tr><td>{name.replace("corrida", "")}</td><td class="num new">{len(good)} de {len(rs)}</td>'
                    f'<td class="num">{statistics.mean(e):.1f} cm (máx. {max(e):.1f})</td>'
                    f'<td class="num">{statistics.mean(r["time_s"] for r in good):.0f} s</td><td>{fail}</td></tr>')
    ok = [r for r in recs if r.get("success")]
    errs = [r["vision_err_cm"] for r in recs if r.get("vision_err_cm") is not None]
    subs = {"ROWS": "\n".join(rows),
            "KPI_OK": f"{len(ok)} de {len(recs)}",
            "KPI_ERR": f"{statistics.mean(errs):.1f}" if errs else "—",
            "KPI_ERRMAX": f"{max(errs):.1f} cm" if errs else "—",
            "KPI_T": f"{statistics.mean(r['time_s'] for r in ok):.0f}" if ok else "—"}
    # entrega a un visitante (tools/deliver_trials.py)
    drs, drows = [], []
    for d in sorted(glob.glob(os.path.join(RES, "entrega*"))):
        drs += [json.loads(line) for line in open(os.path.join(d, "trials.jsonl"))]
    for r in drs:
        f = r.get("fetch") or {}
        res = ("entregada" if r.get("success") else
               RESULT.get(f.get("result"), f.get("result") or r.get("final_phase") or r.get("reach") or "—"))
        dist = f'{f["handover_dist"]:.2f} m' if r.get("success") and f.get("handover_dist") else "—"
        drows.append(f'            <tr><td>{r["n"]}</td><td class="num">({r["start"][0]:g}, {r["start"][1]:g})</td>'
                     f'<td class="{"num new" if r.get("success") else "num"}">{res}</td><td class="num">{dist}</td>'
                     f'<td class="num">{r["total_s"]:.0f} s</td></tr>')
    dok = [r for r in drs if r.get("success")]
    subs["DEL_ROWS"] = "\n".join(drows)
    subs["DEL_OK"] = f"{len(dok)} de {len(drs)} entregas" if drs else "—"
    html = open(os.path.join(HERE, "fase6.tpl.html"), encoding="utf-8").read()

    def sub(m):
        k = m.group(1)
        if k in subs:
            return subs[k]
        path = os.path.join(HERE, "img", f"fase6_{k}.jpg")
        return "data:image/jpeg;base64," + base64.b64encode(open(path, "rb").read()).decode()

    html = re.sub(r"@@(\w+)", sub, html)
    page = open(PAGE, encoding="utf-8").read()
    block = "<!-- fase6 -->\n" + html + "<!-- /fase6 -->\n"
    if "<!-- fase6 -->" in page:
        page = re.sub(r"<!-- fase6 -->.*?<!-- /fase6 -->\n", lambda _: block, page, flags=re.S)
    else:
        page = page.replace("<!-- /fase5 -->\n", "<!-- /fase5 -->\n" + block, 1)
    btn = ('<button role="tab" id="tab-fase6" data-tab="fase6" aria-controls="fase6">'
           '<span class="tn">6</span>Coger objetos</button>')
    if 'id="tab-fase6"' not in page:
        page = re.sub(r'(<button role="tab" id="tab-fase5"[^\n]*</button>)', lambda m: m.group(1) + "\n      " + btn, page)
    page = page.replace('var ids = ["fase1", "fase2", "fase3", "fase4", "fase5"];',
                        'var ids = ["fase1", "fase2", "fase3", "fase4", "fase5", "fase6"];')
    page = page.replace("pepper_visitors · pepper_cleaning · pepper_app ·",
                        "pepper_visitors · pepper_cleaning · pepper_fetch · pepper_app ·")
    open(PAGE, "w", encoding="utf-8").write(page)
    print(f"fase6: {len(ok)}/{len(recs)} intentos, {len(dok)}/{len(drs)} entregas; página {len(page) // 1024} KB")


if __name__ == "__main__":
    main()
