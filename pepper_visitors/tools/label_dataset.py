#!/usr/bin/env python3
"""Etiqueta automáticamente las capturas de capture_dataset y recorta a cada persona.

    <venv de yolo_ros>/bin/python tools/label_dataset.py <raw> <out> [--yolo yolo11m.pt]

<raw> tiene una carpeta por mundo (images/ + meta.jsonl). Para cada imagen, YOLO detecta
personas y cada caja se compara con la caja esperada de cada persona estática del mundo,
proyectada con la pose real de Pepper (gt_pose) y la cámara de museum.py:

  - coincide con una sola persona (columna y altura compatibles) → la clase de su modelo;
  - coincide con varias → ambigua, se descarta;
  - no coincide con ninguna y el mundo tiene actores → "otro" (un actor), siempre que no haya
    ninguna persona estática cerca de esa columna aunque esté tapada por una pared.
En el mundo con actores sólo se guarda "otro": un actor que pasa delante de una persona estática
coincidiría con ella, así que las clases de visitantes salen sólo del mundo sin actores.

Los recortes (caja + 8 %, al menos 48 px de alto) van a <out>/{train,val,test}/<clase>/. La
división es por persona: las de VAL y TEST nunca aparecen en train, así que la precisión de test
mide si el modelo reconoce a un visitante en un sitio y con un fondo que no vio. Los actores se
dividen por imagen. <out>/labels.csv guarda cada decisión para revisarla.
"""

import argparse
import csv
import json
import math
import os
import sys
import zlib

import cv2

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from pepper_visitors import museum  # noqa: E402

WORLDS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "pepper_museum", "worlds")
WALLS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "pepper_museum", "maps",
                     "museum_walls.txt")
VAL = {"female_10", "female_06", "male_05", "male_13"}
TEST = {"female02_Inf_Der", "female03_Med_Med", "male_10", "male_12"}
CONF = 0.35
MIN_H = 48


def split_of(person, image):
    if person in TEST:
        return "test"
    if person in VAL:
        return "val"
    if person is None:  # actor: por imagen, 70/15/15
        b = zlib.crc32(image.encode()) % 20
        return "train" if b < 14 else "val" if b < 17 else "test"
    return "train"


def expected(persons, walls, pose):
    out = []
    for name, cls, px, py, _ in persons:
        e = museum.project(px, py, *pose)
        if e is None or not -200 < e[0] < museum.W + 200:
            continue
        out.append((name, cls, e, walls.line_of_sight(pose[0], pose[1], px, py)))
    return out


def fits(box, e):
    """Columna y extensión vertical compatibles con la persona esperada."""
    x1, y1, x2, y2 = box
    u, vt, vb, _ = e
    he = vb - vt
    du = abs((x1 + x2) / 2 - u) / (0.2 * he + 20)
    if du > 1:
        return None
    top_cut, bottom_cut = y1 < 3, y2 > museum.H - 3
    tol = 0.15 * he + 15
    if not bottom_cut and abs(y2 - vb) > tol:
        return None
    if not top_cut and abs(y1 - vt) > tol:
        return None
    return du


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("raw")
    ap.add_argument("out")
    ap.add_argument("--yolo", default=os.path.expanduser("~/pepper_ws/yolo11m.pt"))
    args = ap.parse_args()

    from ultralytics import YOLO
    model = YOLO(args.yolo)
    walls = museum.Walls(WALLS)
    os.makedirs(args.out, exist_ok=True)
    log = csv.writer(open(os.path.join(args.out, "labels.csv"), "w", newline=""))
    log.writerow(["image", "box", "conf", "decision", "person", "class", "split", "dist", "crop"])
    stats = {}

    for world in sorted(os.listdir(args.raw)):
        meta_path = os.path.join(args.raw, world, "meta.jsonl")
        if not os.path.exists(meta_path):
            continue
        sdf = os.path.join(WORLDS, world + ".sdf")
        persons, has_actors = museum.people(sdf), bool(museum.actor_paths(sdf))
        metas = [json.loads(l) for l in open(meta_path)]
        print(f"{world}: {len(metas)} imágenes, {len(persons)} personas, actores: {has_actors}")
        for m in metas:
            path = os.path.join(args.raw, world, "images", m["image"])
            img = cv2.imread(path)
            exp = expected(persons, walls, m["gt_pose"])
            res = model(img, verbose=False, classes=[0], conf=CONF)[0]
            dets = [(tuple(float(v) for v in b.xyxy[0]), float(b.conf)) for b in res.boxes]
            claims = {}
            for i, (box, conf) in enumerate(dets):
                cands = [(fits(box, e), name, cls, e) for name, cls, e, los in exp if los]
                cands = [c for c in cands if c[0] is not None]
                if len(cands) == 1:
                    du, name, cls, e = cands[0]
                    if name not in claims or du < claims[name][0]:
                        claims[name] = (du, i, cls, e[3])
                    continue
                decision = "ambigua" if cands else "sin_persona"
                near = any(abs((box[0] + box[2]) / 2 - e[0]) < 2.5 * (0.2 * (e[2] - e[1]) + 20) for _, _, e, _ in exp)
                if not cands and has_actors and conf >= 0.5 and not near:
                    decision = "otro"
                    save(img, box, museum.OTHER, None, m, i, conf, args.out, log, stats, None)
                else:
                    log.writerow([m["image"], fmt(box), f"{conf:.2f}", decision, "", "", "", ""])
                    stats[decision] = stats.get(decision, 0) + 1
            for name, (du, i, cls, dist) in claims.items():
                if has_actors:  # un actor puede pasar delante: las estáticas sólo del mundo sin actores
                    log.writerow([m["image"], fmt(dets[i][0]), f"{dets[i][1]:.2f}", "estatica_con_actores",
                                  name, cls, "", f"{dist:.2f}"])
                    stats["estatica_con_actores"] = stats.get("estatica_con_actores", 0) + 1
                    continue
                save(img, dets[i][0], cls, name, m, i, dets[i][1], args.out, log, stats, dist)
    print("decisiones:", stats)
    for split in ("train", "val", "test"):
        d = os.path.join(args.out, split)
        if os.path.isdir(d):
            print(split, {c: len(os.listdir(os.path.join(d, c))) for c in sorted(os.listdir(d))})


def fmt(box):
    return " ".join(f"{v:.0f}" for v in box)


def save(img, box, cls, person, m, i, conf, out, log, stats, dist):
    x1, y1, x2, y2 = box
    if y2 - y1 < MIN_H:
        log.writerow([m["image"], fmt(box), f"{conf:.2f}", "pequeña", person or "", cls, "", ""])
        stats["pequeña"] = stats.get("pequeña", 0) + 1
        return
    split = split_of(person, m["image"])
    px, py = 0.08 * (x2 - x1), 0.08 * (y2 - y1)
    crop = img[max(0, int(y1 - py)):min(museum.H, int(y2 + py)), max(0, int(x1 - px)):min(museum.W, int(x2 + px))]
    d = os.path.join(out, split, cls)
    os.makedirs(d, exist_ok=True)
    name = f"{m['image'][:-4]}_{i}.jpg"
    cv2.imwrite(os.path.join(d, name), crop, [cv2.IMWRITE_JPEG_QUALITY, 95])
    log.writerow([m["image"], fmt(box), f"{conf:.2f}", "etiquetada", person or "", cls, split,
                  "" if dist is None else f"{dist:.2f}", name])
    stats["etiquetada"] = stats.get("etiquetada", 0) + 1


if __name__ == "__main__":
    main()
