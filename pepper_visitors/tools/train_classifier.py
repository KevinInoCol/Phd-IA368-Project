#!/usr/bin/env python3
"""Fine-tuning del clasificador de visitantes y comparación con una línea base sin red.

    <venv de yolo_ros>/bin/python tools/train_classifier.py <dataset> <results> [--epochs 30]

<dataset> es la salida de label_dataset.py ({train,val,test}/<clase>/*.jpg y labels.csv).

  1. Línea base: histograma HSV (color de la ropa) de cada recorte + centroide más cercano por
     clase, calculado en train. Es lo que se puede hacer sin entrenar ninguna red.
  2. yolo11n-cls preentrenado en ImageNet, fine-tuning sobre train, selección con val.

Ambos se evalúan en test (personas que no están en train). Escribe en <results>:
metrics.json (exactitud global, por clase y por distancia, matrices de confusión) y copia
los pesos a pepper_visitors/models/visitor_cls.pt.
"""

import argparse
import csv
import glob
import json
import os
import shutil

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
WEIGHTS_OUT = os.path.join(HERE, "..", "models", "visitor_cls.pt")
BANDS = [(0, 2), (2, 3.5), (3.5, 99)]


def items(root, split):
    out = []
    for cls in sorted(os.listdir(os.path.join(root, split))):
        out += [(f, cls) for f in sorted(glob.glob(os.path.join(root, split, cls, "*.jpg")))]
    return out


def hsv_hist(path):
    hsv = cv2.cvtColor(cv2.imread(path), cv2.COLOR_BGR2HSV)
    h = cv2.calcHist([hsv], [0, 1, 2], None, [12, 4, 4], [0, 180, 0, 256, 0, 256]).ravel()
    return h / (h.sum() + 1e-9)


def report(y_true, y_pred, classes, dists):
    cm = np.zeros((len(classes), len(classes)), int)
    for t, p in zip(y_true, y_pred):
        cm[classes.index(t), classes.index(p)] += 1
    ok = np.array([t == p for t, p in zip(y_true, y_pred)])
    per_class = {c: float(cm[i, i] / max(1, cm[i].sum())) for i, c in enumerate(classes)}
    per_band = {}
    for lo, hi in BANDS:
        m = np.array([d is not None and lo <= d < hi for d in dists])
        if m.any():
            per_band[f"{lo}-{hi if hi < 99 else ''}"] = {"acc": float(ok[m].mean()), "n": int(m.sum())}
    return {"acc": float(ok.mean()), "n": len(ok), "per_class": per_class, "per_distance": per_band,
            "confusion": cm.tolist(), "classes": classes}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dataset")
    ap.add_argument("results")
    ap.add_argument("--epochs", type=int, default=30)
    ap.add_argument("--imgsz", type=int, default=128)
    ap.add_argument("--pretrained", default=os.path.expanduser("~/pepper_ws/weights/yolo11n-cls.pt"))
    args = ap.parse_args()
    os.makedirs(args.results, exist_ok=True)

    with open(os.path.join(args.dataset, "labels.csv")) as f:
        dist = {r["crop"]: float(r["dist"]) for r in csv.DictReader(f)
                if r["decision"] == "etiquetada" and r["dist"]}

    def dist_of(path):
        return dist.get(os.path.basename(path))

    train, test = items(args.dataset, "train"), items(args.dataset, "test")
    classes = sorted({c for _, c in train})
    counts = {s: {c: len(glob.glob(os.path.join(args.dataset, s, c, "*.jpg"))) for c in classes}
              for s in ("train", "val", "test")}
    print("recortes:", counts)
    dists = [dist_of(f) for f, _ in test]

    # 1. Línea base: color
    feats = {c: np.mean([hsv_hist(f) for f, k in train if k == c], axis=0) for c in classes}
    base_pred = [min(classes, key=lambda c: np.abs(hsv_hist(f) - feats[c]).sum()) for f, _ in test]
    baseline = report([c for _, c in test], base_pred, classes, dists)
    print(f"línea base (color): {baseline['acc']:.3f}")

    # 2. Fine-tuning
    from ultralytics import YOLO
    model = YOLO(args.pretrained)
    model.train(data=args.dataset, epochs=args.epochs, imgsz=args.imgsz, batch=64, device=0,
                project=args.results, name="train", exist_ok=True, plots=True, verbose=False, seed=0)
    best = os.path.join(args.results, "train", "weights", "best.pt")
    tuned = YOLO(best)
    pred = []
    for f, _ in test:
        r = tuned(f, verbose=False, imgsz=args.imgsz)[0]
        pred.append(r.names[int(r.probs.top1)])
    finetuned = report([c for _, c in test], pred, classes, dists)
    print(f"yolo11n-cls fine-tuned: {finetuned['acc']:.3f}")

    os.makedirs(os.path.dirname(WEIGHTS_OUT), exist_ok=True)
    shutil.copy(best, WEIGHTS_OUT)
    with open(os.path.join(args.results, "metrics.json"), "w") as f:
        json.dump({"counts": counts, "imgsz": args.imgsz, "epochs": args.epochs,
                   "baseline_color": baseline, "yolo11n_cls": finetuned}, f, indent=2)
    print(json.dumps({"baseline": baseline["per_class"], "finetuned": finetuned["per_class"]}, indent=1))


if __name__ == "__main__":
    main()
