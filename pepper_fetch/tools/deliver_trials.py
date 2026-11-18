"""Pruebas de "¿me traes una manzana?" de punta a punta, por el bridge de la app (como el teléfono).

    python3 tools/deliver_trials.py --wavs llamada.wav pedido.wav gracias.wav --out results/fetch/entrega1

Necesita fetch_demo.launch.py, el bridge (pepper_app/bridge/pepper_bridge.py, :8080) y voice_listener
(source:=topic). Las tres frases son WAV (p. ej. de Piper): la llamada ("Hola Pepper, ven por favor"),
el pedido ("¿Me traes una manzana, por favor?") y la de tomarla ("Muchas gracias Pepper, ya la tomo").

En cada intento Pepper va con Nav2 a uno de los puntos de partida de fetch_trials.py; luego, por
POST /api/visitor_voice: llamada → (visitor_approach lo lleva hasta la persona quieta más cercana) →
pedido (Gemini llama a bring_object → fetch_object va a la mesa, coge la manzana y vuelve) → al
ofrecerla, la frase de tomarla. Guarda una línea JSON por intento (fases, tiempos, transcripciones) y
el registro de fetch_object (`--fetch-log`) para la distancia de entrega.
"""

import argparse
import json
import math
import os
import time
import urllib.request

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

from fetch_trials import STARTS

URL = "http://localhost:8080"


def post_wav(path, timeout=90):
    req = urllib.request.Request(URL + "/api/visitor_voice", data=open(path, "rb").read(),
                                 headers={"Content-Type": "audio/wav"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read())


def visitor():
    with urllib.request.urlopen(URL + "/api/visitor", timeout=3) as r:
        return json.loads(r.read())


def wait_phase(want, stop, timeout):
    """Espera a que la fase del bridge esté en `want` (o en `stop`: fallo). Devuelve (fase, s)."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            ph = visitor()["phase"]
        except Exception:
            ph = None
        if ph in want or ph in stop:
            return ph, round(time.time() - t0, 1)
        time.sleep(0.5)
    return "timeout", round(time.time() - t0, 1)


class Nav(Node):
    def __init__(self):
        super().__init__("deliver_trials")
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")

    def go(self, x, y, yaw_deg):
        g = NavigateToPose.Goal()
        g.pose = PoseStamped()
        g.pose.header.frame_id = "map"
        g.pose.pose.position.x, g.pose.pose.position.y = float(x), float(y)
        a = math.radians(yaw_deg)
        g.pose.pose.orientation.z, g.pose.pose.orientation.w = math.sin(a / 2), math.cos(a / 2)
        self.nav.wait_for_server()
        f = self.nav.send_goal_async(g)
        rclpy.spin_until_future_complete(self, f)
        r = f.result().get_result_async()
        rclpy.spin_until_future_complete(self, r, timeout_sec=180)
        return r.done() and r.result().status == GoalStatus.STATUS_SUCCEEDED


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--wavs", nargs=3, required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--fetch-log", default="", help="log_file de fetch_object (una línea por pedido)")
    ap.add_argument("--n", type=int, default=5)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    rclpy.init()
    nav = Nav()
    for i in range(a.n):
        sx, sy, syaw = STARTS[i % len(STARTS)]
        rec = {"n": i + 1, "start": [sx, sy, syaw]}
        rec["start_ok"] = nav.go(sx, sy, syaw)
        t0 = time.time()
        r = post_wav(a.wavs[0])
        rec["call_transcript"] = r.get("transcript")
        rec["reach"], rec["reach_s"] = wait_phase({"talking"}, {"ended", "nobody", "nav_failed", "error"}, 300)
        if rec["reach"] == "talking":
            r = post_wav(a.wavs[1])
            rec["ask_transcript"], rec["reply"] = r.get("transcript"), r.get("reply")
            rec["offer"], rec["offer_s"] = wait_phase({"offering"}, {"ended", "talking"}, 400)
            if rec["offer"] == "offering":
                time.sleep(3.0)
                r = post_wav(a.wavs[2])
                rec["take_transcript"], rec["taken"] = r.get("transcript"), r.get("taken", False)
                rec["end"], _ = wait_phase({"ended"}, set(), 120)
                st = visitor().get("status", {})
                rec["final_phase"] = st.get("phase")
        rec["total_s"] = round(time.time() - t0, 1)
        time.sleep(12.0)            # retract + reponer la manzana
        if a.fetch_log and os.path.exists(a.fetch_log):
            last = json.loads(open(a.fetch_log).read().strip().splitlines()[-1])
            rec["fetch"] = {k: last.get(k) for k in ("result", "taken_by_word", "handover_dist",
                                                     "deliver_nav_s", "duration_s", "lift_m", "deliver")}
        rec["success"] = rec.get("final_phase") == "delivered" and bool(rec.get("taken"))
        print(json.dumps(rec, ensure_ascii=False), flush=True)
        with open(os.path.join(a.out, "trials.jsonl"), "a") as f:
            f.write(json.dumps(rec, ensure_ascii=False) + "\n")
    nav.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
