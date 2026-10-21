#!/usr/bin/env python3
"""Prueba voice_listener sin micrófono: frases sintetizadas que suenan en un sumidero virtual.

    # 1. voice_listener escuchando el sumidero de pruebas (lo crea este script si no existe)
    ros2 run pepper_visitors voice_listener --ros-args -p device:=pepper_test_audio -p capture_sink:=true
    # 2. las frases (llamadas y no llamadas, dos voces), y la tabla de resultados
    python3 tools/voice_test.py [--voices es_ES-davefx-medium es_MX-claude-high] [--snr 10] [--out res.json]

Síntesis con Piper (entorno ~/pepper_ws/venvs/tts, voces en ~/pepper_ws/weights/piper; ver
README). Los WAV se guardan en datasets/voice_test/ y se reutilizan. Con --snr se les mezcla ruido
rosa (relación señal/ruido en dB) para simular la sala. Por frase: lo que transcribió Whisper
(/pepper/voice/transcript), si hubo llamada (/pepper/visitor_call) y la latencia desde el final
del audio hasta la llamada.
"""

import argparse
import json
import os
import subprocess
import threading
import time
import wave

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String

HERE = os.path.dirname(os.path.abspath(__file__))
CACHE = os.path.join(HERE, "..", "..", "datasets", "voice_test")
PIPER = os.path.expanduser("~/pepper_ws/venvs/tts/bin/piper")
VOICES_DIR = os.path.expanduser("~/pepper_ws/weights/piper")
SINK = "pepper_test_audio"

# (frase, ¿es una llamada?). Las que no son llamadas no llevan ninguna palabra de llamada.
PHRASES = [
    ("Hola Pepper, ¿me puedes contar algo de esta pieza?", True),
    ("Oye, robot, ven un momento por favor.", True),
    ("Disculpa, tengo una pregunta sobre el museo.", True),
    ("Pepper, ¿dónde está la salida?", True),
    ("Perdón, ¿me ayudas?", True),
    ("Ven aquí, Pepper.", True),
    ("Qué bonito está el museo hoy.", False),
    ("Mira esa escultura de allá.", False),
    ("Vamos a almorzar después de la visita.", False),
    ("Me gustan mucho los cuadros de esta sala.", False),
]


def ensure_sink():
    out = subprocess.run(["pw-cli", "ls", "Node"], capture_output=True, text=True).stdout
    if f'node.name = "{SINK}"' not in out:
        subprocess.run(["pw-cli", "create-node", "adapter",
                        "{ factory.name=support.null-audio-sink node.name=" + SINK +
                        " node.description=Audio-de-pruebas-de-Pepper media.class=Audio/Sink"
                        " audio.position=[MONO] object.linger=true }"], capture_output=True, check=True)
        time.sleep(1.0)


def synth(text, voice, idx):
    os.makedirs(CACHE, exist_ok=True)
    path = os.path.join(CACHE, f"{voice}_{idx:02d}.wav")
    if not os.path.exists(path):
        subprocess.run([PIPER, "-m", os.path.join(VOICES_DIR, voice + ".onnx"), "-f", path],
                       input=text.encode(), capture_output=True, check=True)
    return path


def with_noise(path, snr_db, seed):
    """Copia del WAV con ruido rosa a la SNR pedida, más 0.5 s de ruido antes y después."""
    with wave.open(path) as w:
        rate = w.getframerate()
        x = np.frombuffer(w.readframes(w.getnframes()), np.int16).astype(np.float32) / 32768
    pad = np.zeros(int(0.5 * rate), np.float32)
    x = np.concatenate([pad, x, pad])
    rng = np.random.default_rng(seed)
    white = rng.standard_normal(len(x))
    f = np.fft.rfftfreq(len(x))
    f[0] = f[1]
    pink = np.fft.irfft(np.fft.rfft(white) / np.sqrt(f), len(x)).astype(np.float32)
    speech = x[np.abs(x) > 0.01]
    p_sig = np.mean(speech ** 2) if len(speech) else np.mean(x ** 2)
    pink *= np.sqrt(p_sig / (10 ** (snr_db / 10)) / np.mean(pink ** 2))
    y = np.clip(x + pink, -1, 1)
    out = path.replace(".wav", f"_snr{snr_db:g}.wav")
    with wave.open(out, "w") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(rate)
        w.writeframes((y * 32767).astype(np.int16).tobytes())
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--voices", nargs="+", default=["es_ES-davefx-medium", "es_MX-claude-high"])
    ap.add_argument("--snr", type=float, default=None, help="mezclar ruido rosa a esta SNR [dB]")
    ap.add_argument("--wait", type=float, default=4.0, help="s de espera tras cada frase")
    ap.add_argument("--out", default=None, help="resultados en JSON")
    args = ap.parse_args()

    ensure_sink()
    rclpy.init()
    node = rclpy.create_node("voice_test")
    events = []   # (tipo, texto, instante)
    lock = threading.Lock()

    def cb(kind):
        def f(msg):
            with lock:
                events.append((kind, msg.data, time.time()))
        return f
    node.create_subscription(String, "/pepper/voice/transcript", cb("transcript"), 10)
    node.create_subscription(String, "/pepper/visitor_call", cb("call"), 10)
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    spin = threading.Thread(target=ex.spin, daemon=True)
    spin.start()
    time.sleep(2.0)   # voice_listener mide el ruido de fondo en su primer segundo

    rows = []
    for voice in args.voices:
        for i, (text, is_call) in enumerate(PHRASES):
            wav = synth(text, voice, i)
            if args.snr is not None:
                wav = with_noise(wav, args.snr, i)
            with lock:
                events.clear()
            subprocess.run(["pw-play", "--target", SINK, wav], check=True)
            t_end = time.time()
            time.sleep(args.wait)
            with lock:
                ev = list(events)
            transcript = " | ".join(t for k, t, _ in ev if k == "transcript")
            calls = [t for k, _, t in ev if k == "call"]
            rows.append({"voice": voice, "text": text, "is_call": is_call, "transcript": transcript,
                         "called": bool(calls),
                         "latency_s": round(calls[0] - t_end, 2) if calls else None})
            r = rows[-1]
            ok = r["called"] == is_call
            print(f"{'✓' if ok else '✗'} [{voice.split('-')[0]}] {text!r} → {transcript!r}"
                  f"{' · llamada %.2f s' % r['latency_s'] if calls else ''}", flush=True)

    tp = sum(r["called"] and r["is_call"] for r in rows)
    fn = sum(not r["called"] and r["is_call"] for r in rows)
    fp = sum(r["called"] and not r["is_call"] for r in rows)
    lat = [r["latency_s"] for r in rows if r["latency_s"] is not None]
    print(f"\nLlamadas detectadas {tp}/{tp + fn}; falsas llamadas {fp}/{sum(not r['is_call'] for r in rows)}; "
          f"latencia media {np.mean(lat):.2f} s" if lat else "\nNinguna llamada detectada")
    if args.out:
        with open(args.out, "w") as f:
            json.dump({"snr_db": args.snr, "rows": rows}, f, indent=2, ensure_ascii=False)
    ex.shutdown()
    spin.join(timeout=2.0)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
