"""Escucha el micrófono del PC y avisa cuando un visitante llama a Pepper.

    ros2 run pepper_visitors voice_listener                      # micrófono por defecto
    ros2 run pepper_visitors voice_listener --ros-args -p device:=<node.name de PipeWire>
    ros2 run pepper_visitors voice_listener --ros-args -p device:=<sumidero> -p capture_sink:=true
        # escucha lo que suena en un sumidero (su monitor) en vez de un micrófono: pruebas con
        # audio grabado o sintetizado (tools/voice_test.py)
    ros2 run pepper_visitors voice_listener --ros-args -p source:=topic
        # sin micrófono del PC: sólo frases que llegan por /pepper/voice/audio (la app del
        # teléfono, vía pepper_app/bridge); `source:=both` escucha las dos cosas

Corre con el entorno ~/pepper_ws/venvs/voice (faster-whisper en la GPU); el ejecutable
`voice_listener` instalado es un envoltorio que lo usa.

  1. pw-record entrega el audio (16 kHz, mono) del micrófono elegido.
  2. Segmentación por energía: el ruido de fondo se mide al arrancar; una frase empieza cuando
     la energía lo supera claramente y termina tras `silence_end` s de silencio.
  3. Whisper (large-v3-turbo) transcribe la frase en español, con su filtro de voz (VAD) para
     no "alucinar" texto en silencios.

Publica /pepper/voice/transcript (cada frase) y /pepper/visitor_call (std_msgs/String con el
texto) cuando la frase contiene una palabra de llamada (`keywords`: pepper, hola, oye...).
Una frase llegada por /pepper/voice/audio (std_msgs/Int16MultiArray, 16 kHz mono, ya cortada)
siempre tiene respuesta en /pepper/voice/transcript: vacía si no se entendió nada, y nunca sale
por /pepper/visitor_call: la responde el chat de la app (Gemini), que no debe competir con
visitor_approach por la base y la cabeza. (La pestaña Visitante de la app sí llama: el bridge
publica el texto en /pepper/visitor_call.)
"""

import collections
import queue
import subprocess
import threading
import time
import unicodedata

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, String

RATE = 16000
CHUNK = 480  # 30 ms
# Frases que Whisper inventa con ruido o silencio (vienen de subtítulos de su entrenamiento)
HALLUCINATIONS = ("gracias por ver", "suscribete", "subtitulos", "amara.org")
# ...y frases sueltas que salen solas con ruido (se comparan con la frase entera, sin puntuación)
NOISE_ONLY = {"gracias", "amen", "muchas gracias", "aplausos", "musica", "risas"}


def normalize(text):
    t = unicodedata.normalize("NFD", text.lower())
    return "".join(c for c in t if unicodedata.category(c) != "Mn")


class VoiceListener(Node):
    def __init__(self):
        super().__init__("voice_listener")
        self.source = self.declare_parameter("source", "mic").value  # mic | topic | both
        self.device = self.declare_parameter("device", "").value
        self.capture_sink = self.declare_parameter("capture_sink", False).value
        self.model_name = self.declare_parameter("model", "large-v3-turbo").value
        self.language = self.declare_parameter("language", "es").value
        self.keywords = self.declare_parameter(
            "keywords", ["pepper", "peper", "pepe", "hola", "oye", "disculpa", "perdon", "ven",
                         "ayuda", "pregunta", "robot"]).value
        self.factor = self.declare_parameter("energy_factor", 3.0).value
        self.min_rms = self.declare_parameter("min_rms", 0.004).value
        self.silence_end = self.declare_parameter("silence_end", 0.8).value
        self.max_len = self.declare_parameter("max_utterance", 10.0).value
        # Palabras que Whisper debe esperar (sin esto, "Pepper" sale como "Pepe" o "Espera")
        self.hotwords = self.declare_parameter("hotwords", "Pepper").value

        self.pub_text = self.create_publisher(String, "/pepper/voice/transcript", 10)
        self.pub_call = self.create_publisher(String, "/pepper/visitor_call", 10)

        from faster_whisper import WhisperModel
        t = time.time()
        self.model = WhisperModel(self.model_name, device="cuda", compute_type="float16",
                                  download_root=self._weights_dir())
        self.get_logger().info(f"Whisper {self.model_name} cargado en {time.time() - t:.1f} s")

        self.segments = queue.Queue()
        if self.source in ("mic", "both"):
            threading.Thread(target=self.capture, daemon=True).start()
        if self.source in ("topic", "both"):
            self.create_subscription(Int16MultiArray, "/pepper/voice/audio", self.audio_cb, 10)
        threading.Thread(target=self.transcribe_loop, daemon=True).start()

    @staticmethod
    def _weights_dir():
        import os
        return os.path.expanduser("~/pepper_ws/weights/whisper")

    def audio_cb(self, msg):
        audio = np.asarray(msg.data, np.int16).astype(np.float32) / 32768.0
        self.segments.put((audio[: int(self.max_len * RATE)], True))

    def capture(self):
        cmd = ["pw-record", "--rate", str(RATE), "--channels", "1", "--format", "s16"]
        if self.device:
            cmd += ["--target", self.device]
        if self.capture_sink:
            cmd += ["-P", "{ stream.capture.sink=true }"]
        proc = subprocess.Popen(cmd + ["-"], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        pre = collections.deque(maxlen=int(0.3 * RATE / CHUNK))  # 0.3 s antes del inicio
        floor, n_floor = 0.0, 0
        speech, silent, buf = False, 0, []
        while rclpy.ok():
            raw = proc.stdout.read(CHUNK * 2)
            if len(raw) < CHUNK * 2:
                self.get_logger().error("pw-record se detuvo")
                return
            x = np.frombuffer(raw, np.int16).astype(np.float32) / 32768.0
            rms = float(np.sqrt(np.mean(x * x)))
            if n_floor < int(1.0 * RATE / CHUNK):  # primer segundo: ruido de fondo
                floor = (floor * n_floor + rms) / (n_floor + 1)
                n_floor += 1
                if n_floor == int(1.0 * RATE / CHUNK):
                    self.get_logger().info(f"ruido de fondo {floor:.4f}; escuchando")
                continue
            loud = rms > max(self.min_rms, self.factor * floor)
            if not speech:
                pre.append(x)
                if loud:
                    speech, silent, buf = True, 0, list(pre)
                else:
                    floor = 0.995 * floor + 0.005 * rms  # el ruido de fondo se adapta despacio
                continue
            buf.append(x)
            silent = 0 if loud else silent + 1
            dur = len(buf) * CHUNK / RATE
            if silent * CHUNK / RATE >= self.silence_end or dur >= self.max_len:
                if dur - silent * CHUNK / RATE >= 0.3:  # al menos 0.3 s de voz
                    self.segments.put((np.concatenate(buf), False))
                speech, pre = False, collections.deque(maxlen=pre.maxlen)

    def transcribe_loop(self):
        while rclpy.ok():
            audio, reply_always = self.segments.get()
            t = time.time()
            segs, _ = self.model.transcribe(audio, language=self.language, vad_filter=True, beam_size=5,
                                            hotwords=self.hotwords or None)
            text = " ".join(s.text.strip() for s in segs).strip()
            norm = normalize(text)
            plain = " ".join("".join(c if c.isalnum() else " " for c in norm).split())
            if not plain or any(h in norm for h in HALLUCINATIONS) or plain in NOISE_ONLY:
                if reply_always:
                    self.pub_text.publish(String(data=""))
                continue
            words = set(plain.split())
            # reply_always = frase de la app: la atiende su chat, no es una llamada de visitante
            call = not reply_always and bool(words & {normalize(k) for k in self.keywords})
            self.get_logger().info(f'"{text}" ({len(audio) / RATE:.1f} s de audio, '
                                   f'{time.time() - t:.2f} s){"  → LLAMADA" if call else ""}')
            self.pub_text.publish(String(data=text))
            if call:
                self.pub_call.publish(String(data=text))


def main():
    rclpy.init()
    node = VoiceListener()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
