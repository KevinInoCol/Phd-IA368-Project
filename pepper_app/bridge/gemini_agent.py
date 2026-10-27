#!/usr/bin/env python3
"""
Gemini Flash Agent with Function Calling for Pepper ROS 2 Bridge.
Uses Google Cloud Vertex AI only (OAuth2 via gcloud Application Default Credentials).
"""

import base64
import os
import json
import time
import shutil
import subprocess
import urllib.request
import urllib.error
import numpy as np

# El entorno de pepper_museum (modelo "ISCAS Museum", museo de tecnología del software): salas y
# temas sacados de los carteles de sus texturas y del mapa museum_hokuyo (~19 x 22 m).
MUSEUM_DESCRIPTION = (
    "Te mueves por un museo de tecnología del software (el Museo ISCAS, del Instituto de Software "
    "de la Academia China de Ciencias), una planta de unos 19 x 22 m con paredes grises y suelo claro. "
    "En las paredes hay paneles y carteles informativos (muchos en chino) sobre: introducción al "
    "museo, el desarrollo de los computadores, historia del software en el mundo, sistemas operativos, "
    "procesamiento de texto en chino, métodos y entornos de desarrollo de software, ciencia de la "
    "computación y teoría del software, y el desarrollo de la industria del software. "
    "El recorrido es un gran pasillo que rodea un bloque central de exposición; al oeste hay salas "
    "laterales y una estructura circular, y al norte una sala amplia. "
    "Hay visitantes en el museo: unos están quietos mirando las exposiciones y otros caminan. "
    "Puedes contar los temas del museo en general, pero no sabes qué tema está en cada zona: no "
    "digas cuál hay cerca de ti salvo que lo leas en tu imagen, y no inventes detalles de un cartel "
    "concreto que no puedas leer. "
)

# Charla con un visitante (pestaña Visitante de la app), tras acercarse a él con visitor_approach
VISITOR_INSTRUCTION = (
    "Ahora estás frente a un visitante del museo: te llamó, lo buscaste con la cámara y te acercaste "
    "a él, y ya lo saludaste («Hola, ¿en qué te puedo ayudar?»): no vuelvas a saludar. Conversa "
    "con él como guía del museo: responde sus preguntas, cuéntale de los temas del "
    "museo y pregúntale qué le interesa. Lo tienes delante en la imagen de tu cámara: puedes "
    "comentar lo que ves de él si viene al caso. No te muevas: quédate de frente a él. "
    "Cuando el visitante dé por terminada la charla (dice que no tiene más preguntas, se despide, "
    "o agradece la ayuda sin pedir nada más), llama a end_conversation y despídete con una frase "
    "breve y amable. Si agradece pero sigue preguntando, responde y no la termines. "
)
# Despedidas para cuando Gemini no responde (parser local)
FAREWELL_WORDS = ("gracias", "adios", "adiós", "chao", "chau", "hasta luego", "eso es todo",
                  "nada mas", "nada más", "ninguna", "no tengo mas", "no tengo más")

MAX_TOOL_ROUNDS = 3  # llamadas a Gemini por mensaje como máximo
SETTLE_TIME = 1.0  # s tras un movimiento antes de tomar la imagen nueva (la cámara va a ~2.5 fps)


class GeminiAgent:
    def __init__(self, bridge_node=None):
        self.bridge_node = bridge_node
        self.last_error = None

        # Load environment values
        self.env_vars = self._load_all_env_vars()
        self.model = self.env_vars.get("GEMINI_MODEL", "gemini-3.5-flash")
        self.thinking_level = self.env_vars.get("GEMINI_THINKING_LEVEL", "LOW")
        self.gcp_project_id = self.env_vars.get("GCP_PROJECT_ID")
        # gemini-3.x models are served from the 'global' location
        self.gcp_location = self.env_vars.get("GCP_LOCATION", "global")
        self.use_vertex = True

        # Token cache for Vertex AI
        self._cached_token = None
        self._token_expiry = 0.0

    def is_configured(self):
        return bool(self.gcp_project_id)

    def _vertex_url(self):
        host = ("aiplatform.googleapis.com" if self.gcp_location == "global"
                else f"{self.gcp_location}-aiplatform.googleapis.com")
        return (f"https://{host}/v1/projects/{self.gcp_project_id}/locations/"
                f"{self.gcp_location}/publishers/google/models/{self.model}:generateContent")


    def _load_all_env_vars(self):
        vars_dict = dict(os.environ)
        paths = [
            "/home/kevao/Robotica/2026-IA368II-Proyecto-Robotica/.env",
            "/home/kevao/Robotica/2026-IA368II-Proyecto-Robotica/pepper_app/.env"
        ]
        for p in paths:
            if os.path.exists(p):
                try:
                    with open(p, "r", encoding="utf-8") as f:
                        for line in f:
                            line = line.strip()
                            if line and not line.startswith("#") and "=" in line:
                                parts = line.split("=", 1)
                                k = parts[0].strip()
                                v = parts[1].strip("\"' ")
                                if v:
                                    vars_dict[k] = v
                except Exception:
                    pass
        return vars_dict

    def _get_vertex_token(self):
        now = time.time()
        if self._cached_token and now < self._token_expiry:
            return self._cached_token

        # Find gcloud binary
        gcloud_bin = shutil.which("gcloud") or os.path.expanduser("~/google-cloud-sdk/bin/gcloud")
        if not os.path.exists(gcloud_bin):
            return None

        # Try print-access-token (or application-default)
        for cmd in [
            [gcloud_bin, "auth", "application-default", "print-access-token"],
            [gcloud_bin, "auth", "print-access-token"]
        ]:
            try:
                proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=5)
                token = proc.stdout.strip()
                if proc.returncode == 0 and len(token) > 20:
                    self._cached_token = token
                    self._token_expiry = now + 1800 # 30 min cache
                    return token
            except Exception:
                continue

        return None

    def _visitor_tools_spec(self):
        return [{"functionDeclarations": [{
            "name": "end_conversation",
            "description": "Termina la charla con el visitante cuando él la da por terminada "
                           "(no tiene más preguntas, se despide o agradece la ayuda).",
            "parameters": {"type": "OBJECT", "properties": {}},
        }]}]

    def _get_tools_spec(self):
        return [
            {
                "functionDeclarations": [
                    {
                        "name": "move_robot",
                        "description": "Mueve la base móvil del robot Pepper en una dirección determinada.",
                        "parameters": {
                            "type": "OBJECT",
                            "properties": {
                                "direction": {
                                    "type": "STRING",
                                    "enum": ["forward", "backward", "left", "right"],
                                    "description": "Dirección del movimiento: 'forward' para avanzar, 'backward' para retroceder, 'left' para girar a la izquierda, 'right' para girar a la derecha."
                                },
                                "duration_seconds": {
                                    "type": "NUMBER",
                                    "description": "Duración del movimiento en segundos (ej: 1.5)."
                                }
                            },
                            "required": ["direction"]
                        }
                    },
                    {
                        "name": "move_head",
                        "description": "Mueve los servos del cuello/cabeza de Pepper (inclinación y giro).",
                        "parameters": {
                            "type": "OBJECT",
                            "properties": {
                                "yaw_deg": {
                                    "type": "NUMBER",
                                    "description": "Giro horizontal de la cabeza en grados (-80 a +80). 0 es al frente."
                                },
                                "pitch_deg": {
                                    "type": "NUMBER",
                                    "description": "Inclinación vertical en grados (-25 hacia arriba, +25 hacia abajo). 0 es nivelado."
                                }
                            }
                        }
                    },
                    {
                        "name": "stop_robot",
                        "description": "Detiene inmediatamente cualquier movimiento del robot (parada de emergencia).",
                        "parameters": {
                            "type": "OBJECT",
                            "properties": {}
                        }
                    },
                    {
                        "name": "get_robot_pose",
                        "description": "Obtiene la posición actual (X, Y en metros y orientación Yaw) de Pepper en el mapa del museo.",
                        "parameters": {
                            "type": "OBJECT",
                            "properties": {}
                        }
                    }
                ]
            }
        ]

    def _execute_tool(self, name, args):
        if not self.bridge_node:
            return {"status": "no_bridge_node"}

        if name == "move_robot":
            direction = args.get("direction", "forward")
            duration = float(args.get("duration_seconds", 1.5))
            duration = max(0.5, min(duration, 4.0))

            if direction == "forward":
                self.bridge_node._execute_timed_move(0.25, 0.0, duration)
            elif direction == "backward":
                self.bridge_node._execute_timed_move(-0.2, 0.0, duration)
            elif direction == "left":
                self.bridge_node._execute_timed_move(0.0, 0.5, duration)
            elif direction == "right":
                self.bridge_node._execute_timed_move(0.0, -0.5, duration)
            return {"status": "executed", "direction": direction, "duration": duration}

        elif name == "move_head":
            yaw_deg = float(args.get("yaw_deg", 0.0))
            pitch_deg = float(args.get("pitch_deg", 0.0))
            yaw_rad = np.clip(np.radians(yaw_deg), -1.5, 1.5)
            pitch_rad = np.clip(np.radians(pitch_deg), -0.5, 0.5)
            self.bridge_node.publish_head(yaw_rad, pitch_rad)
            return {"status": "executed", "yaw_deg": yaw_deg, "pitch_deg": pitch_deg}

        elif name == "stop_robot":
            self.bridge_node.publish_cmd_vel(0.0, 0.0, 0.0)
            return {"status": "stopped"}

        elif name == "end_conversation":
            return {"status": "ended"}

        elif name == "get_robot_pose":
            with self.bridge_node.state.lock:
                x = self.bridge_node.state.robot_x
                y = self.bridge_node.state.robot_y
                yaw_deg = float(self.bridge_node.state.robot_yaw * 180.0 / np.pi)
            return {"x": round(x, 2), "y": round(y, 2), "yaw_degrees": round(yaw_deg, 1)}

        return {"status": "unknown_function"}

    def _front_frame(self):
        """Último JPEG de la cámara frontal (sin las cajas de YOLO), o None."""
        state = getattr(self.bridge_node, "state", None)
        if state is None:
            return None
        with state.lock:
            return state.front_frame_jpeg

    def generate_response(self, user_text, conversation_history=None):
        """Chat del operador (pestaña Chat). Devuelve (texto, error)."""
        system_instruction = (
            "Eres Pepper, un robot humanoide de servicio y guía del museo en una simulación de ROS 2 Jazzy. "
            "Eres amable, simpático, servicial y curioso. "
            + MUSEUM_DESCRIPTION +
            "Tienes herramientas para mover tu cuerpo (move_robot), tu cabeza (move_head), frenar (stop_robot) "
            "y consultar tu posición (get_robot_pose). "
            "Con cada mensaje del usuario recibes la imagen actual de tu cámara frontal: úsala cuando "
            "te pregunten qué ves o necesites mirar algo, y describe sólo lo que de verdad aparece en ella. "
            "Cuando el usuario te pida una acción física, invoca la herramienta correspondiente. "
            "Tus respuestas deben ser breves, habladas y claras (máximo 1 o 2 oraciones en español), "
            "ya que se reproducirán por tu sintetizador de voz."
        )
        speech_text, executed_actions, err = self._converse(
            system_instruction, user_text, conversation_history, self._get_tools_spec())
        if err:
            return None, err
        if executed_actions and not speech_text:
            action_name = executed_actions[0][0]
            if action_name == "move_robot":
                speech_text = "¡En movimiento! Ejecutando tu indicación."
            elif action_name == "move_head":
                speech_text = "Moviendo la cabeza."
            elif action_name == "stop_robot":
                speech_text = "¡Detenido!"
            elif action_name == "get_robot_pose":
                pose = executed_actions[0][1]
                speech_text = f"Estoy en la posición X: {pose.get('x')} m, Y: {pose.get('y')} m."
        return speech_text or "¡Comprendido!", None

    def visitor_response(self, user_text, conversation_history=None):
        """Charla con el visitante (pestaña Visitante). Devuelve (texto, terminó, error); sin
        Gemini, un parser local reconoce las despedidas."""
        system_instruction = (
            "Eres Pepper, un robot humanoide guía del museo en una simulación de ROS 2 Jazzy. "
            "Eres amable, simpático y paciente. " + MUSEUM_DESCRIPTION + VISITOR_INSTRUCTION +
            "Tus respuestas deben ser breves, habladas y claras (máximo 2 o 3 oraciones en español), "
            "ya que se reproducirán por tu sintetizador de voz."
        )
        speech_text, executed_actions, err = self._converse(
            system_instruction, user_text, conversation_history, self._visitor_tools_spec())
        if err:
            t = user_text.lower()
            if any(w in t for w in FAREWELL_WORDS):
                return "¡Con gusto! Que disfrutes el resto del museo.", True, err
            return "Perdona, ahora no puedo pensar bien la respuesta. ¿Me lo repites?", False, err
        ended = any(name == "end_conversation" for name, _ in executed_actions)
        return speech_text or ("¡Hasta luego, que disfrutes el museo!" if ended else "¿Perdón?"), ended, None

    def _converse(self, system_instruction, user_text, conversation_history, tools):
        """Bucle de function calling con la imagen de la cámara. Devuelve (texto, acciones, error)."""
        headers = {"Content-Type": "application/json"}

        if not self.gcp_project_id:
            return "", [], "GCP_PROJECT_ID not set in .env"
        token = self._get_vertex_token()
        if not token:
            return "", [], "VERTEX_TOKEN_NOT_FOUND (Run: gcloud auth application-default login)"
        url = self._vertex_url()
        headers["Authorization"] = f"Bearer {token}"
        headers["x-goog-user-project"] = self.gcp_project_id

        contents = []
        if conversation_history:
            for item in conversation_history[-10:]:
                role = "user" if item.get("sender") == "user" else "model"
                contents.append({
                    "role": role,
                    "parts": [{"text": item.get("text", "")}]
                })

        user_parts = [{"text": user_text}]
        frame = self._front_frame()
        if frame:
            user_parts.insert(0, {"inlineData": {"mimeType": "image/jpeg",
                                                 "data": base64.b64encode(frame).decode("ascii")}})
        contents.append({
            "role": "user",
            "parts": user_parts
        })

        payload = {
            "contents": contents,
            "systemInstruction": {"parts": [{"text": system_instruction}]},
            "tools": tools,
            "generationConfig": {
                "thinkingConfig": {
                    "thinkingLevel": self.thinking_level
                }
            }
        }

        try:
            speech_text = ""
            executed_actions = []

            # Bucle de function calling: el resultado de cada herramienta vuelve al modelo para que
            # pueda responder con él (p. ej. decir la posición o qué hay en esa zona)
            for _ in range(MAX_TOOL_ROUNDS):
                req = urllib.request.Request(
                    url,
                    data=json.dumps(payload).encode("utf-8"),
                    headers=headers
                )
                with urllib.request.urlopen(req, timeout=20) as resp:
                    data = json.loads(resp.read().decode("utf-8"))

                candidates = data.get("candidates", [])
                if not candidates:
                    return "", executed_actions, "NO_CANDIDATE"

                content = candidates[0].get("content", {})
                parts = content.get("parts", [])

                speech_text = ""
                responses = []
                for part in parts:
                    if "functionCall" in part:
                        fc = part["functionCall"]
                        fn_name = fc.get("name")
                        res = self._execute_tool(fn_name, fc.get("args", {}))
                        executed_actions.append((fn_name, res))
                        responses.append({"functionResponse": {"name": fn_name, "response": res}})
                    if "text" in part and not part.get("thought"):  # el razonamiento no se dice
                        speech_text += part["text"].strip() + " "

                if not responses:
                    break
                # Si se movió, esperar a que termine y mandar la imagen nueva de la cámara
                moved = [res for name, res in executed_actions[-len(responses):]
                         if name in ("move_robot", "move_head")]
                if moved:
                    time.sleep(max(res.get("duration", 0.0) for res in moved) + SETTLE_TIME)
                    frame = self._front_frame()
                    if frame:
                        # Respuesta multimodal de Gemini 3: la imagen dentro del functionResponse
                        # (como parte suelta el modelo se confunde y suelta texto basura)
                        responses[-1]["functionResponse"]["parts"] = [{"inlineData": {
                            "mimeType": "image/jpeg",
                            "data": base64.b64encode(frame).decode("ascii")}}]
                # Las partes del modelo van tal cual: Gemini 3 exige devolver sus thoughtSignature
                contents.append({"role": "model", "parts": parts})
                contents.append({"role": "user", "parts": responses})

            return speech_text.strip(), executed_actions, None

        except urllib.error.HTTPError as e:
            err_msg = e.read().decode("utf-8")
            self.last_error = f"HTTP {e.code}: {err_msg[:200]}"
            return "", [], self.last_error
        except Exception as e:
            self.last_error = str(e)
            return "", [], self.last_error
