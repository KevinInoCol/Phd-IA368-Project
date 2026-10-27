#!/usr/bin/env python3
"""
Pepper Mobile App Bridge Server
Provides MJPEG camera streaming and HTTP/REST control for the Pepper Flutter App.
"""

import io
import sys
import wave
import json
import time
import threading
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import Int16MultiArray, String
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import numpy as np

from gemini_agent import GeminiAgent


class AppBridgeState:
    def __init__(self):
        self.lock = threading.Lock()
        self.front_frame_jpeg = None
        self.bottom_frame_jpeg = None
        self.yolo_frame_jpeg = None  # /yolo/dbg_image: cámara frontal con las cajas de YOLO
        self.front_condition = threading.Condition(self.lock)
        self.bottom_condition = threading.Condition(self.lock)
        self.yolo_condition = threading.Condition(self.lock)
        
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.last_speech = ""
        self.last_speech_time = 0.0
        self.front_frame_count = 0
        self.front_fps = 0.0
        self.last_fps_calc = time.time()
        self.current_fps_counter = 0
        self.chat_history = [
            {
                "sender": "pepper",
                "text": "¡Hola! Soy Pepper, tu guía del museo. Puedes pedirme que me mueva, pregúntame dónde estoy o darme cualquier indicación.",
                "time": time.time()
            }
        ]

        # Modo visitante (3.ª pestaña): lo atiende visitor_approach, no Gemini
        self.visitor_phase = 'idle'
        self.visitor_status = {}
        self.visitor_since = 0.0   # último cambio de fase
        self.visitor_log = []   # {'kind': 'visitor'|'pepper'|'phase', 'text', 'phase', 'time'}
        self.visitor_chat = []  # charla de la llamada en curso, para Gemini ({'sender', 'text'})

    def add_visitor_log(self, kind, text, phase=None):
        with self.lock:
            self.visitor_log.append({'kind': kind, 'text': text, 'phase': phase, 'time': time.time()})
            del self.visitor_log[:-40]

    def add_chat(self, sender, text):
        with self.lock:
            self.chat_history.append({
                "sender": sender,
                "text": text,
                "time": time.time()
            })
            if len(self.chat_history) > 50:
                self.chat_history.pop(0)

    def generate_placeholder(self, text="Pepper Camera Waiting..."):
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        # Gradient background
        for y in range(480):
            img[y, :, 0] = int(20 + 20 * (y / 480))
            img[y, :, 1] = int(24 + 30 * (y / 480))
            img[y, :, 2] = int(35 + 45 * (y / 480))
        
        cv2.putText(img, text, (60, 240), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 220, 255), 2, cv2.LINE_AA)
        cv2.putText(img, "Connecting to Gazebo / ROS 2...", (120, 290), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (180, 180, 180), 1, cv2.LINE_AA)
        _, jpeg = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        return jpeg.tobytes()


class BridgeHTTPHandler(BaseHTTPRequestHandler):
    bridge_node = None
    state = None

    def _set_cors_headers(self):
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')

    def do_OPTIONS(self):
        self.send_response(200)
        self._set_cors_headers()
        self.end_headers()

    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path

        if path in ('/stream/front', '/stream/front/'):
            self.handle_mjpeg_stream('front')
        elif path in ('/stream/bottom', '/stream/bottom/'):
            self.handle_mjpeg_stream('bottom')
        elif path in ('/stream/yolo', '/stream/yolo/'):
            self.handle_mjpeg_stream('yolo')
        elif path == '/snapshot/front':
            self.handle_snapshot('front')
        elif path == '/api/status':
            self.handle_status()
        elif path == '/api/chat':
            self.handle_chat_history()
        elif path == '/api/visitor':
            self.handle_visitor_state()
        elif path == '/' or path == '/index.html':
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self._set_cors_headers()
            self.end_headers()
            html = f"""<!DOCTYPE html>
            <html>
            <head><title>Pepper ROS 2 Bridge</title><style>body{{font-family:sans-serif;background:#121212;color:#eee;padding:20px;text-align:center}}img{{max-width:90%;border-radius:8px;box-shadow:0 4px 12px rgba(0,0,0,0.5)}}</style></head>
            <body>
                <h1>Pepper Mobile App Bridge</h1>
                <p>Status: Active | Camera Stream below:</p>
                <img src="/stream/front" alt="Front Camera Stream" />
            </body>
            </html>"""
            self.wfile.write(html.encode('utf-8'))
        else:
            self.send_response(404)
            self.end_headers()
            self.wfile.write(b'Not Found')

    def do_POST(self):
        parsed = urlparse(self.path)
        path = parsed.path

        content_length = int(self.headers.get('Content-Length', 0))
        post_data = self.rfile.read(content_length) if content_length > 0 else b'{}'

        try:
            payload = json.loads(post_data.decode('utf-8'))
        except Exception:
            payload = {}

        if path == '/api/cmd_vel':
            linear_x = float(payload.get('linear_x', 0.0))
            linear_y = float(payload.get('linear_y', 0.0))
            angular_z = float(payload.get('angular_z', 0.0))
            if self.bridge_node:
                self.bridge_node.publish_cmd_vel(linear_x, linear_y, angular_z)
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self._set_cors_headers()
            self.end_headers()
            self.wfile.write(json.dumps({'status': 'ok'}).encode('utf-8'))

        elif path == '/api/head':
            yaw = float(payload.get('yaw', 0.0))
            pitch = float(payload.get('pitch', 0.0))
            if self.bridge_node:
                self.bridge_node.publish_head(yaw, pitch)
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self._set_cors_headers()
            self.end_headers()
            self.wfile.write(json.dumps({'status': 'ok'}).encode('utf-8'))

        elif path == '/api/say':
            text = str(payload.get('text', ''))
            if self.bridge_node and text:
                self.bridge_node.publish_say(text)
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self._set_cors_headers()
            self.end_headers()
            self.wfile.write(json.dumps({'status': 'ok'}).encode('utf-8'))

        elif path == '/api/voice':
            self.handle_voice(post_data)

        elif path == '/api/visitor_voice':
            self.handle_visitor_voice(post_data)

        elif path == '/api/visitor_end':
            if self.bridge_node:
                self.bridge_node.visitor_end('El operador terminó la charla.')
            self._send_json(200, {'status': 'ok'})

        elif path == '/api/chat':
            user_msg = str(payload.get('message') or payload.get('text') or '').strip()
            if not user_msg:
                self.send_response(400)
                self.end_headers()
                return

            self.state.add_chat("user", user_msg)
            reply = ""
            if self.bridge_node:
                reply = self.bridge_node.process_chat_command(user_msg)
            else:
                reply = f"Entendido: '{user_msg}'"
            self.state.add_chat("pepper", reply)

            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self._set_cors_headers()
            self.end_headers()
            with self.state.lock:
                history = list(self.state.chat_history)
            self.wfile.write(json.dumps({
                'status': 'ok',
                'reply': reply,
                'history': history
            }).encode('utf-8'))
        else:
            self.send_response(404)
            self.end_headers()

    def _send_json(self, code, obj):
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self._set_cors_headers()
        self.end_headers()
        self.wfile.write(json.dumps(obj).encode('utf-8'))

    def handle_voice(self, wav_bytes):
        """POST /api/voice: WAV grabado en el teléfono → texto (Whisper de voice_listener)."""
        try:
            audio = wav_to_pcm16k(wav_bytes)
        except Exception as e:
            self._send_json(400, {'status': 'error', 'error': f'WAV inválido: {e}'})
            return
        if not self.bridge_node:
            self._send_json(503, {'status': 'error', 'error': 'Bridge sin nodo ROS'})
            return
        text, err = self.bridge_node.transcribe(audio)
        if err:
            self._send_json(503, {'status': 'error', 'error': err})
        else:
            self._send_json(200, {'status': 'ok', 'transcript': text})

    def handle_visitor_voice(self, wav_bytes):
        """POST /api/visitor_voice: la frase la dice "un visitante del museo": se transcribe igual
        que /api/voice. Si Pepper ya llegó hasta él, es parte de la charla (Gemini responde); si no,
        va a /pepper/visitor_call (visitor_approach lo busca y se acerca)."""
        try:
            audio = wav_to_pcm16k(wav_bytes)
        except Exception as e:
            self._send_json(400, {'status': 'error', 'error': f'WAV inválido: {e}'})
            return
        if not self.bridge_node:
            self._send_json(503, {'status': 'error', 'error': 'Bridge sin nodo ROS'})
            return
        text, err = self.bridge_node.transcribe(audio)
        if err:
            self._send_json(503, {'status': 'error', 'error': err})
            return
        if not text:
            self._send_json(200, {'status': 'ok', 'transcript': '', 'sent': False})
            return
        with self.state.lock:
            talking = self.state.visitor_phase == 'talking'
        if talking:
            reply, ended = self.bridge_node.visitor_chat(text)
            self._send_json(200, {'status': 'ok', 'transcript': text, 'reply': reply, 'ended': ended})
            return
        err = self.bridge_node.visitor_call(text)
        self._send_json(503 if err else 200, {'status': 'error' if err else 'ok', 'transcript': text,
                                              'sent': not err, 'error': err})

    def handle_visitor_state(self):
        with self.state.lock:
            data = {'phase': self.state.visitor_phase, 'status': self.state.visitor_status,
                    'log': list(self.state.visitor_log)}
        self._send_json(200, data)

    def handle_chat_history(self):
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self._set_cors_headers()
        self.end_headers()
        with self.state.lock:
            history = list(self.state.chat_history)
        self.wfile.write(json.dumps({'history': history}).encode('utf-8'))

    def handle_status(self):
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self._set_cors_headers()
        self.end_headers()
        with self.state.lock:
            data = {
                'connected': True,
                'fps': round(self.state.front_fps, 1),
                'robot_x': round(self.state.robot_x, 3),
                'robot_y': round(self.state.robot_y, 3),
                'robot_yaw': round(self.state.robot_yaw, 3),
                'last_speech': self.state.last_speech,
                'has_front_camera': self.state.front_frame_jpeg is not None
            }
        self.wfile.write(json.dumps(data).encode('utf-8'))

    def handle_snapshot(self, camera='front'):
        with self.state.lock:
            frame = getattr(self.state, f'{camera}_frame_jpeg')
            if frame is None:
                frame = self.state.generate_placeholder(f"Pepper {camera.title()} Camera")
        self.send_response(200)
        self.send_header('Content-Type', 'image/jpeg')
        self.send_header('Content-Length', str(len(frame)))
        self._set_cors_headers()
        self.end_headers()
        self.wfile.write(frame)

    def handle_mjpeg_stream(self, camera='front'):
        self.send_response(200)
        self.send_header('Age', '0')
        self.send_header('Cache-Control', 'no-cache, private')
        self.send_header('Pragma', 'no-cache')
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
        self._set_cors_headers()
        self.end_headers()

        condition = getattr(self.state, f'{camera}_condition')
        placeholder = self.state.generate_placeholder(
            "Sin YOLO: lanza pepper_yolo.launch.py" if camera == 'yolo'
            else f"Pepper {camera.title()} Camera Waiting...")

        try:
            while True:
                with condition:
                    frame = getattr(self.state, f'{camera}_frame_jpeg')
                    if frame is None:
                        frame = placeholder
                        # Wait for a real frame or timeout after 0.5s to send keep-alive placeholder
                        condition.wait(timeout=0.5)
                    else:
                        condition.wait(timeout=1.0)
                        frame = getattr(self.state, f'{camera}_frame_jpeg')

                if frame is not None:
                    header = (
                        b'--FRAME\r\n'
                        b'Content-Type: image/jpeg\r\n'
                        + f'Content-Length: {len(frame)}\r\n\r\n'.encode('ascii')
                    )
                    self.wfile.write(header)
                    self.wfile.write(frame)
                    self.wfile.write(b'\r\n')
        except (BrokenPipeError, ConnectionResetError):
            pass

    def log_message(self, format, *args):
        # Mute normal stream GET logs to keep terminal readable
        if '/stream/' in args[0] or '/status' in args[0]:
            return
        super().log_message(format, *args)


def wav_to_pcm16k(data):
    """WAV PCM 16 bits (cualquier frecuencia y canales) → int16 mono a 16 kHz."""
    with wave.open(io.BytesIO(data)) as w:
        if w.getsampwidth() != 2:
            raise ValueError('se esperaba PCM de 16 bits')
        rate, channels = w.getframerate(), w.getnchannels()
        x = np.frombuffer(w.readframes(w.getnframes()), np.int16)
    x = x.reshape(-1, channels).mean(axis=1)
    if rate != VOICE_RATE:
        n = int(len(x) * VOICE_RATE / rate)
        x = np.interp(np.linspace(0, len(x) - 1, n), np.arange(len(x)), x)
    return x.astype(np.int16)


VOICE_RATE = 16000
VISITOR_ACTIVE = ('searching', 'approaching', 'facing')
VISITOR_STUCK = 180.0  # s sin cambio de fase: visitor_approach murió a mitad de una llamada
VOICE_TIMEOUT = 20.0  # s esperando a voice_listener


class PepperBridgeNode(Node):
    def __init__(self, state):
        super().__init__('pepper_mobile_bridge')
        self.state = state
        self.cv_bridge = CvBridge()

        # QoS for camera streaming (sensor data = best effort)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.front_sub = self.create_subscription(
            Image,
            '/pepper/camera/front/image_raw',
            self.front_camera_cb,
            sensor_qos
        )

        self.bottom_sub = self.create_subscription(
            Image,
            '/pepper/camera/bottom/image_raw',
            self.bottom_camera_cb,
            sensor_qos
        )

        # Imagen de depuración de yolo_ros (pepper_gazebo_plugin/pepper_yolo.launch.py de la base)
        self.yolo_sub = self.create_subscription(
            Image,
            '/yolo/dbg_image',
            self.yolo_image_cb,
            sensor_qos
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/pepper/odom_groundtruth',
            self.odom_cb,
            10
        )

        self.say_sub = self.create_subscription(
            String,
            '/pepper/say',
            self.say_cb,
            10
        )

        self.cmd_vel_pub = self.create_publisher(Twist, '/pepper/cmd_vel', 10)
        self.head_pub = self.create_publisher(JointTrajectory, '/pepper/Head_controller/command', 10)
        self.say_pub = self.create_publisher(String, '/pepper/say', 10)

        # Modo visitante: la frase va a visitor_approach como si la dijera alguien del museo
        self.visitor_call_pub = self.create_publisher(String, '/pepper/visitor_call', 10)
        self.create_subscription(String, '/pepper/visitor_status', self.visitor_status_cb, 10)

        # Voz del teléfono: la transcribe voice_listener (pepper_visitors, source:=topic)
        self.audio_pub = self.create_publisher(Int16MultiArray, '/pepper/voice/audio', 10)
        self.create_subscription(String, '/pepper/voice/transcript', self.transcript_cb, 10)
        self.voice_lock = threading.Lock()  # una frase a la vez
        self.voice_done = threading.Event()
        self.voice_text = ''

        # Gemini Agent (AI Reasoning & Function Calling)
        self.gemini_agent = GeminiAgent(self)
        if self.gemini_agent.is_configured():
            provider = "Vertex AI" if self.gemini_agent.use_vertex else "AI Studio"
            self.get_logger().info(
                f"Gemini Agent initialized: model={self.gemini_agent.model}, "
                f"thinking_level={self.gemini_agent.thinking_level}, provider={provider}"
            )
        else:
            self.get_logger().warn("Gemini Agent: No API Key or GCP config found, using local intent parser.")

        self.get_logger().info("Pepper Mobile Bridge Node initialized.")

    def front_camera_cb(self, msg):
        try:
            cv_img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Encode as JPEG
            _, jpeg = cv2.imencode('.jpg', cv_img, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            jpeg_bytes = jpeg.tobytes()

            with self.state.front_condition:
                self.state.front_frame_jpeg = jpeg_bytes
                self.state.front_frame_count += 1
                self.state.current_fps_counter += 1
                now = time.time()
                elapsed = now - self.state.last_fps_calc
                if elapsed >= 1.0:
                    self.state.front_fps = self.state.current_fps_counter / elapsed
                    self.state.current_fps_counter = 0
                    self.state.last_fps_calc = now
                self.state.front_condition.notify_all()
        except Exception as e:
            self.get_logger().error(f"Error converting front image: {e}")

    def yolo_image_cb(self, msg):
        try:
            cv_img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            _, jpeg = cv2.imencode('.jpg', cv_img, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            with self.state.yolo_condition:
                self.state.yolo_frame_jpeg = jpeg.tobytes()
                self.state.yolo_condition.notify_all()
        except Exception as e:
            self.get_logger().error(f"Error converting YOLO image: {e}")

    def bottom_camera_cb(self, msg):
        try:
            cv_img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            _, jpeg = cv2.imencode('.jpg', cv_img, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            jpeg_bytes = jpeg.tobytes()

            with self.state.bottom_condition:
                self.state.bottom_frame_jpeg = jpeg_bytes
                self.state.bottom_condition.notify_all()
        except Exception as e:
            self.get_logger().error(f"Error converting bottom image: {e}")

    def odom_cb(self, msg):
        with self.state.lock:
            self.state.robot_x = msg.pose.pose.position.x
            self.state.robot_y = msg.pose.pose.position.y
            # Extract yaw from quaternion
            q = msg.pose.pose.orientation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self.state.robot_yaw = np.arctan2(siny_cosp, cosy_cosp)

    def transcript_cb(self, msg):
        self.voice_text = msg.data
        self.voice_done.set()

    def transcribe(self, audio):
        """Manda la frase a voice_listener y espera su texto. Devuelve (texto, error)."""
        if self.audio_pub.get_subscription_count() == 0:
            return '', ('voice_listener no está corriendo: ros2 run pepper_visitors '
                        'voice_listener --ros-args -p source:=topic')
        with self.voice_lock:
            self.voice_done.clear()
            self.audio_pub.publish(Int16MultiArray(data=audio.tolist()))
            if not self.voice_done.wait(VOICE_TIMEOUT):
                return '', 'voice_listener no respondió a tiempo'
            self.get_logger().info(f'Voz del teléfono ({len(audio) / VOICE_RATE:.1f} s): '
                                   f'"{self.voice_text}"')
            return self.voice_text, None

    def say_cb(self, msg):
        with self.state.lock:
            self.state.last_speech = msg.data
            self.state.last_speech_time = time.time()
            visitor = self.state.visitor_phase in VISITOR_ACTIVE
        if visitor:  # lo dice visitor_approach mientras atiende la llamada
            self.state.add_visitor_log('pepper', msg.data)

    def visitor_call(self, text):
        """Publica la frase en /pepper/visitor_call. Devuelve un error o None."""
        if self.visitor_call_pub.get_subscription_count() == 0:
            return 'visitor_approach no está corriendo (visitors_demo.launch.py)'
        with self.state.lock:
            busy = (self.state.visitor_phase in VISITOR_ACTIVE
                    and time.time() - self.state.visitor_since < VISITOR_STUCK)
        if busy:
            return 'Pepper todavía está atendiendo la llamada anterior'
        self.state.add_visitor_log('visitor', text)
        with self.state.lock:
            self.state.visitor_phase = 'searching'   # hasta que llegue el primer estado
            self.state.visitor_chat = [{'sender': 'user', 'text': text}]
            self.state.visitor_since = time.time()
        self.visitor_call_pub.publish(String(data=text))
        self.get_logger().info(f'Llamada de visitante: "{text}"')
        return None

    def visitor_status_cb(self, msg):
        try:
            st = json.loads(msg.data)
        except ValueError:
            return
        phase = st.get('phase', '')
        if phase == 'busy':
            return
        with self.state.lock:
            # al llegar empieza la charla: las frases siguientes las responde Gemini
            self.state.visitor_phase = 'talking' if phase == 'reached' else phase
            self.state.visitor_status = st
            self.state.visitor_since = time.time()
            if phase == 'reached':
                self.state.visitor_chat.append({'sender': 'pepper', 'text': 'Hola, ¿en qué te puedo ayudar?'})
        self.state.add_visitor_log('phase', st.get('text', ''), phase)

    def visitor_chat(self, text):
        """Una frase del visitante durante la charla: responde Gemini (con la imagen de la cámara,
        sin herramientas de movimiento). Devuelve (respuesta, terminó)."""
        self.state.add_visitor_log('visitor', text)
        with self.state.lock:
            history = list(self.state.visitor_chat)
            self.state.visitor_chat.append({'sender': 'user', 'text': text})
        reply, ended, err = self.gemini_agent.visitor_response(text, history)
        if err:
            self.get_logger().warn(f'Gemini (visitante): {err}')
        with self.state.lock:
            self.state.visitor_chat.append({'sender': 'pepper', 'text': reply})
        self.state.add_visitor_log('pepper', reply)
        self.publish_say(reply)
        if ended:
            self.visitor_end('El visitante terminó la charla.')
        return reply, ended

    def visitor_end(self, text):
        with self.state.lock:
            if self.state.visitor_phase != 'talking':
                return
            self.state.visitor_phase = 'ended'
            self.state.visitor_status = {'phase': 'ended', 'text': text}
            self.state.visitor_since = time.time()
        self.state.add_visitor_log('phase', text, 'ended')

    def publish_cmd_vel(self, linear_x, linear_y, angular_z):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.linear.y = float(linear_y)
        msg.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(msg)

    def publish_head(self, yaw, pitch):
        msg = JointTrajectory()
        msg.joint_names = ['HeadYaw', 'HeadPitch']
        pt = JointTrajectoryPoint()
        pt.positions = [float(yaw), float(pitch)]
        pt.time_from_start = Duration(sec=0, nanosec=300000000) # 300 ms
        msg.points.append(pt)
        self.head_pub.publish(msg)

    def publish_say(self, text):
        msg = String()
        msg.data = text
        self.say_pub.publish(msg)

    def process_chat_command(self, text):
        # 1. Try Gemini Flash Agent with function calling
        if self.gemini_agent and self.gemini_agent.is_configured():
            # sin el último mensaje: es `text`, que el agente ya manda junto con la imagen
            with self.state.lock:
                history = list(self.state.chat_history[:-1])
            reply, err = self.gemini_agent.generate_response(text, history)
            if reply:
                self.publish_say(reply)
                return reply
            else:
                self.get_logger().warn(f"Gemini API returned: {err}. Using local fallback.")

        # 2. Local Fallback (Deterministic Intent Parser)
        return self.process_local_chat_command(text)

    def process_local_chat_command(self, text):
        t = text.lower().strip()

        # 1. Movimiento adelante
        if any(w in t for w in ["avanza", "adelante", "camina", "muévete adelante", "muevete adelante"]):
            self._execute_timed_move(0.25, 0.0, 1.5)
            reply = "¡Entendido! Avanzando un paso adelante."
        # 2. Movimiento atrás
        elif any(w in t for w in ["retrocede", "atrás", "atras", "muévete atrás", "muevete atras"]):
            self._execute_timed_move(-0.2, 0.0, 1.5)
            reply = "Retrocediendo un paso con precaución."
        # 3. Giro izquierda
        elif any(w in t for w in ["gira a la izquierda", "rota a la izquierda", "gira izquierda", "izquierda"]):
            self._execute_timed_move(0.0, 0.5, 1.8)
            reply = "Girando hacia mi izquierda."
        # 4. Giro derecha
        elif any(w in t for w in ["gira a la derecha", "rota a la derecha", "gira derecha", "derecha"]):
            self._execute_timed_move(0.0, -0.5, 1.8)
            reply = "Girando hacia mi derecha."
        # 5. Parada
        elif any(w in t for w in ["para", "alto", "detente", "stop", "frena", "parada"]):
            self.publish_cmd_vel(0.0, 0.0, 0.0)
            reply = "¡Parada inmediata! Me he detenido."
        # 6. Cabeza
        elif "mira arriba" in t or "arriba" in t:
            self.publish_head(0.0, -0.35)
            reply = "Mirando hacia arriba."
        elif "mira abajo" in t or "abajo" in t:
            self.publish_head(0.0, 0.35)
            reply = "Inclinando la cabeza hacia abajo."
        elif "mira al frente" in t or "al frente" in t or "centra" in t:
            self.publish_head(0.0, 0.0)
            reply = "Mirando directamente al frente."
        elif "mira a la izquierda" in t:
            self.publish_head(0.8, 0.0)
            reply = "Girando mi cabeza hacia la izquierda."
        elif "mira a la derecha" in t:
            self.publish_head(-0.8, 0.0)
            reply = "Girando mi cabeza hacia la derecha."
        # 7. Posición / Dónde estás
        elif any(w in t for w in ["dónde estás", "donde estas", "dónde te encuentras", "posicion", "posición", "coordenadas", "donde te encuentras"]):
            with self.state.lock:
                x = self.state.robot_x
                y = self.state.robot_y
                yaw_deg = self.state.robot_yaw * 180.0 / np.pi
            reply = f"Mi posición actual en el mapa es X: {x:.2f} m, Y: {y:.2f} m, con orientación de {yaw_deg:.1f}°."
        # 8. Visión / Cámara / Qué ves
        elif any(w in t for w in ["qué ves", "que ves", "cámara", "camara", "visión", "vision", "observas"]):
            reply = "Ahora no puedo analizar mi cámara (Gemini no responde). Mira la vista YOLO de la app."
        # 9. Visitantes / Museo
        elif any(w in t for w in ["visitante", "personas", "busca personas", "gente", "acércate", "acercate"]):
            self.publish_head(0.5, 0.0)
            reply = "Escaneando el entorno del museo para detectar visitantes con YOLO y cámara de profundidad."
        # 10. Saludos y conversación
        elif any(w in t for w in ["hola", "buenos días", "buenas tardes", "buenas noches", "saludos"]):
            reply = "¡Hola! Mucho gusto. Soy Pepper, tu robot humanoide y guía del museo. ¿Qué recorrido o tarea deseas que hagamos?"
        elif any(w in t for w in ["quién eres", "quien eres", "cómo te llamas", "como te llamas"]):
            reply = "Soy Pepper, robot de servicio del proyecto de Robótica IA368. Estoy programado para guiar visitantes, hacer SLAM y navegar."
        elif any(w in t for w in ["gracias", "muchas gracias"]):
            reply = "¡Es un placer ayudarte! Estoy siempre a tus órdenes."
        elif "ayuda" in t or "comandos" in t:
            reply = "Puedes decirme: 'avanza', 'retrocede', 'gira izquierda/derecha', 'alto', 'mira arriba/abajo/al frente', 'dónde estás' o 'qué ves'."
        else:
            reply = f"Recibí tu indicación: '{text}'. Ejecutando y atento a tus siguientes órdenes en el museo."

        # Publicar también en /pepper/say para síntesis de voz en ROS 2
        self.publish_say(reply)
        return reply

    def _execute_timed_move(self, linear_x, angular_z, duration_sec):
        def _move_task():
            start = time.time()
            while time.time() - start < duration_sec:
                self.publish_cmd_vel(linear_x, 0.0, angular_z)
                time.sleep(0.1)
            self.publish_cmd_vel(0.0, 0.0, 0.0)
        t = threading.Thread(target=_move_task, daemon=True)
        t.start()


def run_server(port, state, bridge_node):
    BridgeHTTPHandler.state = state
    BridgeHTTPHandler.bridge_node = bridge_node
    server = ThreadingHTTPServer(('0.0.0.0', port), BridgeHTTPHandler)
    server.daemon_threads = True
    print(f"[PepperBridge] HTTP MJPEG server listening on http://0.0.0.0:{port}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


def main():
    rclpy.init()
    state = AppBridgeState()
    node = PepperBridgeNode(state)

    port = 8080
    if len(sys.argv) > 1:
        try:
            port = int(sys.argv[1])
        except ValueError:
            pass

    http_thread = threading.Thread(target=run_server, args=(port, state, node), daemon=True)
    http_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
