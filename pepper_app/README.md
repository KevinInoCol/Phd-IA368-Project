# pepper_app

App Flutter para controlar e interactuar con Pepper (simulado en Gazebo) desde el teléfono.
Habla con `bridge/pepper_bridge.py`, un nodo ROS 2 que sirve HTTP en el puerto 8080.

Pestañas:

- **Cámara & Control**: cámara frontal (normal, con YOLO o inferior), cruceta, cabeza, telemetría.
- **Chat Pepper**: texto o voz → Gemini 3.5 Flash (Vertex AI) con la imagen de la cámara y
  herramientas para mover la base y la cabeza.
- **Visitante**: el mismo micrófono, pero quien habla es "un visitante del museo". La frase va a
  `/pepper/visitor_call`: `visitor_approach` busca con la cámara a la persona quieta más cercana,
  se acerca a ~1 m con Nav2 y queda de frente (cuerpo y cabeza). Luego charla con él (Gemini como
  guía) hasta que el visitante agradece o se despide. Necesita `visitors_demo.launch.py`.

```bash
# PC: bridge (y voice_listener para la voz)
source /opt/ros/jazzy/setup.bash && source ~/pepper_ws/install/setup.bash
python3 pepper_app/bridge/pepper_bridge.py
ros2 run pepper_visitors voice_listener --ros-args -p source:=topic

# App (los íconos nuevos necesitan --no-tree-shake-icons)
flutter build apk --release --split-per-abi --target-platform android-arm64 --no-tree-shake-icons
flutter build linux --release --no-tree-shake-icons   # escritorio, para probar sin emulador
```

Endpoints del bridge: `GET /stream/{front,bottom,yolo}`, `/snapshot/front`, `/api/status`,
`/api/chat`, `/api/visitor`; `POST /api/cmd_vel`, `/api/head`, `/api/say`, `/api/chat`,
`/api/voice` (WAV → texto), `/api/visitor_voice` (WAV → texto → `/pepper/visitor_call`, o en la
charla → respuesta de Gemini), `/api/visitor_end` (termina la charla).

Configuración de Gemini en `.env` (raíz del proyecto, fuera de git). Detalles en el `CLAUDE.md`
del proyecto.
