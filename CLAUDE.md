# Proyecto IA368 (Robótica, Prof. Eric Rohmer) — contexto de trabajo

Proyecto de curso con **Pepper simulado** en ROS 2 Jazzy + Gazebo Harmonic (Ubuntu 24.04 nativo,
RTX 4060). Empezado el 2026-10-15. `README.md` es el documento del proyecto (explicación,
uso, resultados): **actualizarlo al cerrar cada fase o experimento**.

## Estado y próximos pasos (actualizado 2026-10-25)

**Hecho**
- Fase 1 ✅: fusión + acumulación + ajuste de Karto; batería normal y con odometría degradada.
- Fase 2 ✅ (2026-10-20): mundo como parámetro (`WORLD=` en `run_experiment.sh`, `world:=` en
  `demo.launch.py`), ruta `museum` (13 puntos, ~58 m), referencia hokuyo
  (`pepper_museum/maps/museum_hokuyo.yaml`, también mapa de Nav2), 29 corridas, estelas
  (`tools/trails.py`) y filtro `dynamic_filter` de `scan_fusion` (`DYN=1`). Resultados en
  `results/museo/summary.md`. Conclusión: en el museo el SLAM con los láseres de Pepper se
  desvía a mitad de ruta (tramos abiertos, paredes a > 7 m) y su ATE (0.3–0.5 m) es peor que la
  odometría simulada (~0.17 m), pero cierra el lazo (error final 0.09–0.15 vs 0.3–0.4 m); con
  odometría mala 1.22 → 0.18 m. La acumulación triplica las estelas (≈1 vs 0.3 m²); el filtro
  las elimina pero no mejora la localización.
- Fase 3 ✅ salvo la voz real (2026-10-20): `visitor_approach` + `visitors_demo.launch.py`;
  10/10 llamadas en el museo dinámico, 9/10 la más cercana (la otra: empate de 3 cm), 0 falsos
  estáticos, error de posición 0.16 m, 35 s por llamada. Detalle y versiones en
  `results/fase3/approach/summary.md`. Problema abierto: el clasificador toma a `female_03` por
  `male_03` en vivo (sólo afecta al saludo).
- Voz sin micrófono ✅ (2026-10-21): `voice_listener capture_sink:=true` escucha el monitor de un
  sumidero; `tools/voice_test.py` (Piper, venv `~/pepper_ws/venvs/tts`, voces en
  `~/pepper_ws/weights/piper`, sumidero `pepper_test_audio`) → 35/36 llamadas, 0/24 falsas, ~1.2 s;
  `hotwords: "Pepper"` por defecto. De punta a punta con `visitors_demo ... voice:=true
  device:=pepper_test_audio capture_sink:=true` + `pw-play`: OK (`results/fase3/voice/summary.md`).
- README, HTML (3 pestañas) y Vercel actualizados hasta la prueba de voz; commit + push.

**En curso al cerrar la sesión del 2026-10-21: exploración autónoma (fronteras + Nav2)**
- Código (commiteado, sin documentar aún en README/HTML):
  - `pepper_multilaser_slam/frontier_explorer.py`: fronteras sobre `/map`, Nav2 NavigateToPose,
    replanteo si ya no quedan celdas de frontera a < 0.6 m del objetivo, lista negra, vuelta al
    inicio.
  - `launch/explore.launch.py`: `nav2_bringup/navigation_launch.py` con los params de pepper_nav
    y `topic` → `/pepper/scan_fused`; sin AMCL, porque localiza slam_toolbox.
  - `run_experiment.sh EXPLORE=1`: Nav2 en segundo plano y explorador con timeout (900 s en la
    oficina, 1500 s en el museo).
- Problemas encontrados y corregidos:
  - **slam_toolbox no añade scans si el robot sólo gira**, y Karto necesita 2 lecturas por
    celda. Sin moverse no hay mapa ni fronteras. Arranque: Spin 360° + DriveOnHeading 0.5 m
    (o BackUp); si sigue sin fronteras, giro de 90° + 0.5 m, hasta 6 veces (este último cambio
    **sin probar todavía**).
  - Esperar a que `bt_navigator` esté active (`/bt_navigator/get_state`) antes de enviar
    objetivos.
  - Ignorar fronteras a < 0.8 m: los láseres no ven a < 0.3 m y queda un anillo desconocido.
  - Los grupos de frontera son enormes (rayos dispersos) y su representante se mueve; por eso
    "explored" se decide por celdas cercanas, no por el representante.
  - `slam.launch.py`: vigilante que configura/activa slam_toolbox si se perdió el evento de
    transición (`failed to send response to .../change_state`).
- Resultado de depuración en la oficina (tras avanzar 1 m a mano): 22 objetivos, 0 fallidos,
  terminó sola y volvió al inicio. Mapa: 137.6 m² libres (hokuyo 138), completitud 85.8 %,
  precisión 79 %.
- La primera batería automática dio 0 objetivos (el arranque de 0.5 m no bastaba) → se paró.
  Prueba automática con el arranque repetido (`results/exploracion/oficina/acc4nofreeT_r1`, no
  commiteada): **arranca (21 objetivos), pero sale mal**: 14 fallidos (11 failed, 4 timeout), ATE
  SLAM 2.3 m, yaw RMSE 102°, área libre 94 m² (de 138). La odometría sola también da 2.4 m de ATE
  (lo normal es ~0.15): **sospecha de choque o atasco físico** (¿Pepper contra una pared, un mueble
  o una persona?, ¿BackUp/DriveOnHeading repetidos?) que rompe la odometría y luego el SLAM.
  **Siguiente paso:** mirar `trajectory.csv` (dónde se separan gt y odom), `logs/route.log` y
  `logs/nav2.log` (collision_monitor, controller, recoveries) y repetir con `GUI=1` para verlo.

**Lo que falta**
1. Exploración: depurar la prueba fallida de la oficina (arriba); cuando salga bien, batería `EXPLORE=1 TUNED=1`: oficina ×3,
   museo vacío ×2, museo dinámico ×2. Comandos en el historial: bucle `run()` con reintento
   como en las baterías del museo. `evaluate` ya da "Recorrido [m]" y "Duración [s]"; comparar
   con la ruta guiada (oficina `results/acc4nofreeT_r*`, museo `results/museo/*/acc4nofreeT_r*`).
   Después README, pestaña del HTML (¿Fase 1/2?), Vercel y push.
2. **Voz real = el micrófono del teléfono** (decidido el 2026-10-27; ya no se espera un micrófono
   para el PC): la app graba → `/api/voice` → `voice_listener source:=topic` (lanzarlo aparte:
   `visitors_demo` sin `voice:=true`). Las frases de la app **no** publican `/pepper/visitor_call`
   aunque digan "Pepper": las atiende Gemini, para no competir con `visitor_approach`. Para que
   la app pida el acercamiento falta la herramienta (B) `approach_person`.
3. Opcional: clasificador de `female_03` en vivo; localizar mejor en espacios grandes.
4. **App de teléfono `pepper_app/`**: funciona en el emulador con cámara, teleoperación y chat con
   Gemini 3.5 Flash vía Vertex. Pendiente: visión + acercarse a una persona reusando
   `visitor_approach`, revisión de código y commit (ver la sección "App de teléfono" más abajo).

Repositorio: https://github.com/KevinInoCol/Phd-IA368-Project (público, rama `main`; sólo esta
carpeta; la base está en su propio repo). Identidad git local: KevinInoCol
<kevin.inofuente.colque.27@gmail.com>. `results/**/logs/` y `__pycache__` en `.gitignore`.

Publicación: `docs/proyecto.html` está en Vercel (cuenta kevininocol, proyecto `pepper-ia368`,
**https://pepper-ia368.vercel.app**, público) y `docs/arquitectura.html` en
**https://pepper-ia368.vercel.app/arquitectura** y `docs/nodos-topicos.html` en **/nodos-topicos**
(desde 2026-10-27; sus fuentes y cómo regenerarlas en `docs/src/README.md`). Redesplegar tras cada cambio de un
HTML: en un directorio aparte, `proyecto.html` como `index.html` + `arquitectura.html` + `vercel.json`
con `{"cleanUrls": true}` (+ `nodos-topicos.html`), y `vercel link --yes --project pepper-ia368 && vercel deploy --prod --yes`
(Node/Vercel CLI vía nvm en `~/.nvm`; abrir con `bash -ic` si el PATH no los ve).

Preferencias del usuario: páginas como HTML autónomo en `docs/` (no artifacts), con estilo de
presentación; responde en español.

## App de teléfono — `pepper_app/` (actualizado 2026-10-27, commiteada)

Pedida por el usuario para controlar e interactuar con Pepper desde un teléfono Android. **La
escribió el agente de Antigravity (Gemini Flash), no Claude Code**: funciona, pero no ha tenido
una revisión de código a fondo. Commiteada el 2026-10-27 (sin `.env`, `build/`, `.dart_tool/`).

**Entorno**
- Flutter 3.47.8 / Dart 3.13.7 en `~/flutter` (en el PATH vía `~/.bashrc`). Android Studio en
  `/opt/android-studio`; SDK en `~/Android/Sdk` (android-37, build-tools 36, cmdline-tools en
  `cmdline-tools/latest`, licencias aceptadas). `adb` en `~/Android/Sdk/platform-tools`, fuera
  del PATH.
- Emulador: AVD `Medium_Phone_API_37.0` (`~/Android/Sdk/emulator/emulator -avd
  Medium_Phone_API_37.0 -gpu host -no-boot-anim`); antes de `adb install`, esperar a
  `adb shell getprop sys.boot_completed` = 1 (si no: `Can't find service: package`).
- Google Cloud SDK en `~/google-cloud-sdk` (en el PATH vía `~/.bashrc`). Sólo hay login ADC
  (`gcloud auth application-default login`), **no** `gcloud auth login`: los comandos de la CLI
  como `gcloud services enable` fallan por "no active account".

**Arquitectura**: teléfono (Flutter) ⇄ HTTP :8080 ⇄ `bridge/pepper_bridge.py` (nodo ROS 2 en el
PC) ⇄ tópicos de Pepper. No hay `rosbridge_suite` ni `foxglove_bridge`. La app se conecta a
`10.0.2.2:8080` por defecto (el host visto desde el emulador); con un teléfono físico hay que poner
la IP del PC en la LAN (192.168.0.9) desde el botón de ajustes.
- `lib/main.dart` (deps: `http`, `flutter_mjpeg`), tema oscuro, tres pestañas:
  - *Cámara & Control*: MJPEG de la cámara frontal, frontal + YOLO (`/stream/yolo` ← `/yolo/dbg_image`;
    hay que lanzar `pepper_gazebo_plugin pepper_yolo.launch.py` de la base) o inferior, cruceta → `/api/cmd_vel`, parada
    de emergencia, sliders de cabeza (yaw/pitch) → `/api/head`, telemetría (x, y, yaw, última
    frase de `/pepper/say`).
  - *Chat Pepper*: burbujas, chips de acciones rápidas, campo de texto → `/api/chat`; botón de
    micrófono (2026-10-27): toca para grabar (WAV 16 kHz mono, `record` + `path_provider`, máx.
    10 s), toca otra vez para enviar → `/api/voice` → el texto se manda como mensaje del chat.
  - *Visitante* (2026-10-27): el mismo micrófono, pero quien habla es "alguien del museo".
    `/api/visitor_voice` transcribe y el bridge publica **cualquier** frase en
    `/pepper/visitor_call` (no hace falta palabra de llamada; no pasa por Gemini). Muestra la cámara
    con YOLO, cuatro pasos (te escucho → te busco → me acerco → de frente) y el registro (frase del
    visitante, lo que dice Pepper, fases), sondeando `GET /api/visitor` cada 0.8 s.
    **Charla** (2026-10-27): cuando visitor_approach publica `reached`, el bridge pasa a la fase
    `talking` y las frases siguientes de la pestaña van a `GeminiAgent.visitor_response` (prompt de
    guía frente al visitante, imagen de la cámara, **sin herramientas de movimiento**, sólo
    `end_conversation`). Gemini la llama cuando el visitante agradece sin pedir nada más, se despide
    o dice que no tiene más preguntas → fase `ended` (la siguiente frase vuelve a ser una llamada).
    Botón "Terminar" → `POST /api/visitor_end`. Sin Gemini, un parser local reconoce despedidas.
    Probado por curl con Piper: "¿de qué trata el museo?" y "gracias, ¿y qué es un sistema
    operativo?" siguen la charla; "no, eso es todo, muchas gracias" y "eso sería todo, chao" la
    terminan.
- `bridge/pepper_bridge.py` (correr con ROS sourceado: `source /opt/ros/jazzy/setup.bash && source
  ~/pepper_ws/install/setup.bash && python3 pepper_app/bridge/pepper_bridge.py`):
  - Suscribe `/pepper/camera/front/image_raw` (best effort), cámara inferior,
    `/pepper/odom_groundtruth` y `/pepper/say`; publica `/pepper/cmd_vel`,
    `/pepper/Head_controller/command` (JointTrajectory) y `/pepper/say`.
  - GET: `/stream/front`, `/stream/bottom`, `/snapshot/front`, `/api/status`, `/api/chat`
    (historial). POST: `/api/cmd_vel`, `/api/head`, `/api/say`, `/api/chat` (JSON `message`; también
    acepta `text`), `/api/voice` (cuerpo = WAV PCM 16 bits; responde `transcript`, vacío si no
    se entendió).
  - **Voz sin duplicar Whisper**: el bridge publica la frase en `/pepper/voice/audio`
    (`Int16MultiArray`, 16 kHz) y espera `/pepper/voice/transcript` de `voice_listener`, que hay
    que lanzar aparte con `ros2 run pepper_visitors voice_listener --ros-args -p source:=topic`
    (`source`: `mic` por defecto, `topic`, `both`). Una frase del chat **nunca** sale por
    `/pepper/visitor_call`; las de la pestaña Visitante sí, pero las publica el bridge.
  - Modo visitante: publica `/pepper/visitor_call` y sigue `/pepper/visitor_status` (JSON de
    `visitor_approach`: `searching | approaching | facing | reached | nobody | nav_failed | busy |
    error`). Rechaza una llamada mientras la anterior está en curso (salvo 180 s sin cambio de
    fase: visitor_approach murió) o si `visitor_approach` no corre (hace falta `visitors_demo`).
  - Chat: primero `GeminiAgent`; si falla, un parser local de palabras clave (avanza, retrocede,
    gira, ...) para que el robot siga respondiendo.
- `bridge/gemini_agent.py`: **sólo Vertex AI** (el usuario no quiere API key de AI Studio; su
  prepago está agotado, HTTP 402). Token con `gcloud auth application-default print-access-token`
  (en caché 30 min) y cabecera `x-goog-user-project`. Modelo `gemini-3.5-flash` con
  `thinkingConfig.thinkingLevel = LOW` (en Gemini 3 sustituye a `thinkingBudget`; la temperatura
  se ignora). Herramientas: `move_robot` (avance/giro temporizado a ciegas), `move_head`,
  `stop_robot`, `get_robot_pose`.
  - **Gemini 3.x en Vertex sólo responde en `locations/global`** (host
    `aiplatform.googleapis.com`); en `us-central1` da 404.
  - Proyecto GCP: `itc-kevao-inofuente` (también es el quota project del ADC). También
    responden `datapath-kevin-inofuente` y `comunidad-intelekia`; `datapath-kevin-inofuentecolque`
    no tiene facturación y `kevao-inofuente-colque` está en DELETE_REQUESTED.
- `.env` en la raíz del proyecto (en `.gitignore` junto con `*.env`, `pepper_app/build/`,
  `pepper_app/.dart_tool/`): `GCP_PROJECT_ID`, `GCP_LOCATION=global`, `GEMINI_MODEL`,
  `GEMINI_THINKING_LEVEL` y una `GEMINI_API_KEY` que ya no se usa. **No mostrar ni commitear su
  contenido**; ninguna credencial va dentro del APK.

**Conexión (2026-10-27)**: en ⚙️ la IP del PC (casa) y, opcional, la URL de ngrok + usuario/clave
(`ngrok http 8080 --basic-auth "usuario:clave"`, la URL gratis cambia en cada arranque); ajustes en
`shared_preferences`; tras 2 fallos seguidos de `/api/status` la app prueba la otra conexión.
Credenciales en la URL → Basic auth de dart:io (también en el MJPEG). Probado por USB/LAN en el
teléfono real (voz incluida); ngrok verificado con curl (401 sin clave). Instalación en el teléfono:
el APK suelto da "No se puede verificar el desarrollador" (verificación de Android) → `adb install`.
APK para el teléfono: `flutter build apk --release --split-per-abi --target-platform android-arm64`
(18 MB, firmado con la clave debug).

**Compilar con `--no-tree-shake-icons`** (2026-10-27): sin esa opción, la compilación release de
Linux salió sin los íconos nuevos de la pestaña Visitante (círculos vacíos). Para el teléfono:
`flutter build apk --release --split-per-abi --target-platform android-arm64 --no-tree-shake-icons`.
Escritorio Linux (para probar sin emulador): `flutter build linux --release --no-tree-shake-icons`
→ `build/linux/x64/release/bundle/pepper_app`.

**Compilar e instalar**: `cd pepper_app && flutter build apk --debug` →
`build/app/outputs/flutter-apk/app-debug.apk` → `~/Android/Sdk/platform-tools/adb install -r ...`.

**Probado (2026-10-25)** con `demo.launch.py route:=false` (oficina): cámara en vivo, teleoperación,
cabeza y chat ("avanza un paso" movió a Pepper de x = −0.50 a +0.18 m y habló por `/pepper/say`).
Gemini 3.5 Flash por Vertex responde y ejecuta herramientas.

**Visión y contexto (2026-10-27, probado en `visitors_demo` sin GUI + `pepper_yolo.launch.py`)**
- (A) hecho: cada mensaje lleva el JPEG de la cámara frontal. Gemini responde mirando la imagen,
  **no con las detecciones de YOLO** (YOLO sólo dibuja las cajas en la vista de la app). Mirando
  una pared dice que no ve a nadie; frente a female_03 (YOLO `person` 0.94) dice que sí.
- (C) hecho: bucle de function calling (`MAX_TOOL_ROUNDS` = 3). Tras `move_robot`/`move_head`
  espera el movimiento + 1 s y manda la imagen nueva **dentro** de `functionResponse.parts`
  (como parte suelta Gemini 3 soltaba texto basura). Las partes con `thought` no se dicen.
- `MUSEUM_DESCRIPTION` en el system prompt: Museo ISCAS de tecnología del software, temas sacados
  de las texturas del modelo; se le prohíbe decir qué tema hay cerca si no lo lee en la imagen.
- El bridge ya no manda el mensaje actual dos veces (estaba en el historial y otra vez con la imagen).
- Ojo: tras una llamada de la Fase 3, `visitor_approach` sigue mirando a la persona (cabeza a
  10 Hz), así que "gira y dime qué ves" sigue viéndola aunque la base gire.

**Modo visitante probado (2026-10-27, `visitors_demo` sin GUI)**: frase por curl a
`/api/visitor_voice`. (1) female_03 a 1.03 m a la espalda de Pepper: sin Nav2, giro −97° → de
frente (+6°). (2) Pepper a 3.6 m: Nav2 la deja a 1.06 m con −13° de error → giro final → −1°, y
HeadYaw −0.9°: cuerpo y cabeza alineados. El giro final era con el Spin de Nav2, pero en giros de
pocos grados se pasaba (−9° → +8° → −7°) y además la base seguía girando tras terminar: ahora es
control proporcional en `/pepper/cmd_vel` (`face_tolerance` 0.05 rad, 5 lecturas seguidas dentro).
El clasificador describía a female_03 como "camisa a cuadros roja" con prob. 0.40: ahora sólo
describe la ropa con prob. ≥ 0.6. "La persona que habló" = la persona quieta más cercana: no hay
localización de la voz.

**Lugares con nombre (2026-10-28)**: "vamos a la mesa" / "corre a la entrada" → Pepper va solo.
- `pepper_museum/maps/museum_places.yaml`: 12 lugares en el frame map (pose x, y, yaw en grados =
  dónde se para Pepper y hacia dónde mira; `name`, `aliases`, `about`). Objetos ubicados con la
  malla (nombres en pinyin de ISCAS_museum.dae: `jiqi034` = mesa táctil, `lt047` = escalera,
  `paimian` = cartel dorado del museo, `men050` = puertas de madera sur, `men051` = fachada de
  vidrio este) y comprobados teletransportando a Pepper y mirando su cámara. La "mesa" es la mesa
  táctil de la sala oeste (no hay otra mesa en el museo).
- `pepper_visitors/place_navigator.py` (python del sistema, en `visitors_demo.launch.py`):
  `/pepper/go_to` (nombre/alias, o JSON `{place, run}` / `{cancel: true}`) → NavigateToPose;
  publica `/pepper/places` (JSON latched), `/pepper/place_status`, `/pepper/places/markers` (RViz),
  `/pepper/say` al llegar o fallar. Velocidad con `nav2_msgs/SpeedLimit` en `/speed_limit`:
  caminar 40 %, correr 100 %; al terminar 0 (sin límite) para no frenar a visitor_approach. Con 60 %
  caminando no se notaba la diferencia (MPPI casi no llega a 0.5 m/s cerca de las paredes).
- `visitor_approach` deja de mirar al visitante al recibir `/pepper/go_to`.
- Bridge: herramienta de Gemini `go_to_place(place enum, run)` (lista y `about` en el system
  prompt, sólo si `/pepper/places` llegó); `stop_robot`/"detente" también cancela; parser local con
  alias + verbo ("vamos", "ve a", "corre", "vuelve"…); no va a un lugar durante una llamada de
  visitante. "Llegué a…" se añade al historial del chat; la app lo refresca cada 2 s.
- Probado (sim, `museum_test_people`): 12/12 lugares alcanzados, error medio 0.16 m (máx. 0.29), yaw ±13° (tolerancia de Nav2); fotos de llegada coinciden con cada lugar. "Corre" vs caminar: máx. 0.42 vs 0.26 m/s. `ros2 topic pub --once` a veces pierde el
  mensaje antes del descubrimiento: usar `-w 2`.

**Próximo paso propuesto (pendiente de aprobación del usuario)**
- Opcional: pasar las detecciones de `/yolo/detections` como texto para que "sí, la veo" dependa
  de YOLO.
- Plan restante: (B) herramienta
  `approach_person` que publique `/pepper/visitor_call` y **reuse `visitor_approach`** (YOLO +
  profundidad + Nav2), sin duplicarlo; (C) bucle de function calling que devuelva el resultado al
  modelo; (D) para eso hay que lanzar `visitors_demo.launch.py` en lugar de `demo.launch.py`.
- Otros pendientes: mapa `/map` con los lugares en la app; etiquetar un lugar nuevo por voz
  ("este lugar se llama…", guardando la pose de AMCL en el YAML).

## Regla principal: no duplicar la base

`~/Robotica/Pepper-in-ROS2-Jazzy-and-Ubuntu-24` es el port de Pepper (Tutorial V9) del autor y
está **completo**: modelo, sensores, mundos, `laser_publisher.py` (`/pepper/laser_2`),
slam_toolbox, Nav2, YOLO, MoveIt 2. Tiene su propio `CLAUDE.md`. Aquí sólo va lo nuevo;
se reutilizan sus launch/configs (include + overrides), **no se copian ni se modifican**.

Workspace `~/pepper_ws` (`colcon build --symlink-install`):
- `src/pepper` → `~/Robotica/Pepper-in-ROS2-Jazzy-and-Ubuntu-24` (re-enlazado el 2026-10-15:
  antes apuntaba a `~/Proyectos-Robotica`, que ya no existe)
- `src/proyecto` → esta carpeta
- `src/yolo_ros`

Ojo con `--symlink-install`: launch y scripts instalados son enlaces a esta carpeta. **No
editar launch/scripts mientras corre una batería de experimentos** (cambia su comportamiento).

## Fases

| Fase | Qué | Estado |
|---|---|---|
| 1 | SLAM 2D con los 3 láseres bajos de Pepper | ✅ primera versión (2026-10-15) |
| 2 | Objetos dinámicos: museo con personas en movimiento. Entorno portado de la tesis (paquete `pepper_museum`) | ✅ SLAM evaluado, estelas y filtro dinámico (2026-10-20) |
| 3 | Voz por micrófono → Pepper busca a la persona estática más cercana, se acerca y la sigue mirando hasta la siguiente pregunta | ✅ buscar/acercarse/mirar 10/10 (2026-10-20); falta la voz con micrófono real |

## Fase 1 — paquete `pepper_multilaser_slam` (ament_python)

| Archivo | Qué |
|---|---|
| `scan_fusion.py` | `/pepper/scan_{left,front,right}` → `/pepper/scan_fused` (360°, 1°/rayo, `base_footprint`). `accumulate` (def. 4): impactos de N scans en `odom` reproyectados a la pose actual. `free_space` (def. **false**) |
| `config/slam_sparse_lasers.yaml` | overrides de slam_toolbox: `distance_variance_penalty 0.02`, `angle_variance_penalty 0.1`, `minimum_distance_penalty 0.2`, `minimum_angle_penalty 0.5`, `correlation_search_space_dimension 0.3`, `scan_buffer_size 20` |
| `launch/slam.launch.py` | slam_toolbox lifecycle: yaml de la base (`pepper_gazebo_plugin/config/pepper_slam_toolbox.yaml`) + `overrides` (`none` = config del V9 tal cual) |
| `launch/fusion_slam.launch.py` | scan_fusion + slam.launch.py (por defecto: acc4, sin free, con overrides = config recomendada) |
| `route_driver.py` | ruta de 19 puntos por la oficina guiada con `/pepper/odom_groundtruth` (~220 s, vuelve al inicio) |
| `trajectory_recorder.py` | CSV: ground truth, TF map→base, TF odom→base |
| `evaluate.py` | ATE, error final, yaw; mapa vs referencia (hokuyo `office.yaml`): precisión/completitud ≤10 cm, área libre. Agrupa `<config>_r<k>` |
| `scripts/run_experiment.sh <método> <dir>` | experimento completo sin GUI. Métodos `v8`, `hokuyo`, `free`, `acc<N>`, `acc<N>nofree`; `TUNED=1` = overrides; `GUI=1`; `ODOM_NOISE=medio\|alto` |
| `odom_noise.py` | odometría degradada (escala avance/giro + deriva de yaw) → `/pepper/odom_noisy`, TF `odom_noisy → base_noisy` en árbol APARTE; reetiqueta `/pepper/laser_2` → `laser_2_noisy`. scan_fusion: `sensor_frame:=base_footprint` para la geometría de los láseres |
| `launch/demo.launch.py` + `rviz/multilaser_slam.rviz` | demo visual: sim + fusion_slam + RViz (3 láseres en color) + ruta; args `gui`, `route` |
| `docs/proyecto.html` | página HTML autónoma (estilo presentación) con **3 pestañas, una por fase** (antes `arquitectura_fase1.html`; `#fase1/#fase2/#fase3` en la URL). Fase 2: museo, port, GPU, hipótesis de estelas, estado; el plano del museo es la malla cortada a z = 0.2 m. Fase 3: escenario, capturas de la cámara (JPEG en base64), cara vs cuerpo, estado. **Actualizar la pestaña de cada fase al avanzar** |

Resultados en `results/<config>_r<k>/` y `results/summary.md`, `maps.png`, `position_error.png`.
Con odometría degradada: `results/ruido_{alto,medio}/<config>_r<k>/` y su `summary.md`.

### Resultados clave (3 repeticiones, ATE RMSE)

- hokuyo 0.075 m · **v8 0.59 ± 0.48** (inestable, saltos) · **v8T 0.10 ± 0.01** ·
  acc4nofree 0.16 · **acc4nofreeT 0.135 ± 0.035 (recomendado)** · acc4 0.31 · acc4T 0.13.
- Área libre del mapa: v8 ~116 m², acc4* ~134 m², hokuyo 138 m².
- Odometría sola ~0.15 m (el ruido simulado es pequeño).

- **Odometría degradada (2026-10-17, 3 rep.)**: alto → odometría sola ~0.78 m; acc4nofreeT
  **0.113 ± 0.052** (completitud 98.6 %), v8T 0.146, acc4nofree 0.279, v8 0.222. Medio → odom
  ~0.30; acc4nofreeT 0.144, v8T 0.145. El ajuste de Karto sigue ayudando con odometría mala.
  `v8_r3` alto sin mapa (map_saver timeout).

Conclusiones: (1) ajustar la penalización de odometría de Karto es lo que más mejora la
localización; (2) la acumulación mejora sobre todo el mapa; (3) marcar espacio libre es
contraproducente.

### Hechos verificados en el código de Karto (`/opt/ros/jazzy/include/karto_sdk/Karto.h`)

- Mapa (`OccupancyGrid::AddScan`): lectura `NaN` o `≥ range_max` → ignorada; entre
  `max_laser_range` y `range_max` → trazada como libre sin obstáculo.
- Scan matching (`GridIndexLookup::ComputeOffsets`) usa `GetPointReadings()` **sin filtrar**
  (`wantFiltered = false` por defecto) y sólo descarta `NaN`/`inf`: cualquier lectura finita
  (p. ej. marcadores de libre a 7.5 m) entra como punto falso. Por eso `free_space` empeora.
- Penalización por alejarse de la odometría: `1 − 0.2·d²/distance_variance_penalty`, con piso
  `minimum_distance_penalty` (con 0.5 un salto de 25 cm apenas penaliza).
- El aviso "clipped range threshold to be within minimum and maximum range!" es inofensivo
  (umbral por defecto 12 m recortado al fijar `range_max` antes del `max_laser_range`).

### Detalles prácticos

- Láseres de Pepper: 15 rayos, ±30°, 0.3–7 m, 6.25 Hz; sectores frontal/izq./der. con huecos.
- Mundo `simple_office_with_people.world`: sala inferior y∈[-5,5], superior y∈[5,10], puerta
  en x≈1.5; Pepper aparece en (-0.5, 1). Frame `map` = mundo + (0.5, −1).
- Sin GUI: el launch base concatena `"-r " + world` en `gz_args`, así que `world:="-s <ruta>"`
  arranca Gazebo sólo servidor (los sensores siguen renderizándose).
- Cada experimento ~4.5 min. Esperas largas: usar tareas en background, no `sleep`.

### Siguientes pasos posibles (Fase 1)

Hecho: odometría degradada (`odom_noise`). Pendiente: más repeticiones y otra ruta;
exploración autónoma (fronteras + Nav2) en vez de la ruta fija guiada por ground truth (el
profesor puede objetar que la ruta usa conocimiento del mundo); gente en movimiento (Fase 2).
- GPU (2026-10-18): el driver NVIDIA no cargaba con el kernel 7.0.0-38 (los módulos sólo existían
  para 7.0.0-34) → render por CPU (llvmpipe): RTF ~0.44 en la oficina, ~0.02 al cargar el museo.
  Arreglado instalando `linux-modules-nvidia-595-open-generic-hwe-24.04` (módulo y driver
  595.99.02). Verificado tras reiniciar: `glxinfo -B` → RTX 4060 (no llvmpipe); museo con
  personas en movimiento sin GUI: arranque en ~25 s de pared, RTF ≈ 1.0 estable, GPU ~40 %,
  `joint_state_controller` activo, láseres a ~5.8 Hz. Los resultados de la Fase 1 se obtuvieron con render por
  CPU (todo en tiempo simulado, así que siguen siendo válidos).
- sudo: usar `sudo -A` (hay askpass gráfico en ~/.local/bin/askpass).

## Fase 2 — paquete `pepper_museum` (ament_cmake)

Fuente: tesis del autor en `~/Robotica/Referencia pepper_sim_ws-20261018T020618Z-1-001/pepper_sim_ws`
(ROS 1, Gazebo Classic): `src/pepper_virtual/pepper_gazebo_plugin/worlds/museum*.world`,
`collision_plugin/actor_collisions` (Classic, no portado: innecesario con gpu_lidar).

- `tools/convert_worlds.py <dir worlds tesis>` genera `worlds/{museum,museum_with_people,museum_with_people_moving}.sdf`
  (cabecera de sistemas como los mundos de la base; actores con `model://actor_walk/meshes/walk.dae`,
  **z + 1 m** en waypoints; loop/auto_start true). No editar los .sdf a mano.
- `models/iscas_museum` (de GitHub COONEO/neor_mini; no estaba en la tesis), `models/actor_walk`
  (walk.dae de gazebo-classic).
- `launch/museum.launch.py` (`world`, `gui`, `x`=-1.5, `y`=1): incluye `pepper_gazebo_plugin_empty.launch.py`
  y añade `models/` a `GZ_SIM_RESOURCE_PATH`; reactiva `joint_state_controller` (timeout del
  spawner por arranque lento, RTF ~0.02 al cargar).
- Museo: 16 actores en bucle (trayectorias de la tesis, 10–31 s); 10 personas estáticas en el
  mundo "moving", 23 en "with_people". Actores sin colisión física; los láseres sí los ven.
- Verificado: actores caminan (cámara frontal) y cambian rangos de los láseres con Pepper quieto.
- Ruta `museum` en `route_driver.py` (verificada sin simular contra paredes y personas de los 3
  mundos). Mapa del museo: frame `map` = mundo + (1.5, −1). `run_experiment.sh` en el museo usa
  `museum.launch.py` (timeout de ruta 900 s). slam_toolbox a veces no se activa al arrancar
  (`change_state` timeout → trayectoria vacía, sin mapa): repetir la corrida.
- `trails.py`: las celdas "junto a actores" en el mundo con 23 personas quietas están
  contaminadas (esas personas están cerca de las trayectorias); la base se mide en el vacío.


## Fase 3 — evaluación inicial (2026-10-18)

Sólo personas **estáticas** (las que caminan son dificultad para SLAM/navegación). Hallazgos
(capturas con la cámara frontal teletransportando a Pepper con `/world/default/set_pose`):
- Sólo hay **3 modelos** de persona, copiados: `citizen_extras_male_03` (camisa a cuadros roja,
  jeans), `female_02` (vestido negro), `female_03` (chaqueta marrón + jeans en la textura). Las
  copias son idénticas → ni cara ni ropa distinguen dos `male_03`: la identidad es el modelo
  (o modelo + posición). Los actores que caminan son otro aspecto (camisa verde, calvo).
- `female_03` se ve mal en Harmonic: cara con textura de cuadros/azul, ropa negra, una pierna
  sin ropa (materiales mal asignados). Hay que arreglarlo antes de reconocerla.
- Cara pequeña: cabeza ~40 px a 2 m, ~25 px a 3 m (640×480, hfov 1.0); a 1.2 m la cámara corta
  la cabeza (hay que subir HeadPitch). Muchas vistas son de lado o de espalda.
- YOLO11m (`~/pepper_ws/yolo11m.pt`) detecta a las tres de frente con conf. 0.74–0.97 hasta 4.5 m.
- Propuesta: YOLO persona → clasificador de aspecto de cuerpo entero sobre el recorte
  (fine-tune `yolo11n-cls`, 3 clases + "otro"), dataset generado y etiquetado automáticamente en
  simulación. Reconocimiento facial sólo como opción de cerca.

### Escenario de la Fase 3 (acordado con el usuario, 2026-10-18)

- Hablar por el micrófono = un visitante del museo que está **cerca** de Pepper lo llama para
  preguntarle sobre el museo. Pepper está en cualquier punto del museo; debe buscar a su
  alrededor (cabeza, luego cuerpo), encontrar a la persona estática, **acercarse** (~1 m, Nav2) y
  seguir mirándola hasta la siguiente pregunta. Si no hay nadie cerca, la llamada no tiene sentido
  en la historia: Pepper debe decirlo ("no veo a nadie"), no fallar en silencio.
- Decidido: Pepper va **siempre a la persona estática más cercana** (no se nombra a nadie). El
  clasificador de aspecto sirve para saludar, no perder a la persona y medir la percepción.
- Primero un mundo de prueba aparte con 3–4 personas estáticas distintas. **No borrar ni mover**
  las personas de los mundos actuales (23 en `museum_with_people`, 10 + 16 actores en
  `museum_with_people_moving`): la meta final es el museo completo con varias personas.
- Posición de la persona: estimarla con la cámara de profundidad, no con el ground truth.
- Voz: Whisper local, sólo para detectar la llamada.

### Fase 3 — paquete `pepper_visitors` (ament_python) y cambios en `pepper_museum`

| Archivo | Qué |
|---|---|
| `pepper_museum/models/citizen_extras_female_03_fixed` | copia de female_03 con texturas/materiales de nombre único (Harmonic cachea texturas por nombre: el original comparte `Material__1noCulling.JPG` con male_03 y `_03_-_Default1noCulling.JPG` con female_02). `convert_worlds.py` lo usa en todos los mundos (`URI_MAP`). La base sigue con el bug (oficina incluida): no se toca |
| `pepper_museum/worlds/museum_test_people.sdf` | generado: museo con 4 personas de la tesis en sus posiciones (female02_Sup_Med, female03_Sup_Der, male03_Inf_Med, male03_Inf_Izq) |
| `pepper_museum/tools/museum_floorplan.py` → `maps/museum_walls.txt` | malla del museo cortada a z = 0.2 m, celdas de 10 cm (frame del mundo) |
| `pepper_visitors/museum.py` | clases por modelo, cámara (f 585.76, c (320,240), 0.048/1.153 m sobre base_footprint, sin pitch), proyección de personas, paredes |
| `capture_dataset` (nodo) | teletransporta a Pepper (`/world/default/set_pose`), guarda imagen + pose ground truth. Estáticas: por turnos entre clases, 1–6 m, ±17°; mundo con actores: apunta a sus trayectorias |
| `tools/label_dataset.py` (venv yolo_ros) | YOLO11m + caja esperada → recortes etiquetados. División **por persona**: VAL female_10, female_06, male_05, male_13; TEST female02_Inf_Der, female03_Med_Med, male_10, male_12. Actores ("otro") por imagen |
| `tools/train_classifier.py` (venv yolo_ros) | línea base color (HSV + centroide) vs fine-tune `yolo11n-cls` (`~/pepper_ws/weights/yolo11n-cls.pt`); pesos → `pepper_visitors/models/visitor_cls.pt` |
| `visitor_approach` (envoltorio `scripts/visitor_approach`, venv yolo_ros) | llamada → barrido HeadYaw {0, ±1, ±2} + Spin 180° (`full_search`) → YOLO + profundidad (nube proyectada en la imagen frontal, torso de la caja) → pistas uno a uno, estática si ≥ 70 % a < 0.12 m → descarta posiciones sobre `/map` (persona tapada) y clase "otro" (votación de 5 recortes) → la más cercana ≤ 6 m → Nav2 a 1 m → cabeza la sigue a 10 Hz. `/pepper/say`, `/pepper/visitor_target`, `log_file` JSON |
| `launch/visitors_demo.launch.py` | museum.launch.py + `pepper_nav amcl.launch.py` (map `museum_hokuyo.yaml`, scan hokuyo por defecto) + `/initialpose` = (x+1.5, y−1) + visitor_approach (+ voice_listener con `voice:=true`) |
| `tools/eval_approach.py` | llamadas del `log_file` frente al ground truth del mundo (la más cercana con línea de vista, error de posición, falsos estáticos) |

Dataset en `datasets/visitors/` (gitignored): `raw/<mundo>/{images,meta.jsonl}`, `cls/`. Venv: `~/pepper_ws/src/yolo_ros/yolo_ros/.venv/bin/python`.
`/clock` de Gazebo es best effort: suscribirse con `qos_profile_sensor_data`.
Para parar la simulación desde scripts no usar `pkill -f "ros2 launch..."` dentro de un comando que
contenga ese texto (se mata a sí mismo, exit 144). Lo mismo con `pgrep -f`: el patrón también
casa con rutas del propio comando (el `.` de la regex casa con `/`). Lo seguro: parar desde un
script en un archivo, en una llamada aparte, con un patrón tipo `"python -m pepper_visitors[.]visitor_approach"`.
- El Head_controller escucha en `/pepper/Head_controller/command` (no `joint_trajectory`).
- **Matar nodos con `kill -9` deja segmentos de Fast DDS en `/dev/shm`**; acumulados, se pierden
  respuestas de servicios: AMCL/map_server/slam_toolbox no se activan (`failed to send response to
  .../change_state (timeout)`). Remedio: `fastdds shm clean` con todo parado (ya lo hace el
  `cleanup` de `run_experiment.sh`).
- Teletransporte para pruebas: `gz service -s /world/default/set_pose` con `name: "pepper_MP"` y
  después `/initialpose` en map (mundo + (1.5, −1)) para AMCL.
