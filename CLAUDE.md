# Proyecto IA368 (Robótica, Prof. Eric Rohmer) — contexto de trabajo

Proyecto de curso con **Pepper simulado** en ROS 2 Jazzy + Gazebo Harmonic (Ubuntu 24.04 nativo,
RTX 4060). Empezado el 2026-10-15. `README.md` es el documento del proyecto (explicación,
uso, resultados): **actualizarlo al cerrar cada fase o experimento**.

## Estado y próximos pasos (actualizado 2026-10-18)

Sesión 2026-10-17/18:
- Fase 1: demo visual (`demo.launch.py` + RViz con los 3 láseres en color); odometría degradada
  (`odom_noise`) y batería de 18 corridas → el SLAM baja el error de 0.78 m (odometría sola) a
  0.11 m; README actualizado; `docs/proyecto.html` (página de presentación).
- Fase 2: museo y actores de la tesis portados a Harmonic (`pepper_museum`), verificado.
- GPU: instalados los módulos NVIDIA para el kernel 7.0.0-38; verificado tras reiniciar (abajo).

Sesión 2026-10-18 (Fase 3): pasos 1–4 hechos (female_03, mundo de prueba, dataset, clasificador:
color 96.1 % vs yolo11n-cls 97.7 % en test; a > 3.5 m 92.5 vs 97.7 %) + `voice_listener` (Whisper
turbo GPU). HTML con 3 pestañas en Vercel. **Falta probar la llamada con voz real** (sólo se oyó
audio de fondo, que no disparó llamada).

Próximos pasos, en orden:
1. ~~GPU y RTF del museo~~ hecho (2026-10-18): RTF ≈ 1.0 en el museo con personas en movimiento.
   Fase 3 siguiente: buscar (barrido + descartar personas en movimiento + más cercana con
   profundidad), acercarse (Nav2: requiere mapa del museo → hacer antes la ruta/mapa de la Fase 2)
   y mirar (Head_controller).
2. Fase 2: ruta por el museo (puntos libres de muebles y de las trayectorias de los actores) y
   SLAM acc4nofreeT en museo con personas estáticas vs en movimiento (¿estelas por acumulación?).
   `run_experiment.sh` y `demo.launch.py` hoy sólo lanzan la oficina: añadirles el mundo.
3. Actualizar `docs/proyecto.html` / README con lo que salga de la Fase 2.
4. Opcional: exploración autónoma (fronteras + Nav2) en vez de la ruta fija.

Repositorio: https://github.com/KevinInoCol/Phd-IA368-Project (público, rama `main`; sólo esta
carpeta; la base está en su propio repo). Identidad git local: KevinInoCol
<kevin.inofuente.colque.27@gmail.com>. `results/**/logs/` y `__pycache__` en `.gitignore`.

Publicación: `docs/proyecto.html` está en Vercel (cuenta kevininocol, proyecto `pepper-ia368`,
**https://pepper-ia368.vercel.app**, público). Redesplegar tras cada cambio del HTML: copiarlo como
`index.html` a un directorio aparte y `vercel link --yes --project pepper-ia368 && vercel deploy --prod --yes`
(Node/Vercel CLI vía nvm en `~/.nvm`; abrir con `bash -ic` si el PATH no los ve).

Preferencias del usuario: páginas como HTML autónomo en `docs/` (no artifacts), con estilo de
presentación; responde en español.

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
| 2 | Objetos dinámicos: museo con personas en movimiento. Entorno portado de la tesis (paquete `pepper_museum`) | 🔄 entorno listo (2026-10-18); falta evaluar SLAM |
| 3 | Voz por micrófono → Pepper busca a la persona estática más cercana, se acerca y la sigue mirando hasta la siguiente pregunta | 🔄 female_03 arreglada, mundo de prueba, dataset + clasificador (97.7 %), voz (2026-10-18); falta buscar/acercarse/mirar |

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
- Pendiente: SLAM en museo estático vs dinámico (estelas por acumulación), ruta por el museo.


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

Dataset en `datasets/visitors/` (gitignored): `raw/<mundo>/{images,meta.jsonl}`, `cls/`. Venv: `~/pepper_ws/src/yolo_ros/yolo_ros/.venv/bin/python`.
`/clock` de Gazebo es best effort: suscribirse con `qos_profile_sensor_data`.
Para parar la simulación desde scripts no usar `pkill -f "ros2 launch..."` dentro de un comando que
contenga ese texto (se mata a sí mismo, exit 144).
