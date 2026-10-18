# Proyecto IA368 (Robótica, Prof. Eric Rohmer) — contexto de trabajo

Proyecto de curso con **Pepper simulado** en ROS 2 Jazzy + Gazebo Harmonic (Ubuntu 24.04 nativo,
RTX 4060). Empezado el 2026-10-15. `README.md` es el documento del proyecto (explicación,
uso, resultados): **actualizarlo al cerrar cada fase o experimento**.

## Estado y próximos pasos (actualizado 2026-10-18)

Sesión 2026-10-17/18:
- Fase 1: demo visual (`demo.launch.py` + RViz con los 3 láseres en color); odometría degradada
  (`odom_noise`) y batería de 18 corridas → el SLAM baja el error de 0.78 m (odometría sola) a
  0.11 m; README actualizado; `docs/arquitectura_fase1.html` (página de presentación).
- Fase 2: museo y actores de la tesis portados a Harmonic (`pepper_museum`), verificado.
- GPU: instalados los módulos NVIDIA para el kernel 7.0.0-38; **falta reiniciar y verificar**.

Próximos pasos, en orden:
1. Tras reiniciar: `nvidia-smi`, `glxinfo -B` (no debe decir llvmpipe), relanzar el museo y medir
   el RTF (`gz topic -e -t /stats -n 1`).
2. Fase 2: ruta por el museo (puntos libres de muebles y de las trayectorias de los actores) y
   SLAM acc4nofreeT en museo con personas estáticas vs en movimiento (¿estelas por acumulación?).
   `run_experiment.sh` y `demo.launch.py` hoy sólo lanzan la oficina: añadirles el mundo.
3. Actualizar `docs/arquitectura_fase1.html` / README con lo que salga de la Fase 2.
4. Opcional: exploración autónoma (fronteras + Nav2) en vez de la ruta fija; `git init` del
   proyecto (hoy NO es un repositorio git) y subirlo a GitHub.

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
| 3 | Voz por micrófono → Pepper gira/busca con la cámara a la persona (estática) llamada y la sigue mirando hasta la siguiente pregunta | ⏳ |

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
| `docs/arquitectura_fase1.html` | página HTML autónoma (estilo presentación) con arquitectura, método y resultados |

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
  595.99.02). **Pendiente tras reiniciar**: verificar `nvidia-smi`, que `glxinfo -B` no diga
  llvmpipe, y medir el RTF del museo. Los resultados de la Fase 1 se obtuvieron con render por
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

