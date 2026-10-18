#!/usr/bin/env bash
# Corre un experimento de SLAM completo en la oficina y guarda trayectoria + mapa.
#
#   run_experiment.sh <método> <carpeta_salida>
#
# Métodos:
#   v8           /pepper/laser_2 (fusión heredada del V8)          -> línea base
#   hokuyo       /pepper/hokuyo_scan (sensor "falso", 720 rayos)    -> cota superior
#   free         scan_fused, sólo espacio libre (accumulate=1)
#   acc<N>       scan_fused con N scans acumulados Y espacio libre (p. ej. acc4)
#   acc<N>nofree scan_fused con N scans acumulados, sin espacio libre (lo recomendado: acc4nofree)
#
# Variables:
#   TUNED=1        slam_toolbox con config/slam_sparse_lasers.yaml (por defecto, la config del V9)
#   GUI=1          ver Gazebo (por defecto sin interfaz)
#   ODOM_NOISE     medio | alto: el SLAM usa la odometría degradada de odom_noise.py
#                  (/pepper/odom_noisy, TF odom_noisy -> base_noisy) en vez de /pepper/odom.
#                  No vale para hokuyo (su scan va en el frame del sensor).
#   ROUTE_TIMEOUT  segundos (por defecto 600)
set -u
METHOD=$1
OUT=$(realpath -m "$2")
mkdir -p "$OUT"
LOG="$OUT/logs"; mkdir -p "$LOG"

set +u
source /opt/ros/jazzy/setup.bash
source "$HOME/pepper_ws/install/setup.bash"
set -u

WORLD=$(ros2 pkg prefix pepper_gazebo_plugin)/share/pepper_gazebo_plugin/worlds/simple_office_with_people.world
if [ "${GUI:-0}" = "1" ]; then WORLD_ARG="$WORLD"; else WORLD_ARG="-s $WORLD"; fi

PIDS=()
cleanup() {
  for p in "${PIDS[@]}"; do kill -INT -- "-$p" 2>/dev/null; done
  sleep 4
  for p in "${PIDS[@]}"; do kill -KILL -- "-$p" 2>/dev/null; done
  pkill -KILL -f "gz sim" 2>/dev/null
}
trap cleanup EXIT

echo "[$METHOD] simulación"
# El launch de la oficina del paquete base concatena "-r " + world en gz_args: con "-s <world>"
# Gazebo corre sin interfaz gráfica (los sensores se siguen renderizando en el servidor).
setsid ros2 launch pepper_gazebo_plugin pepper_gazebo_plugin_empty.launch.py \
  world:="$WORLD_ARG" x:=-0.5 y:=1.0 > "$LOG/sim.log" 2>&1 &
PIDS+=($!)

# Esperar a que lleguen los láseres y el ground truth
for i in $(seq 60); do
  if timeout 3 ros2 topic echo --once /pepper/odom_groundtruth > /dev/null 2>&1 && \
     timeout 3 ros2 topic echo --once /pepper/laser_2 > /dev/null 2>&1; then break; fi
  sleep 2
done
sleep 8   # controladores cargados y brazos abajo

echo "[$METHOD] SLAM"
if [ "${TUNED:-0}" = "1" ]; then
  OVR=$(ros2 pkg prefix pepper_multilaser_slam)/share/pepper_multilaser_slam/config/slam_sparse_lasers.yaml
else
  OVR=none
fi
SCAN=/pepper/scan_fused; RANGE=7.0; ACC=""; FREE=true
ODOM_FRAME=odom; BASE_FRAME=base_footprint; V8_SCAN=/pepper/laser_2
if [ -n "${ODOM_NOISE:-}" ]; then
  [ "$METHOD" = hokuyo ] && { echo "ODOM_NOISE no admite hokuyo"; exit 1; }
  ODOM_FRAME=odom_noisy; BASE_FRAME=base_noisy; V8_SCAN=/pepper/laser_2_noisy
  setsid ros2 run pepper_multilaser_slam odom_noise --ros-args -p preset:=$ODOM_NOISE \
    -p "relabel_scans:=['/pepper/laser_2:/pepper/laser_2_noisy']" > "$LOG/odom_noise.log" 2>&1 &
  PIDS+=($!)
fi
case "$METHOD" in
  v8)         SCAN=$V8_SCAN ;;
  hokuyo)     SCAN=/pepper/hokuyo_scan; RANGE=20.0 ;;
  free)       ACC=1 ;;
  acc*nofree) ACC=${METHOD#acc}; ACC=${ACC%nofree}; FREE=false ;;
  acc*)       ACC=${METHOD#acc} ;;
  *) echo "método desconocido: $METHOD"; exit 1 ;;
esac
if [ -n "$ACC" ]; then
  setsid ros2 run pepper_multilaser_slam scan_fusion --ros-args -p use_sim_time:=true \
    -p accumulate:=$ACC -p free_space:=$FREE -p odom_frame:=$ODOM_FRAME -p base_frame:=$BASE_FRAME \
    -p sensor_frame:=base_footprint > "$LOG/fusion.log" 2>&1 &
  PIDS+=($!)
fi
setsid ros2 launch pepper_multilaser_slam slam.launch.py scan:=$SCAN max_laser_range:=$RANGE \
  overrides:=$OVR odom_frame:=$ODOM_FRAME base_frame:=$BASE_FRAME > "$LOG/slam.log" 2>&1 &
PIDS+=($!)
sleep 6

setsid ros2 run pepper_multilaser_slam trajectory_recorder --ros-args -p use_sim_time:=true \
  -p output:="$OUT/trajectory.csv" -p odom_frame:=$ODOM_FRAME -p base_frame:=$BASE_FRAME \
  > "$LOG/recorder.log" 2>&1 &
PIDS+=($!)
sleep 2

echo "[$METHOD] recorrido"
START=$(date +%s)
timeout "${ROUTE_TIMEOUT:-600}" ros2 run pepper_multilaser_slam route_driver \
  --ros-args -p use_sim_time:=true > "$LOG/route.log" 2>&1
echo "[$METHOD] recorrido: código $?, $(( $(date +%s) - START )) s"
sleep 5   # último update del mapa (map_update_interval = 2 s)

for i in 1 2 3; do
  ros2 run nav2_map_server map_saver_cli -f "$OUT/map" --ros-args -p use_sim_time:=true \
    > "$LOG/map_saver.log" 2>&1 && break
done
[ -f "$OUT/map.yaml" ] && echo "[$METHOD] mapa guardado" || echo "[$METHOD] NO se pudo guardar el mapa"
