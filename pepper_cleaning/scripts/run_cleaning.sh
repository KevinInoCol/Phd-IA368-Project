#!/bin/bash
# Corrida completa de limpieza sin GUI: simulación + limpieza de todo el museo + grabación.
#
#   pepper_cleaning/scripts/run_cleaning.sh <dir_salida> [mundo] [batería_inicial] [orden]
#   pepper_cleaning/scripts/run_cleaning.sh results/limpieza/test_r2 museum_test_people 45
#   pepper_cleaning/scripts/run_cleaning.sh results/limpieza/moving_r1 museum_with_people_moving 45
#
# orden: "start" (todo el museo, por defecto) o "start <zona>". Deja en <dir>: run.jsonl,
# summary.json, cleaning.png (tools/record_cleaning.py) y logs/sim.log.
#
# Variables opcionales:
#   GUI=true          con Gazebo y RViz en pantalla (por defecto sin GUI)
#   CALLS=chat,silent visitantes que llaman a Pepper mientras limpia (tools/visitor_calls.py):
#                     deja además calls.jsonl y calls_summary.json
#   CALL_ZONES=sala_oeste,entrada   sólo llamar mientras limpia esas zonas (con gente quieta cerca)
#   CALL_GAP=90       s simulados entre retomar la limpieza y la siguiente llamada
OUT=$1
WORLD=${2:-museum_test_people}
BATTERY=${3:-45.0}
ORDER=${4:-start}
HERE=$(cd "$(dirname "$0")/.." && pwd)
source /opt/ros/jazzy/setup.bash
source ~/pepper_ws/install/setup.bash
set -u   # después de los setup.bash de ROS (no toleran variables sin definir)
mkdir -p "$OUT/logs"
OUT=$(cd "$OUT" && pwd)

# Un Gazebo vivo de otra corrida haría que esta se conectara a él (Pepper ya no empieza en el inicio)
if pgrep -f "gz sim -r" > /dev/null; then
    echo "ya hay un Gazebo corriendo (pgrep -af 'gz sim'): páralo antes"; exit 1
fi

cleanup() {
    [ -n "${REC:-}" ] && kill -INT "$REC" 2>/dev/null
    [ -n "${CALLER:-}" ] && kill -INT "$CALLER" 2>/dev/null
    if [ -n "${SIM:-}" ]; then
        # todo el grupo de procesos del launch: matar sólo ros2 launch dejaba gz sim huérfano
        kill -INT -- -"$SIM" 2>/dev/null
        for _ in $(seq 30); do pgrep -g "$SIM" > /dev/null || break; sleep 1; done
        kill -9 -- -"$SIM" 2>/dev/null
    fi
    fastdds shm clean >/dev/null 2>&1
}
trap cleanup EXIT

set -m   # control de trabajos: el launch va en su propio grupo de procesos, con PGID = $!
GUI=${GUI:-false}
CALLS=${CALLS:-}
ros2 launch pepper_cleaning cleaning_demo.launch.py gui:="$GUI" rviz:="$GUI" world:="$WORLD" \
    battery:="$BATTERY" > "$OUT/logs/sim.log" 2>&1 &
SIM=$!
set +m

# Nav2 activo y nodos de limpieza arriba
timeout 240 bash -c "until grep -q 'cleaner: zonas' '$OUT/logs/sim.log'; do sleep 2; done" || { echo "cleaner no arrancó"; exit 1; }
timeout 120 bash -c "until ros2 lifecycle get /bt_navigator 2>/dev/null | grep -q active; do sleep 2; done" || { echo "Nav2 no activo"; exit 1; }
sleep 12   # AMCL con la pose inicial

python3 "$HERE/tools/record_cleaning.py" "$OUT" --timeout 3000 > "$OUT/logs/record.log" 2>&1 &
REC=$!
if [ -n "$CALLS" ]; then
    python3 "$HERE/tools/visitor_calls.py" "$OUT" --plan "$CALLS" \
        --zones "${CALL_ZONES:-}" --gap "${CALL_GAP:-90}" > "$OUT/logs/calls.log" 2>&1 &
    CALLER=$!
fi
sleep 4
ros2 topic pub -w 1 --once /pepper/clean_cmd std_msgs/msg/String "data: $ORDER" > /dev/null
wait "$REC"
REC=
if [ -n "${CALLER:-}" ]; then
    kill -INT "$CALLER" 2>/dev/null; wait "$CALLER"; CALLER=
    cat "$OUT/calls_summary.json"
fi
cat "$OUT/summary.json"
