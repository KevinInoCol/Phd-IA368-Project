"""Genera docs/nodos-topicos.html: grafos de nodos/tópicos (Graphviz → SVG) + tabla + árbol de TF.

Vista A = lo que corre en la demo del museo con la app (medido en vivo con `ros2 node info`).
Vista B = SLAM y exploración (Fases 1–2), sacado del código y de los launch del proyecto.
usage: gen_graph.py <template.html> <out.html>
"""
import json, re, subprocess, sys, html

# ---------------------------------------------------------------- catálogo
# grupo: proj = este proyecto · base = port de Pepper (V9) · std = paquetes estándar (Nav2, slam_toolbox, yolo_ros)
NODES = {
    "ros_gz_bridge": ("base", "puente Gazebo → ROS 2: láseres, cámaras, sonares y reloj de la simulación"),
    "gazebo_ros_p3d": ("base", "pose real de Pepper en el mundo (ground truth), sólo para evaluar y para la app"),
    "gazebo_ros_model_velocity": ("base", "mueve la base omnidireccional con /pepper/cmd_vel; publica la odometría y la TF odom → base_footprint"),
    "joint_state_controller": ("base", "ros2_control: estado de las articulaciones"),
    "Head_controller": ("base", "ros2_control: trayectorias de HeadYaw / HeadPitch"),
    "robot_state_publisher": ("base", "TF de todo el cuerpo a partir del URDF y de /pepper/joint_states"),
    "laser_publisher": ("base", "fusión del V8: une los 3 láseres en /pepper/laser_2 (línea base de la Fase 1)"),
    "map_server": ("std", "Nav2: sirve el mapa guardado del museo (museum_hokuyo.yaml)"),
    "amcl": ("std", "Nav2: localiza a Pepper en el mapa con el láser; publica la TF map → odom"),
    "yolo_node": ("std", "yolo_ros: YOLO11m sobre la cámara frontal"),
    "debug_node": ("std", "yolo_ros: dibuja las cajas sobre la imagen (vista YOLO de la app)"),
    "global_costmap": ("std", "Nav2: costmap global (mapa + láser), lo usa el planner"),
    "local_costmap": ("std", "Nav2: costmap local (láser), lo usa el controller"),
    "bt_navigator": ("std", "Nav2: árbol de comportamiento de navigate_to_pose"),
    "planner_server": ("std", "Nav2: calcula la ruta sobre el costmap global"),
    "controller_server": ("std", "Nav2: sigue la ruta y genera velocidades"),
    "behavior_server": ("std", "Nav2: comportamientos de recuperación (spin, backup, drive_on_heading)"),
    "velocity_smoother": ("std", "Nav2: suaviza las velocidades"),
    "collision_monitor": ("std", "Nav2: frena si el láser ve un obstáculo dentro del polígono de seguridad; publica la velocidad final"),
    "slam_toolbox": ("std", "SLAM (Karto) con la configuración de la base + ajustes para sensores dispersos; publica /map y la TF map → odom"),
    "voice_listener": ("proj", "Whisper large-v3-turbo en la GPU: transcribe el micrófono del PC o el audio de la app; detecta palabras de llamada"),
    "visitor_approach": ("proj", "Fase 3: busca con la cabeza, YOLO11m + clasificador + profundidad, va con Nav2 a ~1 m y mira a la persona"),
    "place_navigator": ("proj", "lugares con nombre (museum_places.yaml): «vamos a la mesa» → alias → pose → Nav2; caminando o corriendo"),
    "pepper_mobile_bridge": ("proj", "pepper_bridge.py: HTTP :8080 para la app; MJPEG, chat con Gemini, voz, modo visitante, lugares, mando y cabeza"),
    "scan_fusion": ("proj", "Fase 1: une los 3 láseres en un barrido de 360°, acumula 4 barridos y (opcional) filtra lo que se mueve"),
    "frontier_explorer": ("proj", "exploración autónoma: elige fronteras del /map y manda objetivos a Nav2"),
    "route_driver": ("proj", "ruta guiada y repetible para los experimentos (alternativa a la exploración)"),
    "trajectory_recorder": ("proj", "graba pose real, pose del SLAM y odometría en trajectory.csv"),
    "odom_noise": ("proj", "odometría degradada (/pepper/odom_noisy) para probar el SLAM con errores sistemáticos"),
}
# tópico: (tipo, Hz medido o texto, descripción)
TOPICS = {
    "/pepper/scan_left": ("sensor_msgs/LaserScan", "4.8", "láser izquierdo de Pepper"),
    "/pepper/scan_front": ("sensor_msgs/LaserScan", "4.4", "láser frontal de Pepper"),
    "/pepper/scan_right": ("sensor_msgs/LaserScan", "4.4", "láser derecho de Pepper"),
    "/pepper/hokuyo_scan": ("sensor_msgs/LaserScan", "4.3", "láser hokuyo de 720 rayos: referencia; en la demo del museo lo usan AMCL, los costmaps y collision_monitor"),
    "/pepper/laser_2": ("sensor_msgs/LaserScan", "4.1", "fusión del V8 (línea base)"),
    "/pepper/scan_fused": ("sensor_msgs/LaserScan", "—", "barrido de 360° de scan_fusion"),
    "/pepper/camera/front/image_raw": ("sensor_msgs/Image", "2.1", "cámara frontal 640×480"),
    "/pepper/camera/depth/image_raw": ("sensor_msgs/Image", "20", "cámara de profundidad 320×240"),
    "/pepper/camera/bottom/image_raw": ("sensor_msgs/Image", "2.5", "cámara inferior 640×480"),
    "/pepper/odom": ("nav_msgs/Odometry", "15", "odometría de la base"),
    "/pepper/odom_noisy": ("nav_msgs/Odometry", "—", "odometría con errores (experimentos)"),
    "/pepper/odom_groundtruth": ("nav_msgs/Odometry", "14", "pose real en el mundo"),
    "/pepper/joint_states": ("sensor_msgs/JointState", "70", "estado de las articulaciones"),
    "/tf": ("tf2_msgs/TFMessage", "31", "transformaciones dinámicas (map → odom → base_footprint → cuerpo)"),
    "/map": ("nav_msgs/OccupancyGrid", "latched", "mapa de ocupación (transient local: se recibe al suscribirse)"),
    "/yolo/detections": ("yolo_msgs/DetectionArray", "3.7", "cajas y clases de YOLO"),
    "/yolo/dbg_image": ("sensor_msgs/Image", "2.8", "imagen con las cajas dibujadas"),
    "/pepper/voice/audio": ("std_msgs/Int16MultiArray", "evento", "frase grabada en la app, 16 kHz mono"),
    "/pepper/voice/transcript": ("std_msgs/String", "evento", "texto de cada frase (vacío si no se entendió)"),
    "/pepper/visitor_call": ("std_msgs/String", "evento", "llamada de un visitante: frase con palabra de llamada (micrófono del PC) o cualquier frase del modo visitante de la app"),
    "/pepper/say": ("std_msgs/String", "evento", "lo que dice Pepper"),
    "/pepper/visitor_target": ("geometry_msgs/PointStamped", "evento", "posición de la persona elegida (para RViz)"),
    "/pepper/visitor_status": ("std_msgs/String", "evento", "fase de la llamada en JSON (buscando, acercándose, de frente…) para la app"),
    "/pepper/go_to": ("std_msgs/String", "evento", "orden de ir a un lugar: nombre o alias, o JSON {place, run} / {cancel}"),
    "/pepper/place_status": ("std_msgs/String", "evento", "JSON: going, arrived, failed, canceled o unknown; el bridge avisa en el chat"),
    "/pepper/places": ("std_msgs/String", "latched", "lista de lugares en JSON (transient local); el bridge se la da a Gemini"),
    "/pepper/places/markers": ("visualization_msgs/MarkerArray", "latched", "flecha y nombre de cada lugar, para RViz"),
    "/speed_limit": ("nav2_msgs/SpeedLimit", "evento", "% de la velocidad máxima: 40 caminando, 100 corriendo"),
    "/cmd_vel_nav": ("geometry_msgs/Twist", "—", "velocidad del controller o de un comportamiento"),
    "/cmd_vel_smoothed": ("geometry_msgs/Twist", "—", "velocidad suavizada"),
    "/pepper/cmd_vel": ("geometry_msgs/Twist", "—", "velocidad final que mueve la base"),
    "/pepper/Head_controller/command": ("trajectory_msgs/JointTrajectory", "evento", "objetivo de HeadYaw / HeadPitch"),
}
ACTIONS = {
    "navigate_to_pose": ("nav2_msgs/action/NavigateToPose", "ir a una pose del mapa"),
    "spin": ("nav2_msgs/action/Spin", "girar en el sitio"),
    "drive_on_heading": ("nav2_msgs/action/DriveOnHeading", "avanzar recto"),
    "backup": ("nav2_msgs/action/BackUp", "retroceder"),
    "compute_path_to_pose": ("nav2_msgs/action/ComputePathToPose", "pedir una ruta al planner"),
    "follow_path": ("nav2_msgs/action/FollowPath", "seguir la ruta"),
}

NAV_CHAIN = [
    ("bt_navigator", "compute_path_to_pose"), ("compute_path_to_pose", "planner_server"),
    ("bt_navigator", "follow_path"), ("follow_path", "controller_server"),
    ("controller_server", "/cmd_vel_nav"), ("behavior_server", "/cmd_vel_nav"),
    ("/cmd_vel_nav", "velocity_smoother"), ("velocity_smoother", "/cmd_vel_smoothed"),
    ("/cmd_vel_smoothed", "collision_monitor"), ("collision_monitor", "/pepper/cmd_vel"),
    ("/pepper/cmd_vel", "gazebo_ros_model_velocity"),
    ("gazebo_ros_model_velocity", "/pepper/odom"), ("/pepper/odom", "controller_server"), ("/pepper/odom", "bt_navigator"),
]

# Vista A: demo del museo + app (ros2 node info, 2026-10-27)
VIEW_A = NAV_CHAIN + [
    ("ros_gz_bridge", "/pepper/hokuyo_scan"),
    ("/pepper/hokuyo_scan", "amcl"), ("/pepper/hokuyo_scan", "collision_monitor"),
    ("/pepper/hokuyo_scan", "global_costmap"), ("/pepper/hokuyo_scan", "local_costmap"),
    ("ros_gz_bridge", "/pepper/camera/front/image_raw"),
    ("/pepper/camera/front/image_raw", "yolo_node"), ("/pepper/camera/front/image_raw", "debug_node"),
    ("/pepper/camera/front/image_raw", "visitor_approach"), ("/pepper/camera/front/image_raw", "pepper_mobile_bridge"),
    ("ros_gz_bridge", "/pepper/camera/depth/image_raw"), ("/pepper/camera/depth/image_raw", "visitor_approach"),
    ("ros_gz_bridge", "/pepper/camera/bottom/image_raw"), ("/pepper/camera/bottom/image_raw", "pepper_mobile_bridge"),
    ("gazebo_ros_p3d", "/pepper/odom_groundtruth"), ("/pepper/odom_groundtruth", "pepper_mobile_bridge"),
    ("joint_state_controller", "/pepper/joint_states"), ("/pepper/joint_states", "robot_state_publisher"),
    ("map_server", "/map"), ("/map", "amcl"), ("/map", "visitor_approach"), ("/map", "global_costmap"),
    ("amcl", "/tf"), ("robot_state_publisher", "/tf"), ("gazebo_ros_model_velocity", "/tf"),
    ("/tf", "visitor_approach"), ("/tf", "bt_navigator"), ("/tf", "collision_monitor"), ("/tf", "amcl"),
    ("yolo_node", "/yolo/detections"), ("/yolo/detections", "debug_node"),
    ("debug_node", "/yolo/dbg_image"), ("/yolo/dbg_image", "pepper_mobile_bridge"),
    ("pepper_mobile_bridge", "/pepper/voice/audio"), ("/pepper/voice/audio", "voice_listener"),
    ("voice_listener", "/pepper/voice/transcript"), ("/pepper/voice/transcript", "pepper_mobile_bridge"),
    ("voice_listener", "/pepper/visitor_call"), ("/pepper/visitor_call", "visitor_approach"),
    ("pepper_mobile_bridge", "/pepper/say"), ("visitor_approach", "/pepper/say"), ("/pepper/say", "pepper_mobile_bridge"),
    ("visitor_approach", "/pepper/visitor_target"),
    ("pepper_mobile_bridge", "/pepper/visitor_call"),
    ("visitor_approach", "/pepper/visitor_status"), ("/pepper/visitor_status", "pepper_mobile_bridge"),
    ("visitor_approach", "/pepper/cmd_vel"),
    ("visitor_approach", "navigate_to_pose"), ("navigate_to_pose", "bt_navigator"),
    ("visitor_approach", "spin"), ("spin", "behavior_server"),
    ("pepper_mobile_bridge", "/pepper/cmd_vel"),
    ("pepper_mobile_bridge", "/pepper/go_to"), ("/pepper/go_to", "place_navigator"), ("/pepper/go_to", "visitor_approach"),
    ("place_navigator", "navigate_to_pose"), ("place_navigator", "/speed_limit"), ("/speed_limit", "controller_server"),
    ("place_navigator", "/pepper/places"), ("/pepper/places", "pepper_mobile_bridge"),
    ("place_navigator", "/pepper/place_status"), ("/pepper/place_status", "pepper_mobile_bridge"),
    ("place_navigator", "/pepper/places/markers"), ("place_navigator", "/pepper/say"),
    ("pepper_mobile_bridge", "/pepper/Head_controller/command"), ("visitor_approach", "/pepper/Head_controller/command"),
    ("/pepper/Head_controller/command", "Head_controller"),
]

# Vista B: SLAM + exploración (código y launch de pepper_multilaser_slam; explore.launch.py = Nav2 sin AMCL)
ALT = {("/pepper/laser_2", "slam_toolbox"), ("laser_publisher", "/pepper/laser_2"),
       ("/pepper/odom", "odom_noise"), ("odom_noise", "/pepper/odom_noisy"),
       ("route_driver", "/pepper/cmd_vel"), ("/pepper/odom_groundtruth", "route_driver")}
VIEW_B = NAV_CHAIN + [
    ("ros_gz_bridge", "/pepper/scan_left"), ("ros_gz_bridge", "/pepper/scan_front"), ("ros_gz_bridge", "/pepper/scan_right"),
    ("/pepper/scan_left", "scan_fusion"), ("/pepper/scan_front", "scan_fusion"), ("/pepper/scan_right", "scan_fusion"),
    ("/pepper/scan_left", "laser_publisher"), ("/pepper/scan_front", "laser_publisher"), ("/pepper/scan_right", "laser_publisher"),
    ("scan_fusion", "/pepper/scan_fused"), ("laser_publisher", "/pepper/laser_2"),
    ("/pepper/scan_fused", "slam_toolbox"), ("/pepper/laser_2", "slam_toolbox"),
    ("/pepper/scan_fused", "global_costmap"), ("/pepper/scan_fused", "local_costmap"), ("/pepper/scan_fused", "collision_monitor"),
    ("slam_toolbox", "/map"), ("slam_toolbox", "/tf"), ("gazebo_ros_model_velocity", "/tf"),
    ("joint_state_controller", "/pepper/joint_states"), ("/pepper/joint_states", "robot_state_publisher"), ("robot_state_publisher", "/tf"),
    ("/tf", "scan_fusion"), ("/tf", "slam_toolbox"), ("/tf", "frontier_explorer"), ("/tf", "trajectory_recorder"), ("/tf", "bt_navigator"),
    ("/map", "frontier_explorer"), ("/map", "global_costmap"),
    ("/pepper/odom", "odom_noise"), ("odom_noise", "/pepper/odom_noisy"),
    ("gazebo_ros_p3d", "/pepper/odom_groundtruth"), ("/pepper/odom_groundtruth", "trajectory_recorder"),
    ("/pepper/odom_groundtruth", "route_driver"), ("route_driver", "/pepper/cmd_vel"),
    ("frontier_explorer", "navigate_to_pose"), ("navigate_to_pose", "bt_navigator"),
    ("frontier_explorer", "spin"), ("frontier_explorer", "drive_on_heading"), ("frontier_explorer", "backup"),
    ("spin", "behavior_server"), ("drive_on_heading", "behavior_server"), ("backup", "behavior_server"),
]

TF_EDGES = [  # (padre, hijo, quién la publica)
    ("map", "odom", "AMCL · slam_toolbox"), ("odom", "base_footprint", "gazebo_ros_model_velocity"),
    ("base_footprint", "Tibia", "robot_state_publisher"),
    ("Tibia", "SurroundingLeftLaser_frame", ""), ("Tibia", "SurroundingFrontLaser_frame", ""),
    ("Tibia", "SurroundingRightLaser_frame", ""), ("Tibia", "SurroundingFrontLaser_fake_hokuyo_frame", ""),
    ("Tibia", "SonarFront_frame", ""), ("Tibia", "SonarBack_frame", ""),
    ("Tibia", "Pelvis", ""), ("Pelvis", "Hip", ""), ("Hip", "torso", ""), ("torso", "Neck", ""), ("Neck", "Head", "HeadYaw · HeadPitch"),
    ("Head", "CameraTop_optical_frame", ""), ("Head", "CameraDepth_optical_frame", ""), ("Head", "CameraBottom_optical_frame", ""),
    ("odom_noisy", "base_noisy", "odom_noise (experimentos)"),
]


NAV2 = {"amcl", "map_server", "global_costmap", "local_costmap", "bt_navigator", "planner_server", "controller_server",
        "behavior_server", "velocity_smoother", "collision_monitor", "compute_path_to_pose", "follow_path",
        "/cmd_vel_nav", "/cmd_vel_smoothed"}
SOURCES = {"ros_gz_bridge", "gazebo_ros_p3d", "joint_state_controller"}
SINKS = {"gazebo_ros_model_velocity", "Head_controller"}
# Lazos de realimentación: no cuentan para ordenar de izquierda a derecha
FEEDBACK = {("gazebo_ros_model_velocity", "/pepper/odom"), ("gazebo_ros_model_velocity", "/tf"),
            ("/pepper/odom", "controller_server"), ("/pepper/odom", "bt_navigator"), ("/pepper/odom", "odom_noise"),
            ("/pepper/say", "pepper_mobile_bridge"), ("/pepper/voice/transcript", "pepper_mobile_bridge"),
            ("/yolo/dbg_image", "pepper_mobile_bridge"), ("/pepper/visitor_status", "pepper_mobile_bridge"),
            ("/pepper/places", "pepper_mobile_bridge"), ("/pepper/place_status", "pepper_mobile_bridge"), ("amcl", "/tf"), ("slam_toolbox", "/tf")}


def kind(i):
    return "topic" if i.startswith("/") else ("action" if i in ACTIONS else "node")


def gid(i):
    return "g_" + re.sub(r"[^A-Za-z0-9]", "_", i)


def dot_for(edges, alt=frozenset()):
    edges = [e for e in edges if "/tf" not in e]  # /tf va en su propia sección (como "Hide tf" en rqt_graph)
    ids = []
    for a, b in edges:
        for x in (a, b):
            if x not in ids:
                ids.append(x)
    out = ['digraph G {', 'rankdir=LR; bgcolor="transparent"; nodesep=0.18; ranksep=0.55; splines=true; pad=0.2;',
           'node [fontname="DejaVu Sans Mono", fontsize=11, height=0.32, margin="0.12,0.05"];',
           'edge [arrowsize=0.6, penwidth=1.1];']
    for i in ids:
        k = kind(i)
        if k == "node":
            grp = NODES[i][0]
            out.append(f'"{i}" [id="{gid(i)}", class="gn {k} {grp}", shape=box, style="rounded,filled", fillcolor="#ffffff", fontsize=12];')
        elif k == "topic":
            out.append(f'"{i}" [id="{gid(i)}", class="gn {k}", shape=box, style="filled", fillcolor="#ffffff", fontsize=10.5];')
        else:
            out.append(f'"{i}" [id="{gid(i)}", class="gn {k}", shape=cds, style="filled", fillcolor="#ffffff", fontsize=10.5, label="⚙ {i}"];')
    # Recuadro de Nav2 y extremos fijos: la simulación a la izquierda, los actuadores a la derecha
    nav = [i for i in ids if i in NAV2]
    if nav:
        out.append('subgraph cluster_nav2 { label="Nav2"; class="cl"; fontname="DejaVu Sans Mono"; fontsize=11; style="rounded,dashed"; margin=10;')
        out += [f'"{i}";' for i in nav]
        out.append("}")
    src = [i for i in ids if i in SOURCES]
    out.append("{rank=source; " + " ".join(f'"{i}";' for i in src) + "}")
    sink = [i for i in ids if i in SINKS]
    out.append("{rank=sink; " + " ".join(f'"{i}";' for i in sink) + "}")
    for a, b in edges:
        cls = "ge" + (" alt" if (a, b) in alt else "") + (" tf" if "/tf" in (a, b) else "")
        style = ', style="dashed"' if (a, b) in alt else ""
        if (a, b) in FEEDBACK or (a == "/tf"):
            style += ", constraint=false"
        out.append(f'"{a}" -> "{b}" [class="{cls}", id="e_{gid(a)}--{gid(b)}"{style}];')
    out.append("}")
    return "\n".join(out), ids


def render(dot_src):
    svg = subprocess.run(["dot", "-Tsvg"], input=dot_src.encode(), capture_output=True, check=True).stdout.decode()
    svg = svg[svg.index("<svg"):]
    svg = re.sub(r'<svg width="[^"]*" height="[^"]*"', '<svg class="graph"', svg, count=1)
    svg = re.sub(r"<!--.*?-->", "", svg, flags=re.S)
    svg = re.sub(r'<polygon fill="white" stroke="transparent"[^>]*/>', "", svg)  # fondo de Graphviz
    return svg


def tf_svg():
    out = ['digraph T {', 'rankdir=TB; bgcolor="transparent"; nodesep=0.15; ranksep=0.32; pad=0.2;',
           'node [fontname="DejaVu Sans Mono", fontsize=10.5, shape=box, style="rounded,filled", fillcolor="#ffffff", height=0.3];',
           'edge [arrowsize=0.55, fontname="DejaVu Sans Mono", fontsize=9];']
    key = {"map", "odom", "base_footprint", "odom_noisy", "base_noisy"}
    seen = set()
    for p, c, by in TF_EDGES:
        for x in (p, c):
            if x not in seen:
                seen.add(x)
                cl = "tfk" if x in key else ("tfs" if "Laser" in x or "Sonar" in x or "Camera" in x else "tfb")
                out.append(f'"{x}" [class="{cl}"];')
        lab = f', label="{by}"' if by else ""
        out.append(f'"{p}" -> "{c}" [class="tfe"{lab}];')
    out.append("}")
    return render("\n".join(out))


def data_json(edges_by_view):
    items = {}
    for v, edges in edges_by_view.items():
        for a, b in edges:
            for x in (a, b):
                k = kind(x)
                it = items.setdefault(x, {"id": x, "gid": gid(x), "kind": k, "views": [], "pub": [], "sub": []})
                if v not in it["views"]:
                    it["views"].append(v)
            # pub = quién escribe en el tópico/acción; sub = quién lee
            if kind(a) == "node":
                items[b]["pub"].append(a) if a not in items[b]["pub"] else None
            if kind(b) == "node":
                items[a]["sub"].append(b) if b not in items[a]["sub"] else None
    for x, it in items.items():
        if it["kind"] == "node":
            it["group"], it["desc"] = NODES[x]
        elif it["kind"] == "topic":
            it["type"], it["hz"], it["desc"] = TOPICS[x]
        else:
            it["type"], it["desc"] = ACTIONS[x]
    return items


def main():
    tpl, out = sys.argv[1], sys.argv[2]
    dot_a, _ = dot_for(VIEW_A)
    dot_b, _ = dot_for(VIEW_B, ALT)
    data = data_json({"A": VIEW_A, "B": VIEW_B})
    page = open(tpl, encoding="utf-8").read()
    n_nodes = {v: len({x for e in E for x in e if kind(x) == "node"}) for v, E in (("A", VIEW_A), ("B", VIEW_B))}
    n_topics = {v: len({x for e in E for x in e if kind(x) == "topic"}) for v, E in (("A", VIEW_A), ("B", VIEW_B))}
    page = (page.replace("{{SVG_A}}", render(dot_a)).replace("{{SVG_B}}", render(dot_b)).replace("{{SVG_TF}}", tf_svg())
            .replace("{{DATA}}", json.dumps(data, ensure_ascii=False))
            .replace("{{NA}}", str(n_nodes["A"])).replace("{{TA}}", str(n_topics["A"]))
            .replace("{{NB}}", str(n_nodes["B"])).replace("{{TB}}", str(n_topics["B"]))
            .replace("{{NTOT}}", str(sum(1 for i in data.values() if i["kind"] == "node")))
            .replace("{{TTOT}}", str(sum(1 for i in data.values() if i["kind"] == "topic")))
            .replace("{{ATOT}}", str(sum(1 for i in data.values() if i["kind"] == "action"))))
    open(out, "w", encoding="utf-8").write(page)
    print("ok", n_nodes, n_topics, len(page) // 1024, "KB")


if __name__ == "__main__":
    main()
