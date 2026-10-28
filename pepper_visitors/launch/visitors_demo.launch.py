"""Fase 3: Pepper en el museo atiende las llamadas de los visitantes.

    ros2 launch pepper_visitors visitors_demo.launch.py                          # museo de prueba (4 personas)
    ros2 launch pepper_visitors visitors_demo.launch.py x:=-2.0 y:=-1.5          # otra pose de arranque
    ros2 launch pepper_visitors visitors_demo.launch.py world:=museum_with_people_moving
    ros2 launch pepper_visitors visitors_demo.launch.py voice:=true              # llamada por micrófono
    ros2 launch pepper_visitors visitors_demo.launch.py scan_fusion:=true scan:=/pepper/scan_fused \
        map:=$(ros2 pkg prefix pepper_museum)/share/pepper_museum/maps/museum_fused.yaml   # sin el hokuyo
    ros2 launch pepper_visitors visitors_demo.launch.py voice:=true device:=pepper_test_audio capture_sink:=true
        # llamada con audio sintetizado (tools/voice_test.py crea el sumidero y reproduce las frases)

y la llamada (sin micrófono):

    ros2 topic pub --once /pepper/visitor_call std_msgs/String "data: hola pepper"
    ros2 topic pub --once /pepper/go_to std_msgs/String "data: la mesa"     # a un lugar con nombre

Simulación (museum.launch.py de pepper_museum) + Nav2 con AMCL (amcl.launch.py de pepper_nav)
en el mapa del museo vacío (pepper_museum/maps/museum_hokuyo.yaml, frame map = mundo + (1.5, −1))
+ visitor_approach + place_navigator (lugares con nombre: "vamos a la mesa") (+ voice_listener si
voice:=true). AMCL arranca en (0, 0) del mapa; si Pepper
aparece en otro sitio se le publica la pose inicial en /initialpose.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    x, y = LaunchConfiguration("x"), LaunchConfiguration("y")
    sim = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare("pepper_museum"), "launch", "museum.launch.py"]),
        launch_arguments={"world": LaunchConfiguration("world"), "gui": LaunchConfiguration("gui"),
                          "x": x, "y": y}.items(),
    )
    nav = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare("pepper_nav"), "launch", "amcl.launch.py"]),
        launch_arguments={"map": LaunchConfiguration("map"),
                          "scan": LaunchConfiguration("scan"),
                          "rviz": LaunchConfiguration("rviz")}.items(),
    )
    # Pose inicial en el frame map (mundo + (1.5, -1)), mirando a +x como al aparecer
    initial_pose = ExecuteProcess(cmd=[
        "ros2", "topic", "pub", "--once", "--qos-durability", "transient_local",
        "/initialpose", "geometry_msgs/msg/PoseWithCovarianceStamped",
        ["{header: {frame_id: map}, pose: {pose: {position: {x: ",
         PythonExpression([x, " + 1.5"]), ", y: ", PythonExpression([y, " - 1.0"]),
         "}, orientation: {w: 1.0}}, covariance: [0.05, 0, 0, 0, 0, 0, 0, 0.05, 0, 0, 0, 0, "
         "0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.02]}}"]],
        output="log")
    # Los 3 láseres de Pepper en un barrido de 360° (como en el SLAM de la Fase 1: acc4nofree)
    fusion = Node(
        package="pepper_multilaser_slam", executable="scan_fusion", output="log",
        parameters=[{"use_sim_time": True, "accumulate": LaunchConfiguration("accumulate"),
                     "free_space": False, "sensor_frame": "base_footprint"}],
        condition=IfCondition(LaunchConfiguration("scan_fusion")),
    )
    approach = Node(
        package="pepper_visitors", executable="visitor_approach", output="screen",
        parameters=[{"use_sim_time": True, "log_file": LaunchConfiguration("log_file")}],
    )
    places = Node(
        package="pepper_visitors", executable="place_navigator", output="screen",
        parameters=[{"use_sim_time": True}],
    )
    voice = Node(
        package="pepper_visitors", executable="voice_listener", output="screen",
        parameters=[{"device": LaunchConfiguration("device"),
                     "capture_sink": LaunchConfiguration("capture_sink")}],
        condition=IfCondition(LaunchConfiguration("voice")),
    )
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="museum_test_people"),
        DeclareLaunchArgument("gui", default_value="true", description="interfaz de Gazebo"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("x", default_value="-1.5", description="pose de arranque (mundo)"),
        DeclareLaunchArgument("y", default_value="1.0"),
        DeclareLaunchArgument("scan", default_value="/pepper/hokuyo_scan",
                              description="láser de AMCL y costmaps: /pepper/hokuyo_scan, /pepper/scan_fused "
                                          "(con scan_fusion:=true) o /pepper/laser_2"),
        DeclareLaunchArgument("map", default_value=PathJoinSubstitution(
            [FindPackageShare("pepper_museum"), "maps", "museum_hokuyo.yaml"]),
            description="mapa de AMCL/Nav2: museum_hokuyo.yaml o museum_fused.yaml (3 láseres de Pepper)"),
        DeclareLaunchArgument("scan_fusion", default_value="false",
                              description="lanzar scan_fusion (/pepper/scan_fused)"),
        DeclareLaunchArgument("accumulate", default_value="4", description="barridos acumulados de scan_fusion"),
        DeclareLaunchArgument("voice", default_value="false"),
        DeclareLaunchArgument("device", default_value="", description="micrófono (node.name de PipeWire)"),
        DeclareLaunchArgument("capture_sink", default_value="false",
                              description="escuchar el monitor del sumidero `device` (pruebas sin micrófono)"),
        DeclareLaunchArgument("log_file", default_value="", description="JSON por llamada"),
        sim,
        fusion,
        # Nav2 y la percepción cuando la simulación ya publica reloj, TF y sensores
        TimerAction(period=25.0, actions=[nav]),
        TimerAction(period=40.0, actions=[initial_pose]),
        TimerAction(period=30.0, actions=[approach, places, voice]),
    ])
