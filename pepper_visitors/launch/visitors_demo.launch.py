"""Fase 3: Pepper en el museo atiende las llamadas de los visitantes.

    ros2 launch pepper_visitors visitors_demo.launch.py                          # museo de prueba (4 personas)
    ros2 launch pepper_visitors visitors_demo.launch.py x:=-2.0 y:=-1.5          # otra pose de arranque
    ros2 launch pepper_visitors visitors_demo.launch.py world:=museum_with_people_moving
    ros2 launch pepper_visitors visitors_demo.launch.py voice:=true              # llamada por micrófono

y la llamada (sin micrófono):

    ros2 topic pub --once /pepper/visitor_call std_msgs/String "data: hola pepper"

Simulación (museum.launch.py de pepper_museum) + Nav2 con AMCL (amcl.launch.py de pepper_nav)
en el mapa del museo vacío (pepper_museum/maps/museum_hokuyo.yaml, frame map = mundo + (1.5, −1))
+ visitor_approach (+ voice_listener si voice:=true). AMCL arranca en (0, 0) del mapa; si Pepper
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
        launch_arguments={"map": PathJoinSubstitution([FindPackageShare("pepper_museum"), "maps",
                                                       "museum_hokuyo.yaml"]),
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
    approach = Node(
        package="pepper_visitors", executable="visitor_approach", output="screen",
        parameters=[{"use_sim_time": True, "log_file": LaunchConfiguration("log_file")}],
    )
    voice = Node(
        package="pepper_visitors", executable="voice_listener", output="screen",
        parameters=[{"device": LaunchConfiguration("device")}],
        condition=IfCondition(LaunchConfiguration("voice")),
    )
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="museum_test_people"),
        DeclareLaunchArgument("gui", default_value="true", description="interfaz de Gazebo"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("x", default_value="-1.5", description="pose de arranque (mundo)"),
        DeclareLaunchArgument("y", default_value="1.0"),
        DeclareLaunchArgument("scan", default_value="/pepper/hokuyo_scan",
                              description="láser de AMCL y costmaps: /pepper/hokuyo_scan o /pepper/laser_2"),
        DeclareLaunchArgument("voice", default_value="false"),
        DeclareLaunchArgument("device", default_value="", description="micrófono (node.name de PipeWire)"),
        DeclareLaunchArgument("log_file", default_value="", description="JSON por llamada"),
        sim,
        # Nav2 y la percepción cuando la simulación ya publica reloj, TF y sensores
        TimerAction(period=25.0, actions=[nav]),
        TimerAction(period=40.0, actions=[initial_pose]),
        TimerAction(period=30.0, actions=[approach, voice]),
    ])
