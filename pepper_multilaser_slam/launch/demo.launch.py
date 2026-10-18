"""Demo visual de la Fase 1: Pepper hace SLAM con sus 3 láseres en la oficina, con Gazebo y RViz.

    ros2 launch pepper_multilaser_slam demo.launch.py                 # recorre la ruta de 19 puntos sola
    ros2 launch pepper_multilaser_slam demo.launch.py route:=false    # moverla uno mismo (teleop en /pepper/cmd_vel)
    ros2 launch pepper_multilaser_slam demo.launch.py gui:=false      # sólo RViz (Gazebo sin interfaz, más liviano)

Simulación de la base (oficina con personas, Pepper en (-0.5, 1)) + fusion_slam.launch.py (config
recomendada: acc4, sin espacio libre, slam ajustado) + RViz con rviz/multilaser_slam.rviz.
La ruta arranca a los `route_delay` s, cuando los controladores ya cargaron.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    base = FindPackageShare("pepper_gazebo_plugin")
    pkg = FindPackageShare("pepper_multilaser_slam")
    world = PathJoinSubstitution([base, "worlds", "simple_office_with_people.world"])
    # El launch base concatena "-r " + world en gz_args: "-s <world>" = Gazebo sin interfaz.
    world_arg = PythonExpression(["'' if '", LaunchConfiguration("gui"), "' == 'true' else '-s '"])

    sim = IncludeLaunchDescription(
        PathJoinSubstitution([base, "launch", "pepper_gazebo_plugin_empty.launch.py"]),
        launch_arguments={"world": [world_arg, world], "x": "-0.5", "y": "1.0"}.items(),
    )
    slam = IncludeLaunchDescription(PathJoinSubstitution([pkg, "launch", "fusion_slam.launch.py"]))
    rviz = Node(
        package="rviz2", executable="rviz2", output="log",
        arguments=["-d", PathJoinSubstitution([pkg, "rviz", "multilaser_slam.rviz"])],
        parameters=[{"use_sim_time": True}],
    )
    route = Node(
        package="pepper_multilaser_slam", executable="route_driver", output="screen",
        parameters=[{"use_sim_time": True}], condition=IfCondition(LaunchConfiguration("route")),
    )
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true", description="interfaz de Gazebo"),
        DeclareLaunchArgument("route", default_value="true", description="recorrer la ruta automática"),
        DeclareLaunchArgument("route_delay", default_value="25.0"),
        sim,
        TimerAction(period=10.0, actions=[slam, rviz]),
        TimerAction(period=LaunchConfiguration("route_delay"), actions=[route]),
    ])
