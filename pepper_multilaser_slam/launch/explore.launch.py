"""Exploración autónoma: Nav2 sin AMCL sobre el mapa del SLAM + frontier_explorer.

    ros2 launch pepper_multilaser_slam explore.launch.py                  # con la simulación y fusion_slam ya corriendo
    ros2 launch pepper_multilaser_slam explore.launch.py explorer:=false  # sólo Nav2 (objetivos a mano en RViz)

La localización la da slam_toolbox (TF map → odom), así que de Nav2 se usa sólo la navegación
(nav2_bringup/navigation_launch.py) con los parámetros de pepper_nav (base, no se copian). Los
costmaps y el collision monitor usan los láseres de Pepper (`scan`, por defecto
/pepper/scan_fused), no el hokuyo: la exploración no usa ningún sensor que el robot no tenga.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    scan = LaunchConfiguration("scan")
    params = RewrittenYaml(
        source_file=PathJoinSubstitution([FindPackageShare("pepper_nav"), "config", "nav2_params.yaml"]),
        param_rewrites={"topic": scan},
        convert_types=True,
    )
    nav = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"]),
        launch_arguments={"params_file": params, "use_sim_time": "true", "autostart": "true",
                          "use_composition": "False"}.items(),
    )
    explorer = Node(
        package="pepper_multilaser_slam", executable="frontier_explorer", output="screen",
        parameters=[{"use_sim_time": True, "max_time": LaunchConfiguration("max_time")}],
        condition=IfCondition(LaunchConfiguration("explorer")),
    )
    return LaunchDescription([
        DeclareLaunchArgument("scan", default_value="/pepper/scan_fused",
                              description="láser de los costmaps y del collision monitor"),
        DeclareLaunchArgument("explorer", default_value="true"),
        DeclareLaunchArgument("max_time", default_value="1200.0", description="s de exploración"),
        nav,
        explorer,
    ])
