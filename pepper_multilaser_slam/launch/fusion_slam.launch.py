"""SLAM con la fusión nueva de los 3 láseres (Fase 1 del proyecto IA368).

    ros2 launch pepper_multilaser_slam fusion_slam.launch.py                  # acumula 4 scans + slam ajustado (lo recomendado)
    ros2 launch pepper_multilaser_slam fusion_slam.launch.py accumulate:=1 free_space:=true
    ros2 launch pepper_multilaser_slam fusion_slam.launch.py overrides:=none   # slam_toolbox del V9 tal cual

Arranca scan_fusion (/pepper/scan_fused) y slam_toolbox con la configuración de la Fase 6 del
Tutorial V9 más los ajustes de config/slam_sparse_lasers.yaml (ver slam.launch.py).
La simulación se lanza aparte, como siempre:

    ros2 launch pepper_gazebo_plugin pepper_gazebo_plugin_in_office_CPU.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    fusion = Node(
        package="pepper_multilaser_slam", executable="scan_fusion", output="screen",
        parameters=[{
            "use_sim_time": True,
            "accumulate": ParameterValue(LaunchConfiguration("accumulate"), value_type=int),
            "free_space": ParameterValue(LaunchConfiguration("free_space"), value_type=bool),
            "num_rays": ParameterValue(LaunchConfiguration("num_rays"), value_type=int),
        }],
    )
    slam = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare("pepper_multilaser_slam"), "launch", "slam.launch.py"]),
        launch_arguments={"scan": "/pepper/scan_fused", "max_laser_range": "7.0",
                          "overrides": LaunchConfiguration("overrides")}.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument("accumulate", default_value="4",
                              description="nº de scans acumulados con la odometría (1 = sin acumular)"),
        DeclareLaunchArgument("free_space", default_value="false",
                              description="rayos sin eco como espacio libre (empeora el matching de Karto, ver scan_fusion.py)"),
        DeclareLaunchArgument("num_rays", default_value="360"),
        DeclareLaunchArgument("overrides", default_value=PathJoinSubstitution(
            [FindPackageShare("pepper_multilaser_slam"), "config", "slam_sparse_lasers.yaml"]),
            description='ajustes de slam_toolbox, o "none"'),
        fusion,
        slam,
    ])
