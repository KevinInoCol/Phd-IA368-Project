"""Pepper coge una manzana de una mesa del museo.

    ros2 launch pepper_fetch fetch_demo.launch.py                    # con Gazebo y RViz
    ros2 launch pepper_fetch fetch_demo.launch.py gui:=false rviz:=false

    ros2 topic pub --once /pepper/fetch std_msgs/String "data: manzana"
    ros2 topic pub --once /pepper/fetch std_msgs/String "data: soltar"

Incluye visitors_demo.launch.py de pepper_visitors (simulación del museo, Nav2 con AMCL, lugares con
nombre, visitor_approach) con el mapa de navegación museum_nav, y añade:
  - la mesa de las frutas (models/fruit_table) y los objetos de config/fetch.yaml (la manzana:
    models/apple, escaneo 013_apple del YCB Object Set), creados en Gazebo con ros_gz_sim create;
  - move_group de MoveIt 2 (moveit.launch.py: configuración del repo base, sin octomap);
  - fetch_object (YOLO + Nav2 + base holonómica + cinemática inversa propia + MoveIt + agarre).
"""

import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction,
                            TimerAction)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def spawn_models(context):
    share = get_package_share_directory("pepper_fetch")
    with open(os.path.join(share, "config", "fetch.yaml")) as f:
        cfg = yaml.safe_load(f)
    ox, oy = cfg.get("map_offset", [1.5, -1.0])
    t = cfg["table"]
    cx, cy = t["center"]
    yaw = math.radians(t.get("yaw_deg", 0.0))

    def create(name, model, x, y, z, yaw=0.0):
        return Node(package="ros_gz_sim", executable="create", output="log",
                    arguments=["-world", "default", "-name", name,
                               "-file", os.path.join(share, "models", model, "model.sdf"),
                               "-x", str(x - ox), "-y", str(y - oy), "-z", str(z), "-Y", str(yaw)])

    nodes = [create(t.get("model", "fruit_table"), "fruit_table", cx, cy, 0.0, yaw)]
    for o in cfg.get("objects", []):
        lx, ly = o["at"]
        x = cx + lx * math.cos(yaw) - ly * math.sin(yaw)
        y = cy + lx * math.sin(yaw) + ly * math.cos(yaw)
        nodes.append(create(o["name"], o["model"], x, y, t["height"] + 0.003))
    return nodes


def generate_launch_description():
    demo = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare("pepper_visitors"), "launch", "visitors_demo.launch.py"]),
        launch_arguments={k: LaunchConfiguration(k) for k in ("world", "gui", "x", "y", "map", "rviz")}.items(),
    )
    moveit = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare("pepper_fetch"), "launch", "moveit.launch.py"]))
    fetch = Node(package="pepper_fetch", executable="fetch_object", output="screen",
                 parameters=[{"use_sim_time": True, "log_file": LaunchConfiguration("log_file")}])
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="museum_test_people"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("x", default_value="-1.5"),
        DeclareLaunchArgument("y", default_value="1.0"),
        DeclareLaunchArgument("map", default_value=PathJoinSubstitution(
            [FindPackageShare("pepper_museum"), "maps", "museum_nav.yaml"])),
        DeclareLaunchArgument("log_file", default_value="", description="JSON por pedido"),
        # con alcance propio: museum.launch.py redefine `world` (ruta con "-s ") y se filtraría aquí
        GroupAction([demo], scoped=True),
        TimerAction(period=15.0, actions=[OpaqueFunction(function=spawn_models)]),
        TimerAction(period=30.0, actions=[moveit, fetch]),
    ])
