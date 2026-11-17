"""Limpieza del museo: piso sucio + limpieza por cobertura + base de carga + batería + autodocking.

    ros2 launch pepper_cleaning cleaning_demo.launch.py                       # museo de prueba (4 personas)
    ros2 launch pepper_cleaning cleaning_demo.launch.py gui:=false rviz:=false
    ros2 launch pepper_cleaning cleaning_demo.launch.py world:=museum_with_people_moving

Incluye visitors_demo.launch.py de pepper_visitors (simulación del museo, Nav2 con AMCL, lugares con
nombre, visitor_approach), con el mapa de navegación pepper_museum/maps/museum_nav.yaml (el del
hokuyo + paneles colgantes y salientes a la altura de Pepper que el láser no ve: sin ellos Pepper
chocó con un panel y quedó doblado), y añade:
  - el modelo de la base de carga (models/charging_dock) en la esquina suroeste del museo, creado en
    Gazebo con ros_gz_sim create en la pose de config/dock.yaml (mundo = map − (1.5, −1));
  - dock_station (haz IR y contactos), battery_sim (/pepper/battery) y autodock;
  - RViz con rviz/cleaning.rviz (el de pepper_nav + piso sucio, base y haz, lugares, plan);
  - dirt_floor (baldosas sucias en las zonas de config/dirt.yaml; se borran al pasar con el cepillo)
    y cleaner (cobertura en zigzag con Nav2; pausa para cargar y sigue).

Órdenes:
    ros2 topic pub -w 1 --once /pepper/clean_cmd std_msgs/String "data: start"             # todo el museo
    ros2 topic pub -w 1 --once /pepper/clean_cmd std_msgs/String "data: start sala_oeste"
    ros2 topic pub -w 1 --once /pepper/clean_cmd std_msgs/String "data: stop"
    ros2 topic pub -w 1 --once /pepper/dirt/reset std_msgs/Empty "{}"                      # ensuciar de nuevo
    ros2 topic pub -w 1 --once /pepper/dock_cmd std_msgs/String "data: dock"      # ir a cargarse
    ros2 topic pub -w 1 --once /pepper/dock_cmd std_msgs/String "data: undock"
    ros2 topic pub -w 1 --once /pepper/battery/set std_msgs/Float32 "data: 15.0"  # batería baja → docking solo
"""

import math

from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction,
                            TimerAction)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory
import os
import yaml


def spawn_dock(context):
    share = get_package_share_directory("pepper_cleaning")
    with open(os.path.join(share, "config", "dock.yaml")) as f:
        cfg = yaml.safe_load(f)
    ox, oy = cfg.get("map_offset", [1.5, -1.0])
    cx, cy = cfg["dock"]["contact"]
    yaw = math.radians(cfg["dock"].get("facing_deg", 0.0))
    return [Node(
        package="ros_gz_sim", executable="create", output="log",
        arguments=["-world", "default", "-name", "charging_dock",
                   "-file", os.path.join(share, "models", "charging_dock", "model.sdf"),
                   "-x", str(cx - ox), "-y", str(cy - oy), "-z", "0.0", "-Y", str(yaw)],
    )]


def generate_launch_description():
    demo = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare("pepper_visitors"), "launch", "visitors_demo.launch.py"]),
        launch_arguments={**{k: LaunchConfiguration(k) for k in ("world", "gui", "x", "y", "map")},
                          "rviz": "false"}.items(),   # se abre el RViz de la limpieza (abajo)
    )
    sim_time = {"use_sim_time": True}
    station = Node(package="pepper_cleaning", executable="dock_station", output="screen",
                   parameters=[sim_time])
    battery = Node(package="pepper_cleaning", executable="battery_sim", output="screen",
                   parameters=[sim_time, {"initial": ParameterValue(LaunchConfiguration("battery"), value_type=float)}])
    autodock = Node(package="pepper_cleaning", executable="autodock", output="screen",
                    parameters=[sim_time, {"low_battery": ParameterValue(LaunchConfiguration("low_battery"),
                                                                         value_type=float)}])
    dirt = Node(package="pepper_cleaning", executable="dirt_floor", output="screen",
                parameters=[sim_time, {"world": LaunchConfiguration("world"),
                                       "brush_always": LaunchConfiguration("brush_always")}],
                condition=IfCondition(LaunchConfiguration("dirt")))
    cleaner = Node(package="pepper_cleaning", executable="cleaner", output="screen",
                   parameters=[sim_time])
    rviz = Node(package="rviz2", executable="rviz2", output="log",
                arguments=["-d", PathJoinSubstitution([FindPackageShare("pepper_cleaning"), "rviz", "cleaning.rviz"])],
                parameters=[sim_time], condition=IfCondition(LaunchConfiguration("rviz")))
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="museum_test_people"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("x", default_value="-1.5"),
        DeclareLaunchArgument("y", default_value="1.0"),
        DeclareLaunchArgument("map", default_value=PathJoinSubstitution(
            [FindPackageShare("pepper_museum"), "maps", "museum_nav.yaml"]),
            description="mapa de Nav2/AMCL: museum_nav = hokuyo + obstáculos a la altura de Pepper "
                        "que el láser no ve (paneles colgantes); museum_hokuyo.yaml = sólo el láser"),
        DeclareLaunchArgument("battery", default_value="100.0", description="carga inicial [%]"),
        DeclareLaunchArgument("low_battery", default_value="20.0", description="docking automático bajo este %"),
        DeclareLaunchArgument("dirt", default_value="true", description="piso sucio (baldosas en Gazebo)"),
        DeclareLaunchArgument("brush_always", default_value="false",
                              description="limpia al pasar aunque no esté limpiando (teleoperación)"),
        # con alcance propio: museum.launch.py redefine `world` (ruta con "-s ") y se filtraría aquí
        GroupAction([demo], scoped=True),
        # la base, cuando Gazebo ya cargó el mundo
        TimerAction(period=15.0, actions=[OpaqueFunction(function=spawn_dock)]),
        TimerAction(period=20.0, actions=[dirt]),
        TimerAction(period=25.0, actions=[rviz]),
        TimerAction(period=30.0, actions=[station, battery, autodock, cleaner]),
    ])
