"""move_group de MoveIt 2 para coger objetos (sin RViz ni octomap).

    ros2 launch pepper_fetch moveit.launch.py

Usa la configuración de pepper_moveit_config del repo base (SRDF, KDL, límites, controladores) igual
que su moveit_planning_execution.launch.py, pero sin `sensors_3d`: el octomap de la cámara de
profundidad pondría la mesa y la manzana como obstáculos y el brazo nunca podría llegar a ella. La
mesa la añade fetch_object a la escena de planificación como una caja.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    controllers_file = os.path.join(
        get_package_share_directory("pepper_control"), "config", "pepper_trajectory_control.yaml")
    moveit_config = (
        MoveItConfigsBuilder("JulietteY20MP", package_name="pepper_moveit_config")
        .robot_description(
            file_path=os.path.join(get_package_share_directory("pepper_description"),
                                   "urdf", "pepper_robot.urdf.xacro"),
            mappings={"gazebo": "true", "controllers_file": controllers_file})
        .robot_description_semantic(file_path="config/pepper.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(publish_robot_description=True,
                                publish_robot_description_semantic=True)
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .to_moveit_configs()
    )
    # MoveItConfigsBuilder carga config/sensors_3d.yaml del paquete aunque no se pida: se vacía
    moveit_config.sensors_3d = {}
    move_group = Node(
        package="moveit_ros_move_group", executable="move_group", output="log",
        # tolerancia de 0.05 rad entre el estado actual y el inicio del plan (0.01 por defecto): con
        # la física, la muñeca sigue oscilando unas centésimas tras cada movimiento
        parameters=[moveit_config.to_dict(), {"use_sim_time": True,
                                              "trajectory_execution.allowed_start_tolerance": 0.05}],
        remappings=[("joint_states", "/pepper/joint_states")],
    )
    return LaunchDescription([move_group])
