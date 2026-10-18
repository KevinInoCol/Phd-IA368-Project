"""Pepper en el museo ISCAS (Fase 2 del proyecto IA368; mundos de la tesis portados a Harmonic).

    ros2 launch pepper_museum museum.launch.py                                    # personas en movimiento
    ros2 launch pepper_museum museum.launch.py world:=museum_with_people          # personas estáticas
    ros2 launch pepper_museum museum.launch.py world:=museum                      # museo vacío
    ros2 launch pepper_museum museum.launch.py gui:=false                         # Gazebo sin interfaz

Reutiliza pepper_gazebo_plugin_empty.launch.py de la base (modelo, controladores, puente,
laser_publisher...) con el mundo del museo. Pepper aparece en (-1.5, 1), como en la tesis.
"""

from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, TimerAction)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare

ACTIVATE_JSB = """
for i in $(seq 30); do
  s=$(ros2 control list_controllers -c /pepper/controller_manager 2>/dev/null | grep joint_state_controller)
  case "$s" in
    *" active"*) exit 0 ;;
    *inactive*) ros2 control switch_controllers -c /pepper/controller_manager \\
                  --activate joint_state_controller >/dev/null 2>&1 \\
                && echo "[museum] joint_state_controller activado" && exit 0 ;;
  esac
  sleep 5
done
echo "[museum] joint_state_controller no se pudo activar"
"""


def generate_launch_description():
    pkg = FindPackageShare("pepper_museum")
    world = PathJoinSubstitution([pkg, "worlds", [LaunchConfiguration("world"), ".sdf"]])
    # El launch base concatena "-r " + world en gz_args: "-s <world>" = Gazebo sin interfaz.
    server_only = PythonExpression(["'' if '", LaunchConfiguration("gui"), "' == 'true' else '-s '"])
    sim = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare("pepper_gazebo_plugin"), "launch",
                              "pepper_gazebo_plugin_empty.launch.py"]),
        launch_arguments={"world": [server_only, world],
                          "x": LaunchConfiguration("x"), "y": LaunchConfiguration("y")}.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="museum_with_people_moving",
                              description="museum | museum_with_people | museum_with_people_moving"),
        DeclareLaunchArgument("gui", default_value="true", description="interfaz de Gazebo"),
        DeclareLaunchArgument("x", default_value="-1.5"),
        DeclareLaunchArgument("y", default_value="1.0"),
        # Modelos del museo (iscas_museum, actor_walk); la base añade los suyos (personas)
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", PathJoinSubstitution([pkg, "models"])),
        sim,
        # El museo (malla de 8 MB) y los actores hacen que el arranque vaya muy lento (RTF ~0.02)
        # y el spawner de la base agota sus 5 s con joint_state_controller, que queda inactivo
        # (sin /pepper/joint_states ni TF de brazos y cabeza). Se reintenta activarlo.
        TimerAction(period=20.0, actions=[ExecuteProcess(cmd=["bash", "-c", ACTIVATE_JSB],
                                                         output="screen")]),
    ])
