"""slam_toolbox con la configuración de la Fase 6 del Tutorial V9 + ajustes del proyecto.

    ros2 launch pepper_multilaser_slam slam.launch.py scan:=/pepper/scan_fused
    ros2 launch pepper_multilaser_slam slam.launch.py scan:=/pepper/laser_2 overrides:=none
    ros2 launch pepper_multilaser_slam slam.launch.py odom_frame:=odom_noisy base_frame:=base_noisy

Carga pepper_gazebo_plugin/config/pepper_slam_toolbox.yaml y, encima, el YAML de `overrides`
(por defecto config/slam_sparse_lasers.yaml de este paquete; "none" para no cambiar nada).
Igual que pepper_slam_toolbox.launch.py: slam_toolbox es lifecycle en Jazzy y hay que
configurarlo y activarlo.
"""

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction,
                            RegisterEventHandler, TimerAction)
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


ACTIVATE_SLAM = """
for i in $(seq 30); do
  case "$(ros2 lifecycle get /slam_toolbox 2>/dev/null)" in
    active*) exit 0 ;;
    inactive*) ros2 lifecycle set /slam_toolbox activate >/dev/null 2>&1 \\
                 && echo "[slam] slam_toolbox activado por el respaldo" ;;
    unconfigured*) ros2 lifecycle set /slam_toolbox configure >/dev/null 2>&1 ;;
  esac
  sleep 2
done
echo "[slam] slam_toolbox no se pudo activar"
"""


def make_slam(context):
    base = FindPackageShare("pepper_gazebo_plugin").perform(context) + "/config/pepper_slam_toolbox.yaml"
    overrides = LaunchConfiguration("overrides").perform(context)
    params = [base]
    if overrides != "none":
        params.append(overrides)
    params.append({"use_sim_time": True,
                   "use_lifecycle_manager": False,
                   "scan_topic": LaunchConfiguration("scan").perform(context),
                   "odom_frame": LaunchConfiguration("odom_frame").perform(context),
                   "base_frame": LaunchConfiguration("base_frame").perform(context),
                   "max_laser_range": float(LaunchConfiguration("max_laser_range").perform(context))})

    slam = LifecycleNode(
        package="slam_toolbox", executable="async_slam_toolbox_node", name="slam_toolbox",
        namespace="", output="screen", parameters=params)
    configure = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(slam),
        transition_id=Transition.TRANSITION_CONFIGURE))
    activate = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=slam, start_state="configuring", goal_state="inactive",
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(slam),
            transition_id=Transition.TRANSITION_ACTIVATE))]))
    # Respaldo: a veces se pierde la respuesta de change_state o el evento de transición en el
    # arranque ("failed to send response to /slam_toolbox/change_state (timeout)") y el nodo se
    # queda inactive, sin mapa ni TF map -> odom. Se comprueba el estado y se completa a mano.
    watchdog = TimerAction(period=6.0, actions=[ExecuteProcess(cmd=["bash", "-c", ACTIVATE_SLAM],
                                                               output="screen")])
    return [slam, configure, activate, watchdog]


def generate_launch_description():
    default_overrides = FindPackageShare("pepper_multilaser_slam").find(
        "pepper_multilaser_slam") + "/config/slam_sparse_lasers.yaml"
    return LaunchDescription([
        DeclareLaunchArgument("scan", default_value="/pepper/scan_fused"),
        DeclareLaunchArgument("max_laser_range", default_value="7.0"),
        DeclareLaunchArgument("odom_frame", default_value="odom",
                              description="odom_noisy con odometría degradada (odom_noise)"),
        DeclareLaunchArgument("base_frame", default_value="base_footprint",
                              description="base_noisy con odometría degradada (odom_noise)"),
        DeclareLaunchArgument("overrides", default_value=default_overrides,
                              description='YAML con los parámetros a cambiar, o "none"'),
        OpaqueFunction(function=make_slam),
    ])
