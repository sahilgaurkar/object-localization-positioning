"""Launch Webots Baxter simulation world."""

import launch
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher, Ros2SupervisorLauncher

PACKAGE_NAME = "webots_baxter"
WORLD_FILE = "baxter_sim_world.wbt"


def generate_launch_description():
    package_dir = get_package_share_directory(PACKAGE_NAME)
    world = LaunchConfiguration("world")

    # The WebotsLauncher is a Webots custom action that allows you to start a Webots simulation instance.
    # It searches for the Webots installation in the path specified by the `WEBOTS_HOME` environment variable and default installation paths.
    # The accepted arguments are:
    # - `world` (str): Path to the world to launch.
    # - `gui` (bool): Whether to display GUI or not.
    # - `mode` (str): Can be `pause`, `realtime`, or `fast`.
    # - ros2_supervisor (bool): If True, spawns the Ros2Supervisor custom node that communicates with a Supervisor robot in the simulation.
    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, "worlds", world]), ros2_supervisor=True
    )

    # The Ros2Supervisor is a Webots custom node that communicates with a Supervisor robot in the simulation.
    # The accepted arguments from launch.actions.ExecuteProcess are:
    # - `output` (string): Output configuration for process output logging. Default is 'screen'.
    # - `respawn` (bool): Relaunch the process that abnormally died.  Default is 'True'.
    ros2_supervisor = Ros2SupervisorLauncher()

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value=WORLD_FILE,
                description="Baxter Simulation World File",
            ),
            webots,
            ros2_supervisor,
            # This action will kill all nodes once the Webots simulation has exited
            launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessExit(
                    target_action=webots,
                    on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
                )
            ),
        ]
    )
