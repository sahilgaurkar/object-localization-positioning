import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher, Ros2SupervisorLauncher
from webots_ros2_driver.utils import controller_url_prefix

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

    # Driver nodes
    # When having multiple robot it is enough to specify the `additional_env` argument.
    # The `WEBOTS_CONTROLLER_URL` has to match the robot name in the world file.
    # You can check for more information at:
    # https://cyberbotics.com/doc/guide/running-extern-robot-controllers#single-simulation-and-multiple-extern-robot-controllers
    baxter_robot_driver = Node(
        package="webots_ros2_driver",
        executable="driver",
        name = 'baxter',
        output="screen",
        additional_env={"WEBOTS_CONTROLLER_URL": controller_url_prefix() + "baxter"},
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value=WORLD_FILE,
                description="Baxter Simulation World File",
            ),
            webots,  # Robot is Spawned with world - NEED TO CHECK HOW SPAWN ROBOT SEPERATELY
            ros2_supervisor,
            # Launch the driver node
            baxter_robot_driver,
            # Kill all the nodes when the driver node is shut down (useful with other ROS 2 nodes)
            launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessExit(
                    target_action=baxter_robot_driver,
                    on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
                )
            ),
        ]
    )
