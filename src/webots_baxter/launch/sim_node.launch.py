import os
import pathlib
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.utils import controller_url_prefix
from controller.supervisor import Supervisor
import rclpy


def generate_launch_description():
    package_dir = get_package_share_directory("webots_baxter")

    #supervisor_obj = Supervisor()
    # baxter_node = supervisor.getFromDef("BAXTER")
    # trans_field = baxter_node.getField("translation")
    # values = trans_field.getSFVec3f()
    # print(values)
    # node = rclpy.create_node('minimal_client')
    # node.get_logger().info(values)'''

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
            # Robot is Spawned with world - NEED TO CHECK HOW SPAWN ROBOT SEPERATELY
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
