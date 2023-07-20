
import os
import pathlib
import launch
import xacro
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.urdf_spawner import URDFSpawner, get_webots_driver_node
from webots_ros2_driver.webots_controller import WebotsController


PACKAGE_NAME = "webots_baxter"


def generate_launch_description():

    package_dir = get_package_share_directory(PACKAGE_NAME)
    ros2_control_params = os.path.join(
        package_dir, "config", "ros2_control_config.yaml"
    )

    baxter_xacro_path = os.path.join(package_dir, "resource", "urdf", "baxter_webots.xacro")
    baxter_xacro_description = xacro.process_file(baxter_xacro_path).toxml()
   
    kinect_xacro_path = os.path.join(package_dir, "resource", "urdf", "kinect.urdf.xacro")
    kinect_xacro_description = xacro.process_file(kinect_xacro_path).toxml()


    kinect_driver = WebotsController(
        robot_name="Vision",
        parameters=[
            {"robot_description": kinect_xacro_path},
            {"use_sim_time": True},
            {"set_robot_state_publisher": False},
        ],
    )


    # Other ROS 2 nodes
    controller_manager_timeout = ["--controller-manager-timeout", "100"]
    controller_manager_prefix = "python.exe" if os.name == "nt" else ""
    
    kinect_joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        prefix=controller_manager_prefix,
        arguments=["kinect_joint_state_broadcaster"]
        + controller_manager_timeout,
    )

    kinect_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": True, "robot_description": kinect_xacro_description}]
    )

    return LaunchDescription(
        [

            kinect_driver,
            #kinect_joint_state_broadcaster_spawner,
            kinect_state_publisher,

            # Kill all the nodes when the driver node is shut down
            launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessExit(
                    target_action=kinect_driver,
                    on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
                )
            ),
        ]
    )
