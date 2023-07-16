#!/usr/bin/env python

# Copyright 1996-2023 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch Webots Universal Robot simulation nodes."""

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
    

    spawn_URDF_baxter = URDFSpawner(
        name="Baxter",
        robot_description=baxter_xacro_description,
        translation="0 0 0.925",
        rotation="0 0 1 0",
    )

    # Driver nodes
    # When having multiple robot it is mandatory to specify the robot name.
    baxter_robot_driver = WebotsController(
        robot_name="Baxter",
        # namespace="baxter",
        parameters=[
            {"robot_description": baxter_xacro_path},
            {"use_sim_time": True},
            {"set_robot_state_publisher": True},
            ros2_control_params,
        ],
    )

    # Other ROS 2 nodes
    controller_manager_timeout = ["--controller-manager-timeout", "100"]
    controller_manager_prefix = "python.exe" if os.name == "nt" else ""
    trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        prefix=controller_manager_prefix,
        arguments=[
            "baxter_joint_trajectory_controller",
            # "-c",
            # "baxter/controller_manager",
        ]
        + controller_manager_timeout,
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        prefix=controller_manager_prefix,
        arguments=["baxter_joint_state_broadcaster", 
                #    "-c", 
                #    "baxter/controller_manager"
        ]
        + controller_manager_timeout,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{'use_sim_time': True, "robot_description": baxter_xacro_description}]
    )

    return LaunchDescription(
        [
            spawn_URDF_baxter,
            # Other ROS 2 nodes
            robot_state_publisher,
            trajectory_controller_spawner,
            joint_state_broadcaster_spawner,
            # Launch the driver node once the URDF robot is spawned
            launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessIO(
                    target_action=spawn_URDF_baxter,
                    on_stdout=lambda event: get_webots_driver_node(
                        event, baxter_robot_driver
                    ),
                )
            ),
            # Kill all the nodes when the driver node is shut down
            launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessExit(
                    target_action=baxter_robot_driver,
                    on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
                )
            ),
        ]
    )
