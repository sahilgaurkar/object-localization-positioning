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
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.urdf_spawner import URDFSpawner, get_webots_driver_node
from webots_ros2_driver.webots_controller import WebotsController
#from webots_ros2_driver.utils import controller_url_prefix


PACKAGE_NAME = 'baxter_webots'


def generate_launch_description():
    package_dir = get_package_share_directory(PACKAGE_NAME)
    baxter_urdf_path = os.path.join(package_dir, 'resource', 'baxter.urdf')
    robot_description_path = os.path.join(package_dir, 'resource', 'baxter.urdf')
    robot_description = pathlib.Path(baxter_urdf_path).read_text()
    ros2_control_params = os.path.join(package_dir, 'resource', 'ros2_control_config.yaml')



    # Driver nodes
    # When having multiple robot it is mandatory to specify the robot name.
    universal_robot_driver = WebotsController(
        robot_name='UR5e',
        parameters=[
            {'robot_description': robot_description_path},
            {'use_sim_time': True},
            {'set_robot_state_publisher': True},
            ros2_control_params
        ]
    )

    # universal_robot_driver = Node(
    #     package='webots_ros2_driver',
    #     executable='driver',
    #     output='screen',
    #     additional_env={'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'baxter'},
    #     parameters=[
    #         {'robot_description': robot_description},
    #         {'use_sim_time': True},
    #         ros2_control_params
    #     ],
    # )

    # Other ROS 2 nodes
    controller_manager_timeout = ['--controller-manager-timeout', '100']
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''
    trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['baxter_joint_trajectory_controller'] + controller_manager_timeout,
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['baxter_joint_state_broadcaster'] + controller_manager_timeout,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description
        }],
    )

    return LaunchDescription([

        # Other ROS 2 nodes
        robot_state_publisher,
        trajectory_controller_spawner,
        joint_state_broadcaster_spawner,
        
        #universal_robot_driver,
        # Launch the driver node once the URDF robot is spawned
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessIO(
                target_action=joint_state_broadcaster_spawner,
                on_stdout=lambda event: get_webots_driver_node(event, universal_robot_driver),
            )
        ),

        # Kill all the nodes when the driver node is shut down
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=universal_robot_driver,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
    ])
