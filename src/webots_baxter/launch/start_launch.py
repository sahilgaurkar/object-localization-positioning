#!/usr/bin/env python


import os
import pathlib
import yaml
import xacro
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes


PACKAGE_NAME = 'webots_baxter'


def generate_launch_description():
    launch_description_nodes = []

    launch_description_nodes.append(
            Node(
                package='pymoveit2',
                executable='motion_test.py',
            )
        )

    launch_description_nodes.append(
            Node(
                package='webots_baxter',
                executable='start.py',
            )
        )
    
    return LaunchDescription(launch_description_nodes)