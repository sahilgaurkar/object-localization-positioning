import os
import pathlib
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.utils import controller_url_prefix

def generate_launch_description():
    package_dir = get_package_share_directory('webots_baxter')


    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'baxter_sim_world.wbt')
    )

    return LaunchDescription([
        webots
    ])