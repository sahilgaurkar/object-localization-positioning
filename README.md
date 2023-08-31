# ROS 2 Object Localization and Positioning


Dependencies:
1. ROS2 (humble) binary install : [Website](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html#ubuntu-debian)
2. Install and enable CycloneDDS : [Website](https://docs.ros.org/en/humble/Installation/DDS-Implementations/Working-with-Eclipse-CycloneDDS.html#eclipse-cyclone-dds)
3. Install Webots_ROS2 (R2023a) : [Website](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Installation-Ubuntu.html#installation-ubuntu)
4. Install Moveit2 Binary (humble) : [Website](https://moveit.ros.org/install-moveit2/binary/)



Steps to use the Software Stack:

Source the setup.bash for all terminals

1. Terminal 1: `ros2 launch webots_baxter robot_moveit_nodes_launch.py`

2. Move the Object present on user table within the cameras FOV.

3. Terminal 2: `ros2 launch webots_baxter start_launch.py`


