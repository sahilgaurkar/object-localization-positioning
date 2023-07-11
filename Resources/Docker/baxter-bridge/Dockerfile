#  Command to build it
# docker built -t <image name > .
FROM osrf/ros:noetic-desktop-full

RUN apt-get -y update
RUN apt-get install -y git && apt-get install -y python3-pip
RUN apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

## Install Baxter Simulation Dependencies
RUN apt install -y ros-noetic-moveit ros-noetic-effort-controllers software-properties-common
RUN add-apt-repository ppa:rock-core/qt4
RUN apt update
RUN apt install -y qt4-dev-tools libqt4-dev libqtcore4 libqtgui4 nano curl


### Install Galactic from Binaries
RUN locale  # check for UTF-8
RUN apt update && sudo apt install locales 
RUN locale-gen en_US en_US.UTF-8 
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 
RUN LANG=en_US.UTF-8 
RUN locale  # verify settings

RUN sudo apt update && sudo apt install curl
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt -y update
RUN apt -y upgrade

RUN apt-get install -y ros-galactic-desktop
RUN apt-get install -y ros-dev-tools
RUN rosdep update

###

# Clone ros1_baxter Simulation and build
RUN mkdir -p /r1_ws_baxter
WORKDIR /r1_ws_baxter
RUN git clone https://github.com/sahilgaurkar/baxter-simulation.git . -b main
RUN /bin/bash -c ". /opt/ros/noetic/setup.bash && catkin_make"
WORKDIR /

# Clone ros2_baxter Messages and build
RUN mkdir -p /r2_ws_baxter/src
WORKDIR /r2_ws_baxter/src
RUN git clone https://github.com/sahilgaurkar/baxter-messages-ros2.git . -b master
WORKDIR /r2_ws_baxter
RUN /bin/bash -c ". /opt/ros/galactic/setup.bash && colcon build"
WORKDIR /


# Clone ros1_bridge galatic branch
RUN mkdir -p /ros1_bridge/src
WORKDIR /ros1_bridge/src
RUN git clone https://github.com/ros2/ros1_bridge -b galactic
WORKDIR /ros1_bridge
# Sourcing workspaces
#RUN /bin/bash -c "source /opt/ros/noetic/setup.bash &&\
#                source /opt/ros/galactic/setup.bash &&\
#                source /r1_ws_baxter/devel/setup.bash &&\
#                source /r2_ws_baxter/install/local_setup.bash &&\
#                colcon build --cmake-force-configure"

WORKDIR /

#Setting python2.7 as python
RUN curl https://bootstrap.pypa.io/pip/2.7/get-pip.py --output get-pip.py
RUN python2 get-pip.py
RUN pip2 install pyyaml rospkg numpy
RUN update-alternatives --install /usr/bin/python python /usr/bin/python2 1



RUN echo "ALL Done"




