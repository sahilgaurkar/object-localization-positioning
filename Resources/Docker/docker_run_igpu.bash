# If not working, first do: sudo rm -rf /tmp/.docker.xauth
# It still not working, try running the script as root.
## Build the image first
### docker build -t r2_path_planning .
## then run this script


xhost local:$USER


XAUTH=/home/$USER/.Xauthority


docker run --rm -it \
    --name=webots_test \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume /home/$USER/.Xauthority:/dot.Xauthority \
    --net=host \
    --privileged \
    osrf/ros:humble-desktop-full \
    bash

echo "Done."
