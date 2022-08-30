FROM osrf/ros:noetic-desktop-full

RUN apt update && apt install -y git python3-catkin-tools

RUN echo source /opt/ros/noetic/setup.bash >> ~/.bashrc

ENTRYPOINT tail -f /dev/null
