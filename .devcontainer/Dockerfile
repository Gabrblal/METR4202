FROM osrf/ros:noetic-desktop-full

RUN apt update && apt install -y \
       wget \
       git \
       python3-catkin-tools \
       ros-noetic-dynamixel-sdk \
       python3-rosdep \
       python3-rosinstall \
       python3-rosinstall-generator \
       python3-wstool \
       build-essential \
       ros-noetic-fiducials \
       ros-noetic-fiducial-msgs \
       ros-noetic-dynamixel-sdk \
       linux-headers-"$(uname -r)"
    && rosdep update

RUN pip install pigpio

# RUN wget https://www.ximea.com/downloads/recent/XIMEA_Linux_SP.tgz \
#     && tar xzf XIMEA_Linux_SP.tgz \
#     && cd package \
#     && ./install \
#     && echo 0 | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb \
#     && echo "*               -       rtprio          0 \
# @realtime       -       rtprio          81 \
# *               -       nice            0 \
# @realtime       -       nice            -16" | sudo tee -a /etc/security/limits.conf \
#     && groupadd realtime \
#     && gpasswd -a $USER realtime \
#     && cd .. \
#     && rm -rf package XIMEA_Linux_SP.tgz

RUN echo source /opt/ros/noetic/setup.bash >> ~/.bashrc

ENTRYPOINT tail -f /dev/null
