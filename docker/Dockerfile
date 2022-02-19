FROM osrf/ros:noetic-desktop-full-focal

SHELL ["/bin/bash", "-c"]

# Install dependencies
RUN apt update \
    && apt install -y \
    git \
    ros-${ROS_DISTRO}-navigation \
    ros-${ROS_DISTRO}-costmap-converter \
    ros-${ROS_DISTRO}-teb-local-planner  \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-stage-ros \
    ros-${ROS_DISTRO}-stage \
    liblapack-dev \
    libblas-dev \
    libboost-dev \
    libarmadillo-dev \
    qtbase5-dev \
    qtdeclarative5-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root

RUN git clone --recurse-submodules https://github.com/sunfu-chou/eurobot_ros_ws.git

RUN mkdir ./eurobot_ros_ws/src/YDLidar-SDK/build/ \
    && cd ./eurobot_ros_ws/src/YDLidar-SDK/build/ \
    && cmake .. \
    && make \
    && make install

ENV CATKIN_WS=/root/eurobot_ros_ws
ENV Qt5_DIR=/usr/lib/x86_64-linux-gnu/qt5

WORKDIR /root/eurobot_ros_ws

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && catkin_make 

RUN echo "PS1='\${debian_chroot:+(\$debian_chroot)}\\[\033[01;32m\\]\\u@\\H\[\\033[00m\\]:\\[\\033[01;34m\\]\\w\\[\\033[00m\\]\\$ '" >> /root/.bashrc \
    && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc \
    && echo "source ${CATKIN_WS}/devel/setup.bash" >> /root/.bashrc 

CMD ["bash"]
