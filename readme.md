# Eurobot 2021 ROS workspace

## Download

Connecting to GitHub with SSH
```
git clone --recurse-submodules git@github.com:sunfu-chou/eurobot_ros_ws.git
```
HTTPS
```
git clone --recurse-submodules https://github.com/sunfu-chou/eurobot_ros_ws.git
```

## ROS dependencies 
1. navigation
2. costmap_converter
3. teb_local_planner
4. robot_localization

## Pre-prepare to build

### Install ydlidar sdk

```bash
cd src/YDLidar-SDK/build
cmake ..
make
sudo make install
```

you can discard change after build and isntall

### Install ROS package

```bash
sudo apt install ros-melodic-navigation ros-melodic-costmap-converter ros-melodic-teb-local-planner ros-melodic-robot-localization
```
