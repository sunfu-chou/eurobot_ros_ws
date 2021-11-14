# Eurobot 2021 ROS workspace

## Environment
Ubuntu 18.04

ros melodic
## Download

Connecting to GitHub with 

SSH
```
git clone --recurse-submodules git@github.com:sunfu-chou/eurobot_ros_ws.git
```
**HTTPS** (*recommended*)
```
git clone --recurse-submodules https://github.com/sunfu-chou/eurobot_ros_ws.git
```

## ROS dependencies 
1. navigation
2. costmap_converter
3. teb_local_planner
4. robot_localization

## Prepare to build

### Build and Install ydlidar sdk

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

## Connect to YDlidar 

### Create serial port alias [optional but recommended]
```
chmod 0777 src/ydlidar_ros_driver/startup/*
sudo sh src/ydlidar_ros_driver/startup/initenv.sh
```

### Test connection
```
roslaunch eurobot lidar_G6.launch
```
![](/doc/connection_to_lidar.png)

## Run localization

Don't need to launch lidar before localization
### Launch Odom Publisher

Topic `odom` is needed
1. Launch rosserial, rx_to_odom, etc. on Raspberry Pi to publish `odom`
2. Launch `fake_odom`
```
roslaunch fake_odom fake_odom.launch
```

### Launch Localization
```
roslaunch eurobot localization.launch
```
Including `lidar_G6`, `localization`, `ekf`

![](doc/tf_tree.png)
![](doc/node_graph.png)
