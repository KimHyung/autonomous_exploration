# Autonomous Exploration
---
##  Install
1. ROS-melodic
```	
    sudo apt update
    sudo apt upgrdae
    wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_melodic.sh
    chmod +x ./install_ros_melodic.sh
    ./install_ros_melodic.sh
    sudo apt-get install ros-melodic-joy ros-melodic-teleop-twist-joy \
    ros-melodic-teleop-twist-keyboard ros-melodic-laser-proc \
    ros-melodic-rgbd-launch ros-melodic-depthimage-to-laserscan \
    ros-melodic-rosserial-arduino ros-melodic-rosserial-python \
    ros-melodic-rosserial-server ros-melodic-rosserial-client \
    ros-melodic-rosserial-msgs ros-melodic-amcl ros-melodic-map-server \
    ros-melodic-move-base ros-melodic-urdf ros-melodic-xacro \
    ros-melodic-compressed-image-transport ros-melodic-rqt* \
    ros-melodic-gmapping ros-melodic-navigation ros-melodic-interactive-markers
```
2. Tutrtlebot3 simulation pkg [Turtlbot3 E-Manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/#overview)
```	
    sudo apt-get install ros-melodic-dynamixel-sdk
    sudo apt-get install ros-melodic-turtlebot3-msgs
    sudo apt-get install ros-melodic-turtlebot3
    cd ~/catkin_ws/src/
    git clone -b melodic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
    cd ~/catkin_ws && catkin_make
```

3. 2d map segmentation pkg [incremental_DuDe_ROS](https://github.com/lfermin77/Incremental_DuDe_ROS)
```	
    sudo apt-get install libcgal-dev
    sudo apt-get install libmpfr-dev
    cd ~/catkin_ws/src/
    git clone https://github.com/lfermin77/Incremental_DuDe_ROS.git
    cd ~/catkin_ws && catkin_make
```

4. autonomous_exploration
```	
    cd ~/catkin_ws/src/
    git clone https://github.com/KimHyung/autonomous_exploration.git
    cd ~/catkin_ws && catkin_make
```
---
## Launch
```	
    //Launch Gazebo Simulation
    roslaunch autonomous_exploration maze.launch
    //Launch SLAM & Rviz
    roslaunch autonomous_exploration gmapping.launch
    //Luanch autonomus exploration
    roslaunch autonomous_exploration autonomous_exploration.launch
```
---
## Description
