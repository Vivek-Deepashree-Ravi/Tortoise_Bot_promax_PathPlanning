# Tortoise_Bot_promax_PathPlanning
 Project aims to leverage the powerful combination of the Navigation Stack 2 (Nav2) and Cartographer within the Robot Operating System 2 (ROS 2) framework to enable autonomous navigation for robotic platforms. 


This workspace has been tested with ROS 2 Galactic and Foxy.This directory contains all the code used for Cartographer based SLAM Algorithms.


## Dependencies
Install the ROS Navigation Stack 

```bash
sudo apt-get install ros-noetic-navigation
sudo apt install ros-galactic-joint-state-publisher
sudo apt install ros-galactic-robot-localization
sudo apt-get install ros-galactic-teleop-twist-keyboard
sudo apt install ros-galactic-cartographer-ros
sudo apt install ros-galactic-navigation2 ros-galactic-nav2-bringup
```

Install the Gazebo dependencies

```bash
sudo apt install ros-galactic-gazebo*
sudo apt install ros-galactic-geometry2
sudo apt install ros-galactic-joint-state-publisher ros-galactic-robot-state-publisher ros-galactic-xacro ros-galactic-gazebo-plugins-*
```

## Steps

1. Having this directory as the present working directory, build the project

```bash
colcon build
```

2. Source the files

```bash
source install/setup.bash
```

# 3. To run Real Hardware Bot in remote control mode

Make sure that the differential_publisher from tortoisebotpromax_firmware is up and running(Terminal 1)
```bash
ros2 run tortoisebotpromax_firmware differential_publisher
```
Next is the Micro_ROS should be up and running inside the robot pc one can use the dockerized version as well!(Terminal 2)
```bash
sudo docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent: galactic serial --dev /dev/ttyUSB0 -v6
```
Do confirm the ports assigned to the microcontroller via the robot PC USB bus. Also, provide permission for it!

Now, need a node that can publish data on the cmd_vel topic; it can either be the ‘teleop_twist_keyboard’ or a joystick node(Terminal 3)
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
# 4. To run Real Hardware Bot in exploration mode with RViz

First run Micro_ROS should be up and running inside the robot pc one can use the dockerized version as well!
```bash
sudo docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent: galactic serial --dev /dev/ttyUSB0 -v6
```
Do confirm the ports assigned to the microcontroller via the robot PC USB bus. Also, provide permission for it!

rplidar_ros running requires the read and write permissions of the serial device. You can manually modify it with the following command:
```bash
sudo chmod 777 /dev/ttyUSB0
```
Once the robot movements is done, we can start with autobringup!
Make sure to either start the LIDAR node separately or include in the following launch file
```bash
ros2 launch tortoisebotpromax_bringup autobringup.launch.py
```
This is launch file will get your robots joint_state_publisher, robot_state_publisher, cartographer_slam and nav2 up and running

# 5. To run Real Hardware Bot in waypoint_follower mode with RViz

First run Micro_ROS should be up and running inside the robot pc one can use the dockerized version as well!
Serial micro-ROS Agent
```bash
sudo docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent: galactic serial --dev /dev/ttyUSB0 -v6
```
Do confirm the ports assigned to the microcontroller via the robot PC USB bus. Also, provide permission for it!

rplidar_ros running requires the read and write permissions of the serial device. You can manually modify it with the following command:
```bash
sudo chmod 777 /dev/ttyUSB0
```
Once the robot movements is done, we can start with autobringup where we need to load in saved map in RViz so use this command!
(for navigation with all the sensors on real robot with given map)

```bash
ros2 launch tortoisebotpromax_bringup autobringup.launch.py use_sim_time:=False exploration:=False map_file:=/home/(ur pc name)/....(directory to the map file)
```

Later map will be loaded in RViz



 
