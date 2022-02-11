# spot_ros_autonomy
## UON - MSc COMP4097 Enhanced MSc Research Project in Computer Science (Artificial Intelligence) 2021/2022

## Supplementary Material Submission 11/02/2022
Work created/modified by the author:

Created:
- All gazebo_world_*.launch files
- All ~/world/gaze_world_*.world files
- ~/obstacle_avoidance.py

Modified
- ~/spot.urdf

If building this project please follow instructions for installation of Champ by (https://github.com/chvmp/champ)

Clone this project repo to ~/workspace/src/

Run catkin_make & source devel/setup.bash

Run roslaunch spot_ros_autonomy gazebo_world_large_1.launch (A connected joystick is required)

# Project Description
This repository contains work for simulation of the quadruped robot SPOT, (Robot by Boston Dynamics) (URDF by Clearpath Robotics).

# System Specifications
Ubuntu - 18.04.06

Kernel - Tested on 5.4.0 & 5.11.0

ROS - Melodic

Gazebo - ver. 9

# Tested on System Hardware:

## System 1

  CPU - AMD 3950x @ 4.3GHz
  
  GPU - Nvidia RTX 2080 Super
  
  RAM - 32GB
  
  Real-Time-Factor Avg: 1
  
## System 2

  CPU - Intel i7 11800H @ 4.6GHz
  
  GPU - Nvidia RTX 3050ti (Mobile)
  
  RAM - 32GB
  
  Real-Time-Factor Avg: 1

# Credits
Spot Base Mechanics: Chvmp/Champ (https://github.com/chvmp/champ) by Author: Juan Miguel Jimeno (https://github.com/grassjelly)

URDF: Clearpath Robotics (https://github.com/clearpathrobotics/spot_ros) by Clearpath Robotics (https://clearpathrobotics.com/)
