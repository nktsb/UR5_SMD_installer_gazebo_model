# UR5_SMD_installer
This repository contains UR5 model for gazebo, which has vacuum gripper and OpenCV camera script for component's angle correction.
ROS distro - noetic.

Because of vacuum gripper plugin specifics, components can't be rotated by UR5 wrist, so rotation emulated by changing component's model yaw angle by /gazebo/set_model_state service.

To start demo, use demo.launch in pkg ur5_vacuum_demo.
To start components installing process, start algorithm.py in ur5_vacuum_demo/scripts/

https://github.com/nktsb/UR5_SMD_installer/assets/55137551/305098ff-c8ae-4c86-b82e-88d791a83583

