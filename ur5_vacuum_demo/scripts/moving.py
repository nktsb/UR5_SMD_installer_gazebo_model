#!/usr/bin/env python3
import sys
import rospy
import rospkg
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from math import pi
from mstates import ModelPose

from gripper import Gripper

class SMD_InstallerMoveSet:

  def __init__(self):
    self.gripper = Gripper()
    self.group_arm = moveit_commander.MoveGroupCommander("arm")
    moveit_commander.roscpp_initialize(sys.argv) 

  def move_and_take(self, xyz):
    self.gripper.grasp_off()
    self.move_arm(xyz)
    self.move_arm(self.goal_down(xyz))
    self.gripper.grasp_on()
    self.move_arm(self.goal_up(xyz))

  def move_and_release(self, xyz):
    self.move_arm(xyz)
    self.move_arm(self.goal_down(xyz))
    self.gripper.grasp_off()
    self.move_arm(self.goal_up(xyz))

  def move_arm(self, xyz):
    goal = ModelPose()
    goal_pose = goal.goal_pose(xyz)
    self.group_arm.set_pose_target(goal_pose)
    plan = self.group_arm.go(wait=True)
    self.group_arm.stop()
    self.group_arm.clear_pose_targets()

  def add_walls(self):
    scene = moveit_commander.PlanningSceneInterface()
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "world"
    box_pose.pose.position.z = 0.05
    box_pose.pose.orientation.w = 1.0
    box_name = "table"
    scene.add_box(box_name, box_pose, size=(1.0, 1.0, 0.1))
    box_name = "wall"
    box_pose.pose.position.y = -0.5
    scene.add_box(box_name, box_pose, size=(1.0, 0.01, 1.0))

  def goal_up(self, xyz):
    xyz['z'] += 0.015
    return xyz

  def goal_down(self, xyz):
    xyz['z'] -= 0.015
    return xyz

  def rotate_gripper(self, component_name, goal_angle):
    state = ModelPose()

    actual_state, roll, pitch, yaw = state.get_angle(component_name)
    yaw += (goal_angle - yaw)*0.3
    state.set_angle(component_name, actual_state, roll, pitch, yaw) #rotate component

    print(component_name, goal_angle, round(yaw, 3), end = '\r')