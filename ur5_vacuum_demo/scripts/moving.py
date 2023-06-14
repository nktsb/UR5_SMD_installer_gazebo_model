#!/usr/bin/env python3
import sys
import rospy
import rospkg
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from math import pi
import tf_conversions

def up(xyz):
  xyz[2] += 0.015
  return goal_pose_calc(xyz)

def down(xyz):
  xyz[2] -= 0.015
  return goal_pose_calc(xyz)

def goal_pose_calc(xyz = [0, 0, 0.25]):
  pose_goal = geometry_msgs.msg.Pose()
  pose_goal_quaternion = tf_conversions.transformations.quaternion_from_euler(-pi/2, 0, 0)
  pose_goal.orientation.x, pose_goal.orientation.y, pose_goal.orientation.z, pose_goal.orientation.w = pose_goal_quaternion
  pose_goal.position.x = xyz[0]
  pose_goal.position.y = xyz[1]
  pose_goal.position.z = xyz[2]
  return pose_goal

def move_arm(goal_pose):
  group_arm.set_pose_target(goal_pose)
  plan = group_arm.go(wait=True)
  group_arm.stop()
  group_arm.clear_pose_targets()

def add_walls():
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

def init():
  global group_arm
  moveit_commander.roscpp_initialize(sys.argv)
  group_arm = moveit_commander.MoveGroupCommander("arm")

