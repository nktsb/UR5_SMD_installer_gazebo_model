#!/usr/bin/env python3
import sys
import rospy
import rospkg
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import csv

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import GetModelState

def set_state(model_name, state, angle):
  goal_state = ModelState()
  goal_state.model_name = model_name
  goal_state.pose.position = state.pose.position
  goal_state.pose.orientation = state.pose.orientation
  goal_state.pose.orientation.z = angle
  rospy.wait_for_service('/gazebo/set_model_state')
  try:
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    resp = set_state(goal_state)
  except rospy.ServiceException:
    print("Service failed")

def get_state(model_name):
  rospy.wait_for_service('/gazebo/get_model_state')
  try:
    get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    state = get_state(model_name, 'world')
    return state
  except rospy.ServiceException:
    print("Service failed")