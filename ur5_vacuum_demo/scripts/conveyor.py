#!/usr/bin/env python3
import sys
import rospy
import rospkg
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import time

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import GetModelState

start_stop_flag = 0
start_state = ModelState()

conveyor_objects=[]

def put_object_on_conveyor(object):
  global conveyor_objects
  conveyor_objects.append(object)

def set_mstate(obj_name, state):
  goal_state = ModelState()
  goal_state.model_name = obj_name
  goal_state.pose.position = state.pose.position
  goal_state.pose.orientation = state.pose.orientation

  rospy.wait_for_service('/gazebo/set_model_state')
  try:
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    resp = set_state(goal_state)
  except rospy.ServiceException:
    print("Service failed")

def get_mstate(obj_name):
  rospy.wait_for_service('/gazebo/get_model_state')
  try:
    get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    state = get_state(obj_name, 'world')
    return state
  except rospy.ServiceException:
    print("Service failed")

def conveyor_task():
  while start_stop_flag == 1:
    for obj in conveyor_objects:
      actual_state = get_mstate(obj)
      actual_state.pose.position.y -= 0.00025
      set_mstate(obj, actual_state)
      time.sleep(0.0025)

def conveyor_init():
  global start_stop_flag
  start_stop_flag = 0

def conveyor_start():
  global start_stop_flag
  start_stop_flag = 1

def conveyor_stop():
  global start_stop_flag
  start_stop_flag = 0

if __name__ == "__main__":
  conveyor_init()
  conveyor_start()
  conveyor_task()
