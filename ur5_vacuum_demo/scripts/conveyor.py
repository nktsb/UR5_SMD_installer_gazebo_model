#!/usr/bin/env python3
import os
import sys
import rospy
import rospkg
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import time
import random
from math import pi

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import GetModelState

start_stop_flag = 0
start_state = ModelState()

conveyor_objects=[]

def put_object_on_conveyor(object):
  global conveyor_objects
  conveyor_objects.append(object)

def remove_from_conveyor(object):
  global conveyor_objects
  conveyor_objects.remove(object)

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

def spawn_pcb(pcb_name):
  yaw = round(random.uniform(0, pi), 3)
  os.system('roslaunch ur5_vacuum_demo pcb.launch pcb_name:="' + str(pcb_name) + 
              '" yaw:="' + str(yaw) + '"')

task_counter = 0
pcb_counter = 0

def conveyor_task():
  global task_counter
  global pcb_counter

  while True:
    if start_stop_flag == 1:

      if task_counter == 0:
        if pcb_counter < 3:
          pcb_counter += 1
          new_pcb = 'pcb_' + str(pcb_counter)
          spawn_pcb(new_pcb)
          put_object_on_conveyor(new_pcb)

      task_counter += 1
      if task_counter == 800:
        task_counter = 0

      for obj in conveyor_objects:
        actual_state = get_mstate(obj)
        actual_state.pose.position.y -= 0.002
        if actual_state.pose.position.y > -0.95:
          set_mstate(obj, actual_state)
        else:
          remove_from_conveyor(obj)

      time.sleep(0.02/len(conveyor_objects))

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
