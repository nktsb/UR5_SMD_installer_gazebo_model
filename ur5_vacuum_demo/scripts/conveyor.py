#!/usr/bin/env python3
import os
import sys
import rospy
import rospkg
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import threading
import time
import random
from math import pi

from mstates import ModelPose

SPAWNER_PERIOD = 20
TIMER_PERIOD = 0.5

class Conveyor:
  def __init__(self):
    self.start_stop_flag = 0
    self.start_stop_flag = 0
    self.pcb_counter = 0
    self.task_counter = 0
    self.pcb_counter = 0
    self.conveyor_objects=[]

  def put_object(self, object_name):
    state = ModelPose()
    object_state = state.get_pose(object_name)
    object = {
      'name': object_name,
      'state': object_state
    }
    self.conveyor_objects.append(object)

  def remove_object(self, object):
    self.conveyor_objects.remove(object)

  def spawn_pcb(self):
    pcb_name = 'pcb_' + str(self.pcb_counter)
    self.pcb_counter += 1
    yaw = round(random.uniform(0, pi), 3)
    os.system('roslaunch ur5_vacuum_demo pcb.launch pcb_name:="' + str(pcb_name) + 
                '" yaw:="' + str(yaw) + '"')
    self.put_object(pcb_name)

  def task(self):
    while True:
      if self.start_stop_flag == 1 and self.conveyor_objects:
        for obj in self.conveyor_objects:
          obj['state'].pose.position.y -= 0.003 * len(self.conveyor_objects)

          if obj['state'].pose.position.y <= -0.75:
            obj['state'].pose.position.y = -10
            self.remove_object(obj)
         
          state = ModelPose()
          state.set_pose(obj['name'], obj['state'])
        # time.sleep(0.1/len(self.conveyor_objects))


  def start(self):
    self.start_stop_flag = 1

  def stop(self):
    self.start_stop_flag = 0

if __name__ == "__main__":
  test = Conveyor()
  test.start()
  test.task()
