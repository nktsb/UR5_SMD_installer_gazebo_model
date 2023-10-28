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

SPAWNER_PERIOD = 5

class Conveyor:
  def __init__(self):
    self.start_stop_flag = 0
    self.start_stop_flag = 0
    self.pcb_counter = 0
    self.task_counter = 0
    self.pcb_counter = 0
    self.conveyor_objects=[]
    self.obj_states = ModelPose()
    self.spawn_timer = threading.Timer(SPAWNER_PERIOD, self.cycle_spawn_pcb)

  def cycle_spawn_pcb(self):
    if self.start_stop_flag == 1:
      self.pcb_counter += 1
      new_pcb = 'pcb_' + str(self.pcb_counter)
      self.spawn_pcb(new_pcb)
      self.put_object(new_pcb)

    self.spawn_timer = threading.Timer(SPAWNER_PERIOD, self.cycle_spawn_pcb)
    self.spawn_timer.start()

  def put_object(self, object):
    self.conveyor_objects.append(object)

  def remove_object(self, object):
    self.conveyor_objects.remove(object)

  def spawn_pcb(self, pcb_name):
    yaw = round(random.uniform(0, pi), 3)
    os.system('roslaunch ur5_vacuum_demo pcb.launch pcb_name:="' + str(pcb_name) + 
                '" yaw:="' + str(yaw) + '"')

  def task(self):
    while True:
      if self.start_stop_flag == 1 and self.conveyor_objects:
        for obj in self.conveyor_objects:
          actual_state = self.obj_states.get_pose(obj)
          actual_state.pose.position.y -= 0.002
          if actual_state.pose.position.y > -0.95:
            self.obj_states.set_pose(obj, actual_state)
          else:
            self.remove_object(obj)

        time.sleep(0.02/len(self.conveyor_objects))


  def start(self):
    self.start_stop_flag = 1
    self.spawn_timer = threading.Timer(SPAWNER_PERIOD, self.cycle_spawn_pcb)
    self.spawn_timer.start()

  def stop(self):
    self.start_stop_flag = 0
    self.spawn_timer.cancel()

if __name__ == "__main__":
  test = Conveyor()
  test.start()
  test.task()
