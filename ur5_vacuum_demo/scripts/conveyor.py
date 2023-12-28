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
    self.timer_counter = 0
    self.spawn_timer = threading.Timer(TIMER_PERIOD, self.cycle_spawn_pcb)

  def cycle_spawn_pcb(self):
    if self.start_stop_flag == 1: 
      if self.timer_counter == 0:
        self.pcb_counter += 1
        new_pcb = 'pcb_' + str(self.pcb_counter)
        self.spawn_pcb(new_pcb)
        self.put_object(new_pcb)

      self.timer_counter += 1
      if self.timer_counter == SPAWNER_PERIOD / TIMER_PERIOD:
        self.timer_counter = 0

    self.spawn_timer = threading.Timer(TIMER_PERIOD, self.cycle_spawn_pcb)
    self.spawn_timer.start()

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

  def spawn_pcb(self, pcb_name):
    yaw = round(random.uniform(0, pi), 3)
    os.system('roslaunch ur5_vacuum_demo pcb.launch pcb_name:="' + str(pcb_name) + 
                '" yaw:="' + str(yaw) + '"')

  def task(self):
    while True:
      if self.start_stop_flag == 1 and self.conveyor_objects:
        for obj in self.conveyor_objects:
          obj['state'].pose.position.y -= 0.004

          if obj['state'].pose.position.y > -0.95:
            state = ModelPose()
            state.set_pose(obj['name'], obj['state'])
          else:
            self.remove_object(obj)

        time.sleep(0.04/len(self.conveyor_objects))


  def start(self):
    self.start_stop_flag = 1
    self.spawn_timer = threading.Timer(TIMER_PERIOD, self.cycle_spawn_pcb)
    self.spawn_timer.start()

  def stop(self):
    self.start_stop_flag = 0
    self.spawn_timer.cancel()

if __name__ == "__main__":
  test = Conveyor()
  test.start()
  test.task()
