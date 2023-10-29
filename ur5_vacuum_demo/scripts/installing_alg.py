#!/usr/bin/env python3
import os
import sys
import rospy
import rospkg
from math import pi
import random
import csv
import numpy as np
from tf.transformations import euler_from_quaternion
import roslaunch
import threading
import time

from mstates import ModelPose
from conveyor import Conveyor
from gripper import Gripper
from moving import SMD_InstallerMoveSet
from cv_camera import Camera

conveyor = Conveyor()
moving = SMD_InstallerMoveSet()
table_cam = Camera()

accuracy = 3

boxes = [
{
  'x': 0.3, 
  'y': 0.35, 
  'z': 0.225, 
  'type': 'R',
  'component': ""
}, 
{
  'x': 0.3, 
  'y': 0.2, 
  'z': 0.225, 
  'type': 'C',
  'component': ""
}
] # boxes coordinates and components type


camera_stand = {
  'x': 0, 
  'y': 0.3, 
  'z': 0.22
}

pcb_origin = {
  'x': -0.34, 
  'y': 0.175, 
  'z':0.22
}

pcb_components = list()

def load_coordinates():
  rospack = rospkg.RosPack()
  global pcb_components
  with open(rospack.get_path('ur5_vacuum_demo')+'/scripts/pcb_components.csv', newline='') as csvfile:
    spamreader = csv.reader(csvfile, delimiter=' ', quotechar='|')
    cnt = 0
    for row in spamreader:
      pcb_components.append({})
      row_data = [cell for cell in row[0].split(',')]
      pcb_components[cnt] = {
        'type': row_data[0][0],
        'x': row_data[1],
        'y': row_data[2],
        'angle': row_data[3],
        'status': 0 #installing status
      }
      cnt += 1
  # print(pcb_components)

def spawn_components(component_1, component_2):
  yaw_1 = round(random.uniform(0, pi), 3)
  yaw_2 = round(random.uniform(0, pi), 3)
  os.system('roslaunch ur5_vacuum_demo components.launch comp_1_name:="' + str(component_1) + 
              '" comp_2_name:="' + str(component_2) + '" yaw_1:="' + str(yaw_2) + '" yaw_2:="' + str(yaw_1) + '"')

def place_comp_to_boxes(comp_1_type, comp_2_type, index):

    component_1 = 'comp_' + str(index + 1)
    component_2 = 'comp_' + str(index + 2)

    for box in boxes:
      if(box['type'] == comp_1_type):
        boxes[boxes.index(box)]['component'] = component_1
      elif(box['type'] == comp_2_type):
        boxes[boxes.index(box)]['component'] = component_2

    spawn_components(component_2, component_1)

def find_box(type):
  for box in boxes:
    if(box['type'] == type):
      return box

def algorithm():
  conveyor.start()
  time.sleep(10)
  model_counter = 0

  while True:
    counter = 0
    succes = 0

    while True:

      if counter % 2 == 0:
        conveyor.stop()
        place_comp_to_boxes(pcb_components[counter]['type'], 
                            pcb_components[counter + 1]['type'], 
                            model_counter)

      box = find_box(pcb_components[counter]['type'])
      component = box['component']
      goal_angle = pcb_components[counter]['angle']
      goal_angle = round(float(goal_angle), accuracy)

      moving.move_and_take(box) #go to box
      moving.move_arm(camera_stand) #go to camera

      # conveyor.stop()
      table_cam.processing_en() #enable openCV angle determining

      while True:

        if table_cam.get_new_val_flg() and table_cam.get_new_val_flg() != 0xDEAD:

          table_cam.clr_new_val_flg()

          cv_angle = round(table_cam.get_cv_angle(), accuracy)
          
          if cv_angle == goal_angle or cv_angle == (goal_angle + round(pi, accuracy)):
            break

          moving.rotate_gripper(component, goal_angle)
        
        elif table_cam.get_new_val_flg() == 0xDEAD:   
          table_cam.clr_new_val_flg()     
          break;

      table_cam.processing_dis()

      component_xyz = {
        'x': float(pcb_components[counter]['x']),
        'y': float(pcb_components[counter]['y']),
        'z': 0
      }

      component_xyz['x'] += pcb_origin['x']
      component_xyz['y'] += pcb_origin['y']
      component_xyz['z'] = pcb_origin['z']

      moving.move_and_release(component_xyz)
      conveyor.put_object(component)

      pcb_components[counter]['status'] = 1 #install current component ok

      succes += 1

      counter += 1
      model_counter += 1

      if succes == len(pcb_components): # all components installed - finish algorithm
        # moving.move_arm(camera_stand)
        break

    conveyor.start()
    time.sleep(8)


def alg_init():
  load_coordinates()
  rospy.init_node("algorithm", anonymous=True)
  moving.add_walls() # add walls to scen

if __name__ == '__main__':
  try:
    
    alg_init()

    algorithm_task = threading.Thread(target = algorithm)
    conveyor_task = threading.Thread(target = conveyor.task)
    
    conveyor_task.start()
    algorithm_task.start()
    
  except KeyboardInterrupt:
    quit()
