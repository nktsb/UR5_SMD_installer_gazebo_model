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
import moving
import rectangle
import rotation
import gripper
import roslaunch
import threading
import conveyor as cn

accuracy = 3

boxes = [
{
  'x': 0.3, 
  'y': 0.35, 
  'z': 0.2, 
  'type': 'R'
}, 
{
  'x': 0.3, 
  'y': 0.2, 
  'z': 0.2, 
  'type': 'C'
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
  'z':0.21
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

def spawn_components(comp_name_1, comp_name_2):
  yaw_1 = round(random.uniform(0, pi), 3)
  yaw_2 = round(random.uniform(0, pi), 3)
  os.system('roslaunch ur5_vacuum_demo components.launch comp_1_name:="comp_' + str(comp_name_1) + 
              '" comp_2_name:="comp_' + str(comp_name_2) + '" yaw_2:="' + str(yaw_2) + '" yaw_1:="' + str(yaw_1) + '"')

def algorithm():
  counter = 0
  succes = 0 #number of succesfully installed components
  while True:
    if pcb_components[counter]['status'] == 0: #comonent still wasn't isntalled
      box_num = 0
      while True:
        if pcb_components[counter]['type'] == boxes[box_num]['type']: #find component type (number of box)
          break
        box_num += 1

      comp_name_1 = counter + 2
      comp_name_2 = counter + 1
      
      if not counter % 2:
        spawn_components(comp_name_1, comp_name_2)

      comp_name = 'comp_' + str(counter + 1) #name of model in gazebo
      comp_angle = pcb_components[counter]['angle'] #goal angle
      comp_angle = round(float(comp_angle), accuracy)

      # print("Components spawned")
      moving.move_arm(moving.up(boxes[box_num])) #go to box
      moving.move_arm(moving.down(boxes[box_num])) #move gripper down
      gripper.grasp_on() #turn on gripper
      moving.move_arm(moving.up(boxes[box_num])) #move gripper up

      moving.move_arm(moving.goal_pose_calc(camera_stand)) #go to camera
  
      rectangle.processing_en() #enable openCV angle determining
      
      while True:
        if rectangle.new_val or rectangle.rectangle: #new value or error code
          break

      rectangle.new_val = 0

      actual_state,roll, pitch, yaw = rotation.get_angle(comp_name)
      
      angle = yaw

      if rectangle.rectangle != 0xFE: #there is component

        while True:
          if rectangle.new_val:
            rectangle.new_val = 0

            actual_state, roll, pitch, yaw = rotation.get_angle(comp_name)
            
            angle += (comp_angle - yaw)*0.2
            #rospy.sleep(0.5)
          
            rotation.set_angle(comp_name, actual_state, roll, pitch, angle) #rotate component
            if round(rectangle.rectangle, accuracy) == comp_angle or round(rectangle.rectangle, accuracy) == (comp_angle + round(pi, accuracy)):
              break
          # if angle >= round(pi*2, 3):
          #   angle = 0
          print(comp_angle, round(rectangle.rectangle, accuracy), round(angle, accuracy), end = '\r')
          

        rectangle.processing_dis()

        comp_coord = {
          'x': float(pcb_components[counter]['x']),
          'y': float(pcb_components[counter]['y']),
          'z': 0.2 # Z axis for component drop
        }

        comp_coord['x'] += pcb_origin['x']
        comp_coord['y'] += pcb_origin['y']

        moving.move_arm(moving.up(comp_coord))
        moving.move_arm(moving.down(comp_coord))
        gripper.grasp_off()
        moving.move_arm(moving.up(comp_coord))
        pcb_components[counter]['status'] = 1 #install current component ok
        succes += 1
      else:
        rectangle.online_en = 0

    counter += 1
    if counter >= len(pcb_components): # try again if some component wasn't succesfully installed
      counter = 0
    if succes == len(pcb_components): # all components installed - finish algorithm
      moving.move_arm(moving.goal_pose_calc(camera_stand))
      quit()

def alg_init():
  load_coordinates()
  rospy.init_node("algorithm", anonymous=True)
  moving.add_walls() # add walls to scen

if __name__ == '__main__':
  try:
    rectangle.cam_init() # init camera and openCV
    moving.init() #init moveit
    
    cn.conveyor_init()
    cn.conveyor_start()

    alg_init()

    algorithm_task = threading.Thread(target = algorithm)
    conveyor_task = threading.Thread(target = cn.conveyor_task)
    
    conveyor_task.start()
    algorithm_task.start()
    
  except KeyboardInterrupt:
    quit()
