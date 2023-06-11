#!/usr/bin/env python3
import sys
import rospy
import rospkg
from math import pi
import csv
import numpy as np
from tf.transformations import euler_from_quaternion
import moving
import rectangle
import rotation
import gripper

accuracy = 3

boxes = [[0.3, 0.35, 0.2, 'R'], [0.3, 0.2, 0.2, 'C']] # boxes coordinates and components type
camera_stand = [0, 0.3, 0.25] #
pcb_origin = [-0.34, 0.175, 0.21]

pcb_components = list()

def load_coordinates():
  rospack = rospkg.RosPack()
  global pcb_components
  with open(rospack.get_path('ur5_vacuum_demo')+'/scripts/pcb_components.csv', newline='') as csvfile:
    spamreader = csv.reader(csvfile, delimiter=' ', quotechar='|')
    c = 0
    for row in spamreader:
      pcb_components.append([a for a in row[0].split(',')])
      pcb_components[c].append(0) #installing status
      c += 1
  #print(pcb_components)

def algorithm():
  counter = 0
  succes = 0 #number of succesfully installed components
  while True:
    if pcb_components[counter][4] == 0: #comonent still wasn't isntalled
      box_num = 0
      while True:
        if pcb_components[counter][0][0] == boxes[box_num][3]: #find component type (number of box)
          break
        box_num += 1

      comp_name = 'comp_' + str(box_num + 1) #name of model in gazebo
      comp_angle = pcb_components[counter][3] #goal angle
      comp_angle = round(float(comp_angle), accuracy)

      moving.move_arm(moving.up(boxes[box_num])) #go to box
      moving.move_arm(moving.down(boxes[box_num])) #move gripper down
      gripper.grasp_on() #turn on gripper
      moving.move_arm(moving.up(boxes[box_num])) #move gripper up

      moving.move_arm(moving.goal_pose_calc(camera_stand)) #go to camera

      rectangle.online_en = 1 #enable openCV angle determining
      
      while True:
        if rectangle.new_val or rectangle.rectangle: #new value or error code
          break

      rectangle.new_val = 0

      actual_state = rotation.get_state(comp_name)
      roll, pitch, yaw = euler_from_quaternion([actual_state.pose.orientation.x, actual_state.pose.orientation.y, actual_state.pose.orientation.z, actual_state.pose.orientation.w])
      
      angle = yaw

      if rectangle.rectangle != 0xFE: #there is component

        while True:
          if rectangle.new_val:
            rectangle.new_val = 0

            actual_state = rotation.get_state(comp_name)
            roll, pitch, yaw = euler_from_quaternion([actual_state.pose.orientation.x, actual_state.pose.orientation.y, actual_state.pose.orientation.z, actual_state.pose.orientation.w])
            
            angle += (comp_angle - yaw)*0.1
            #rospy.sleep(0.5)
          
            rotation.set_state(comp_name, actual_state, roll, pitch, angle) #rotate component
            if round(rectangle.rectangle, accuracy) == comp_angle:
              break
          # if angle >= round(pi*2, 3):
          #   angle = 0
          print(comp_angle, round(rectangle.rectangle, accuracy), round(angle, accuracy), end = '\r')
          

        rectangle.online_en = 0
        comp_coord = [float(a) for a in pcb_components[counter][1:3]]
        comp_coord.append(0.2) # Z axis for component drop

        comp_coord[0] += pcb_origin[0]
        comp_coord[1] += pcb_origin[1]

        moving.move_arm(moving.up(comp_coord))
        moving.move_arm(moving.down(comp_coord))
        gripper.grasp_off()
        moving.move_arm(moving.up(comp_coord))
        pcb_components[counter][4] = 1 #install current component ok
        succes += 1
      else:
        rectangle.online_en = 0

    counter += 1
    if counter >= len(pcb_components): # try again if some component wasn't succesfully installed
      counter = 0
    if succes == len(pcb_components): # all components installed - finish algorithm
      break

def alg_init():
  rospy.init_node("algorithm", anonymous=True)
  moving.add_walls() # add walls to scene
  moving.init() # init moveit
  rectangle.cam_init() # init camera and openCV

if __name__ == '__main__':
  try:
    alg_init()
    load_coordinates()
    algorithm()
    moving.move_arm(moving.goal_pose_calc(camera_stand))
  except KeyboardInterrupt:
    quit()