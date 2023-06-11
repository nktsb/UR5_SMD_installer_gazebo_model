#!/usr/bin/env python3
import sys
import rospy
import rospkg
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import csv
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from math import pi
import tf_conversions
import rectangle
import gripper
import rotation

boxes = [[0.3, 0.35, 0.2, 'R'], [0.3, 0.2, 0.2, 'C']]	# boxes coordinates and components type
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
  print(pcb_components)

def algorithm():
  counter = 0
  succes = 0 
  while True:
    if pcb_components[counter][4] == 0: #comonent still wasn't isntalled
      box_num = 0
      while True:
        if pcb_components[counter][0][0] == boxes[box_num][3]: #find component type
          break
        box_num += 1

      comp_name = 'comp_'+str(box_num+1)
      comp_angle = pcb_components[counter][3]

      move_arm(up(boxes[box_num]))
      move_arm(down(boxes[box_num]))
      gripper.grasp_on()
      move_arm(up(boxes[box_num]))

      move_arm(goal_pose_calc(camera_stand))

      rectangle.online_en = 1
      
      while True:
        if rectangle.new_val or rectangle.rectangle:
          break

      rectangle.new_val = 0

      if rectangle.rectangle != 0xFE: #there is component
        angle = round(rectangle.rectangle, 2)
        actual_state = rotation.get_state(comp_name)
        while True:
          if rectangle.new_val:
            rectangle.new_val = 0
            rotation.set_state(comp_name, actual_state, round(angle, 2))
            angle += 0.05
          if angle >= pi:
            angle = 0
          print(round(angle, 2), rectangle.rectangle)
          if rectangle.rectangle == float(comp_angle):
            break

        rectangle.online_en = 0
        
        comp_coord = [float(a) for a in pcb_components[counter][1:3]]
        comp_coord.append(0.2) # Z axis for component drop

        comp_coord[0] += pcb_origin[0]
        comp_coord[1] += pcb_origin[1]

        move_arm(up(comp_coord))
        move_arm(down(comp_coord))
        gripper.grasp_off()
        move_arm(up(comp_coord))
        pcb_components[counter][4] = 1 #install ok
        succes += 1
      else:
        rectangle.online_en = 0
    
    counter += 1

    if counter >= len(pcb_components):
      counter = 0

    if succes == len(pcb_components):
      break

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

def alg_init():
  rospy.init_node("algorithm", anonymous=True)
  add_walls()
  global group_arm
  moveit_commander.roscpp_initialize(sys.argv)
  group_arm = moveit_commander.MoveGroupCommander("arm")
  rectangle.cam_init()

if __name__ == '__main__':
  try:
    alg_init()
    load_coordinates()
    algorithm()
    move_arm(goal_pose_calc(camera_stand))
  except KeyboardInterrupt:
    quit()
