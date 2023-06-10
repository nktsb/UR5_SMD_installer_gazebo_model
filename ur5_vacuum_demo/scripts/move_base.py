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

boxes = [[0.3, 0.35, 0.21, 'R'], [0.3, 0.2, 0.21, 'C']]	# boxes coordinates and components type
camera_stand = [0, 0.3, 0.25] #
pcb_origin = [-0.34, 0.175, 0.21]

pcb_components = list()

def load_coordinates():
  rospack = rospkg.RosPack()
  global pcb_components
  with open(rospack.get_path('ur5_vacuum_demo')+'/scripts/pcb_components.csv', newline='') as csvfile:
    spamreader = csv.reader(csvfile, delimiter=' ', quotechar='|')
    for row in spamreader:
      pcb_components.append([a for a in row[0].split(',')])

def algorithm():
  for comp in pcb_components:
    c = 0
    while True:
      if comp[0][0] == boxes[c][3]:
        break
      c += 1

    move_arm(up(boxes[c]))
    move_arm(down(boxes[c]))
    gripper.grasp_on()
    move_arm(up(boxes[c]))

    move_arm(goal_pose_calc(camera_stand))
    rectangle.online_en = 1
    rospy.sleep(2)
    print(rectangle.rectangle)
    rectangle.online_en = 0

    coord = [float(a) for a in comp[1:]]
    coord.append(0.2)

    coord[0] += pcb_origin[0]
    coord[1] += pcb_origin[1]

    move_arm(up(coord))
    move_arm(down(coord))

    gripper.grasp_off()

    move_arm(up(coord))
    move_arm(goal_pose_calc(camera_stand))

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

def alg_init():
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node("algorithm", anonymous=True)
  robot = moveit_commander.RobotCommander()
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
  global group_arm
  group_arm = moveit_commander.MoveGroupCommander("arm")
  rectangle.cam_init()

if __name__ == '__main__':
  alg_init()
  load_coordinates()
  algorithm()
    
