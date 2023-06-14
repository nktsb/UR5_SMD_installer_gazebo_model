#!/usr/bin/env python3
import cv2
import rospy
import rospkg
from math import sqrt
from std_msgs.msg import Bool
from std_srvs.srv import Empty

def gripper_status(msg):
    if msg.data:
        return True
        print('gripper status = {}'.format(msg.data))

gripper_status_sub = rospy.Subscriber('/ur5_sub/vacuum_gripper/grasping', Bool, gripper_status, queue_size=1)

def grasp_on():
  rospy.wait_for_service('/ur5_sub/vacuum_gripper/off')
  try:
    turn_on = rospy.ServiceProxy('/ur5_sub/vacuum_gripper/on', Empty)
    resp = turn_on()
    return resp
  except rospy.ServiceException:
    print("Service call failed")

def grasp_off():
  rospy.wait_for_service('/ur5_sub/vacuum_gripper/off')
  try:
    turn_off = rospy.ServiceProxy('/ur5_sub/vacuum_gripper/off', Empty)
    resp = turn_off()
    return resp
  except rospy.ServiceException:
    print("Service call failed")