#!/usr/bin/env python3
import cv2
import rospy
import rospkg
from math import sqrt
from std_msgs.msg import Bool
from std_srvs.srv import Empty

class Gripper:
  def grasp_on(self):
    rospy.wait_for_service('/ur5_sub/vacuum_gripper/off')
    try:
      turn_on = rospy.ServiceProxy('/ur5_sub/vacuum_gripper/on', Empty)
      resp = turn_on()
      return resp
    except rospy.ServiceException:
      print("Service call failed")

  def grasp_off(self):
    rospy.wait_for_service('/ur5_sub/vacuum_gripper/off')
    try:
      turn_off = rospy.ServiceProxy('/ur5_sub/vacuum_gripper/off', Empty)
      resp = turn_off()
      return resp
    except rospy.ServiceException:
      print("Service call failed")