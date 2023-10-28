#!/usr/bin/env python3
import sys
import rospy
import rospkg
import geometry_msgs.msg
import tf_conversions
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import GetModelState
from math import pi

class ModelPose:

  def goal_pose(self, xyz = {'x': 0, 'y': 0, 'z': 0.25}):
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal_quaternion = tf_conversions.transformations.quaternion_from_euler(-pi/2, 0, 0)
    pose_goal.orientation.x, pose_goal.orientation.y, pose_goal.orientation.z, pose_goal.orientation.w = pose_goal_quaternion
    pose_goal.position.x = xyz['x']
    pose_goal.position.y = xyz['y']
    pose_goal.position.z = xyz['z']
    return pose_goal

  def set_pose(self, obj_name, state):
    goal_state = ModelState()
    goal_state.model_name = obj_name
    goal_state.pose.position = state.pose.position
    goal_state.pose.orientation = state.pose.orientation

    rospy.wait_for_service('/gazebo/set_model_state')
    try:
      set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
      resp = set_state(goal_state)
    except rospy.ServiceException:
      print("Service failed")

  def get_pose(self, obj_name):
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
      get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
      state = get_state(obj_name, 'world')
      return state
    except rospy.ServiceException:
      print("Service failed")

  def set_angle(self, model_name, state, roll, pitch, yaw):
    goal_state = ModelState()
    goal_state.model_name = model_name
    goal_state.pose.position = state.pose.position
    goal_state.pose.orientation = state.pose.orientation
    # print(goal_state.pose.orientation)
    quat = quaternion_from_euler(roll, pitch, yaw)
    # print(quat)
    goal_state.pose.orientation.x = quat[0]
    goal_state.pose.orientation.y = quat[1]
    goal_state.pose.orientation.z = quat[2]
    goal_state.pose.orientation.w = quat[3]

    rospy.wait_for_service('/gazebo/set_model_state')
    try:
      set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
      resp = set_state(goal_state)
    except rospy.ServiceException:
      print("Service failed")

  def get_angle(self, model_name):
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
      get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
      state = get_state(model_name, 'world')
      roll, pitch, yaw = euler_from_quaternion([state.pose.orientation.x, state.pose.orientation.y, 
      				state.pose.orientation.z, state.pose.orientation.w])
      return state, roll, pitch, yaw
    except rospy.ServiceException:
      print("Service failed")
