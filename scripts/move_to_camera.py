#!/usr/bin/python3

import rospy
from std_msgs.msg import Float64

LIFT_SH = 	["/ur5_sub/shoulder_lift_position_controller/command", -2.05]
PAN_SH = 	["/ur5_sub/shoulder_pan_position_controller/command", 	0.50]
ELBOW_SH = 	["/ur5_sub/elbow_position_controller/command", 		   -1.01]
WRIST1_SH = ["/ur5_sub/wrist_1_position_controller/command", 	   -1.01]
WRIST2_SH = ["/ur5_sub/wrist_2_position_controller/command", 	   -1.00]
WRIST3_SH = ["/ur5_sub/wrist_3_position_controller/command", 	   -1.55]

rospy.init_node('hey', anonymous = True)
rate = rospy.Rate(50)

def publish_state(topic, position):
	p = rospy.Publisher(topic, Float64, queue_size = 1)
	p.publish(position)
	rate.sleep()

def publisher():

	publish_state(LIFT_SH[0], LIFT_SH[1])
	publish_state(PAN_SH[0], PAN_SH[1])
	publish_state(ELBOW_SH[0], ELBOW_SH[1])
	publish_state(WRIST1_SH[0], WRIST1_SH[1])
	publish_state(WRIST2_SH[0], WRIST2_SH[1])
	publish_state(WRIST3_SH[0], WRIST2_SH[1])

if __name__ == '__main__':
	try:
		publisher()
	except rospy.ROSInterruptException:
		pass
