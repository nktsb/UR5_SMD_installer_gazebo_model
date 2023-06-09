#!/usr/bin/env python3
import cv2
import rospy
import numpy as np
from math import sqrt

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def camera():
    rospy.init_node('camera_read', anonymous=False)
    image_sub = rospy.Subscriber("/camera/image_raw", Image, callback)

def corner_calc(points):
    x1, y1 = points[0]
    x2, y2 = points[1]
    x3, y3 = points[2]
    if(sqrt((x3 - x2)**2 + (y3 - y2)**2) > sqrt((x1 - x2)**2 + (y1 - y2)**2)):
        x1 = x3
        y1 = y3
    return round(np.arctan((x1 - x2)/(y1 - y2)), 3)

def callback(data):
    bridge = CvBridge()

    orig_image = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
    image = orig_image[165:315, 165:315] #crop
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) #grayscale
    blur = cv2.GaussianBlur(gray, (9,9), 0)
    thresh = cv2.adaptiveThreshold(blur, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 51, 2)

    contours, hier = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    rect = cv2.minAreaRect(contours[0])
    box = cv2.boxPoints(rect)
    box = np.int0(box)
    if len(box) == 4:
        print(corner_calc(box))
    cv2.drawContours(image, [box], 0, (0,255,0), 3)
    cv2.imshow("Camera output", image)
    cv2.waitKey(100)
    
camera()
rospy.spin()
