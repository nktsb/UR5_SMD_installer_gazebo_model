#!/usr/bin/env python3
import cv2
import rospy
import numpy as np
from math import sqrt

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()
online_en = 0
rectangle = 0
new_val = 0

def cam_init():
    image_sub = rospy.Subscriber("/camera/image_raw", Image, callback)

def callback(data):
    if online_en == 1:
        global rectangle
        global new_val
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
            rectangle = corner_calc(box)
            new_val = 1
            cv2.putText(image, str(rectangle), (box[1][0] + 5, box[1][1] + 5), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 210, 150), 3, cv2.LINE_AA)
            cv2.drawContours(image, [box], 0, (255, 210, 150), 3)

        cv2.imshow("Camera output", image)
        cv2.waitKey(100)
    else:
        cv2.destroyAllWindows()

def corner_calc(points):
    x1, y1 = points[0]
    x2, y2 = points[1]
    x3, y3 = points[2]
    if(sqrt((x3 - x2)**2 + (y3 - y2)**2) > sqrt((x1 - x2)**2 + (y1 - y2)**2)):
        x1 = x3
        y1 = y3
    return round(np.arctan((x1 - x2)/(y1 - y2)), 1)