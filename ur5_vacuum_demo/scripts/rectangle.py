#!/usr/bin/env python3
import cv2
import rospy
import numpy as np
from math import sqrt
from math import pi
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
        # cv2.imshow("Orig", orig_image)
        image = orig_image[40:440, 40:440] #crop
        # cv2.imshow("Crop", image)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) #grayscale
        # cv2.imshow("Gray", gray)
        blur = cv2.GaussianBlur(gray, (15,15), 0)
        # cv2.imshow("Blur", blur)
        thresh = cv2.adaptiveThreshold(blur, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 101, 3)
        
        # cv2.imshow("Processed", thresh)
        
        contours, hier = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        try:
            rect = cv2.minAreaRect(contours[0])
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            if len(box) == 4:
                rectangle = corner_calc(box)
                new_val = 1
                cv2.putText(image, str(round(rectangle, 3)), (box[1][0] + 5, box[1][1] + 5), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 210, 150), 3, cv2.LINE_AA)
                cv2.drawContours(image, [box], 0, (255, 210, 150), 3)
        except:
            rectangle = 0xFE
            print("No components here")

        cv2.imshow("Final image", image)
        cv2.waitKey(100)
    else:
        cv2.destroyAllWindows()

def corner_calc(points):
    x1, y1 = points[0]
    x2, y2 = points[1]
    x3, y3 = points[2]
    if ((x3 - x2)**2 + (y3 - y2)**2) > ((x1 - x2)**2 + (y1 - y2)**2):
        x1 = x3
        y1 = y3
    if x1 and y1 and x2 and y2:
        return round(pi/2 + np.arctan((x2 - x1)/(y2 - y1)), 5)
    else:
        return 0xFE

if __name__ == '__main__':
    try:
        rospy.init_node('cam_read', anonymous=False)
        cam_init()
        online_en = 1
        while True:
            pass
    except KeyboardInterrupt:
        quit()