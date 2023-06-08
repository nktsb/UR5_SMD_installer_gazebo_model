#!/usr/bin/env python3
import cv2
import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def camera():
    rospy.init_node('camera_read', anonymous=False)
    image_sub = rospy.Subscriber("/camera/image_raw", Image, callback)
    
def callback(data):
    bridge = CvBridge()

    orig_image = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
    image = orig_image[140:340, 140:340] #crop
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) #grayscale
    blur = cv2.GaussianBlur(gray, (9,9), 0)
    thresh = cv2.adaptiveThreshold(blur, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 51, 3)

    contours, hier = cv2.findContours(thresh, 1, 2)
    #print("Number of detected contours", len(contours))
    
    for cnt in contours:
    	x1, y1 = cnt[0][0]
    	approx = cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt, True), True)
    	if len(approx) == 4:
            x, y, w, h = cv2.boundingRect(cnt)
            ratio = float(w)/h
            cv2.putText(image, 'SMD'+str(ratio), (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            image = cv2.drawContours(image, [cnt], -1, (0,255,0), 3)

    cv2.imshow("Camera output", image)
    cv2.waitKey(1000)
    
camera()
rospy.spin()

