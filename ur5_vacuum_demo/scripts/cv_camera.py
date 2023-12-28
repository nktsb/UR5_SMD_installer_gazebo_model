#!/usr/bin/env python3
import cv2
import rospy
import numpy as np
import time
from math import sqrt
from math import pi
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

ERROR_CODE = 0xDEAD

class Camera:
    def __init__(self, topic_name, callback_idx):
        callbacks_list = {
            'table': self.table_callback, 
            'gripper': self.gripper_callback
        }
        self.online_en = 0
        self.new_val_flg = 0
        self.angle = 0
        self.image_sub = rospy.Subscriber(topic_name, Image, callbacks_list[callback_idx])
        self.bridge = CvBridge()

    def get_cv_angle(self):
        return self.angle

    def processing_en(self):
        self.online_en = 1

    def processing_dis(self):
        self.online_en = 0
        cv2.destroyAllWindows()

    def get_new_val_flg(self):
        return self.new_val_flg

    def clr_new_val_flg(self):
        self.new_val_flg = 0

    def gripper_callback(self, data):
        print("Gripper")
        if self.online_en == 1:
            orig_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
            # cv2.imshow("Orig", orig_image)
            gray = cv2.cvtColor(orig_image, cv2.COLOR_BGR2GRAY) #grayscale
            # cv2.imshow("Gray", gray)
            blur = cv2.GaussianBlur(gray, (13,13), 0)
            # cv2.imshow("Blur", blur)
            flg, thresh = cv2.threshold(blur, 100, 255, cv2.THRESH_OTSU)
            
            # cv2.imshow("Processed", thresh)
            
            contours, hier = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            try:
                rect = cv2.minAreaRect(contours[0])
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                if len(box) == 4:
                    a, b = self.box_size_calc(box)
                    if a > 150 and b > 150:
                        center_x, center_y = self.box_center_calc(box)
                        center = (int(center_x), int(center_y))
                        image = cv2.circle(orig_image, center, radius = 3, color=(0, 50, 255), thickness=6)
                        cv2.putText(image, (str(round(a))+"x"+str(round(b))), (box[1][0] + 5, box[1][1] + 5), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 210, 150), 3, cv2.LINE_AA)
                        cv2.drawContours(image, [box], 0, (255, 210, 150), 3)

                        if (250 > center_y > 230) and (380 > center_x > 100):
                            # print("PCB found!")
                            self.new_val_flg = 1
            except:
                self.new_val_flg = ERROR_CODE
                print("No components here")

            cv2.imshow("Gripper image", orig_image)
            cv2.waitKey(100)
            print("quit gripper")

    def table_callback(self, data):
        print("Table")
        if self.online_en == 1:
            orig_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
            # cv2.imshow("Orig", orig_image)
            image = orig_image[80:400, 80:400] #crop
            # cv2.imshow("Crop", image)
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) #grayscale
            # cv2.imshow("Gray", gray)
            blur = cv2.GaussianBlur(gray, (9,9), 0)
            # cv2.imshow("Blur", blur)
            thresh = cv2.adaptiveThreshold(blur, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 101, 3)
            # cv2.imshow("Processed", thresh)
            
            contours, hier = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            try:
                rect = cv2.minAreaRect(contours[0])
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                if len(box) == 4:
                    self.angle = self.angle_calc(box)
                    if self.angle != ERROR_CODE:
                        self.new_val_flg = 1
                    else:
                        self.new_val_flg = ERROR_CODE

                    cv2.putText(image, str(round(self.angle, 3)), (box[1][0] + 5, box[1][1] + 5), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 210, 150), 3, cv2.LINE_AA)
                    cv2.drawContours(image, [box], 0, (255, 210, 150), 3)
            except:
                self.new_val_flg = ERROR_CODE
                print("No components here")
            # cv2.imshow("Table image", image)
            cv2.waitKey(100)
            print("quit table")

    def box_center_calc(self, points):
        x1, y1 = points[0]
        x2, y2 = points[1]
        x3, y3 = points[2]
        x4, y4 = points[3]

        x_av = (x1 + x2 + x3 + x4)/4
        y_av = (y1 + y2 + y3 + y4)/4

        return x_av, y_av

    def box_size_calc(self, points):
        x1, y1 = points[0]
        x2, y2 = points[1]
        x3, y3 = points[2]

        first_side = sqrt((x1 - x2)**2 + (y1 - y2)**2)
        second_side = sqrt((x3 - x2)**2 + (y3 - y2)**2)

        return first_side, second_side

    def angle_calc(self, points):
        x1, y1 = points[0]
        x2, y2 = points[1]
        x3, y3 = points[2]

        if ((x3 - x2)**2 + (y3 - y2)**2) > ((x1 - x2)**2 + (y1 - y2)**2):
            x1 = x3
            y1 = y3

        first_side = x2 - x1
        second_side = y2 - y1

        if second_side and first_side == 0:
            return round(pi/2, 5)

        elif first_side and second_side == 0:
            return 0

        elif first_side and second_side:
            return round(pi/2 - np.arctan((first_side)/(second_side)), 5)

        else:
            return ERROR_CODE

if __name__ == '__main__':
    try:
        rospy.init_node('cam_read', anonymous=False)
        test_1 = Camera("/camera/image_raw", 'table')
        test_2 = Camera("/gripper_camera/image_raw", 'gripper')
        
        test_1.processing_en()
        time.sleep(2)
        test_1.processing_dis()
        time.sleep(2)

        test_2.processing_en()
        # test_2.processing_en()
        while True:
            pass
    except KeyboardInterrupt:
        quit()