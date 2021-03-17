#!/usr/bin/env python
# This final piece of skeleton code will be centred around
# to follow a colour and stop upon sight of another one.

from __future__ import division
import cv2
import cv
import numpy as np
import rospy
import sys

from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError
from actionlib_msgs.msg import *

class circleFinder():
    def __init__(self):
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber('image_topic', Image, self.imageCallback)
        self.green_circle_flag = False
        self.red_circle_flag = False

        # Initialise sensitivity variable for colour detection
        self.Rsensitivity = 1
        self.Gsensitivity = 20
        self.pub_red_circle = rospy.Publisher('red_circle_topic', Bool, queue_size=10)
        self.pub_green_circle = rospy.Publisher('green_circle_topic', Bool, queue_size=10)


    def imageCallback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data)
        except CvBridgeError as e:
            print("Image conversion failed")
            print(e)
            pass

        # Set the upper and lower bounds for red and green circles
        hsv_red_lower = np.array([0 - self.Rsensitivity, 100, 100])
        hsv_red_upper = np.array([0 + self.Rsensitivity, 255, 255])
        hsv_green_lower = np.array([60 - self.Gsensitivity, 50, 50])
        hsv_green_upper = np.array([60 + self.Gsensitivity, 255, 255])

        # Convert the rgb image into a hsv image
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Filter out everything but predefined colours
        mask_red = cv2.inRange(hsv_image, hsv_red_lower, hsv_red_upper)
        mask_green = cv2.inRange(hsv_image, hsv_green_lower, hsv_green_upper)

        # Combine masks
        mask_rg = cv2.bitwise_or(mask_red, mask_green)

        # Apply the mask to the original image
        mask_image_rg = cv2.bitwise_and(cv_image, cv_image, mask=mask_rg)
        mask_image_r = cv2.bitwise_and(cv_image, cv_image, mask=mask_red)
        mask_image_g = cv2.bitwise_and(cv_image, cv_image, mask=mask_green)

        self.findGreenCircle(self, mask_image_g)
        self.findRedCircle(self, mask_image_r)

        cv2.namedWindow('Camera_Feed2')
        cv2.imshow('Camera_Feed2', cv_image)
        cv2.waitKey(3)

    def findGreenCircle(self, cF, cv_image):
        output = cv_image.copy()
        grey_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(grey_image, 100, 200)
        blur = cv2.GaussianBlur(edges, (5, 5), 0)

        # circles = cv2.HoughCircles(blur, cv.CV_HOUGH_GRADIENT, 1.5, 1000, 0, 500)
        circles = cv2.HoughCircles(blur, cv.CV_HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=10, maxRadius=0)

        # For debugging show green greyscale, green edges, blurred edges
        # cv2.imshow("outputgrey green", grey_image)
        # cv2.imshow("output edge green", edges)
        # cv2.imshow("output blur green", blur)

        self.green_circle_flag = False
        if circles is not None:
            circles = np.uint16(np.around(circles))
            self.green_circle_flag = True
            for i in circles[0, :]:
                cv2.circle(output, (i[0], i[1]), i[2], (0, 255, 0), 2)

        self.pub_green_circle.publish(self.green_circle_flag)

        # Debugging show green circle and input image
        # cv2.imshow("output green", np.hstack([cv_image, output]))
        # cv2.waitKey(3)

    def findRedCircle(self, cF, cv_image):
        output = cv_image.copy()
        grey_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(grey_image, 100, 200)
        blur = cv2.GaussianBlur(edges, (5, 5), 0)

        circles = cv2.HoughCircles(blur, cv.CV_HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=10, maxRadius=0)

        # For debugging show red greyscale, red edges, blurred edges
        # cv2.imshow("outputgrey  red", grey_image)
        # cv2.imshow("output edge red", edges)
        # cv2.imshow("output blur red", blur)

        self.red_circle_flag = False
        if circles is not None:
            circles = np.uint16(np.around(circles))
            self.red_circle_flag = True
            for i in circles[0, :]:
                cv2.circle(output, (i[0], i[1]), i[2], (0, 0, 255), 2)

        self.pub_red_circle.publish(self.red_circle_flag)

        # Debugging show red circle and input image
        # cv2.imshow("output red", np.hstack([cv_image, output]))
        # cv2.waitKey(3)

def main(args):
    rospy.init_node('Circle_Finder', anonymous=True)
    cF = circleFinder()
    print("Initializing circle finder")
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

# Check if the node is executing in the main path
if __name__ == '__main__':
    main(sys.argv)
