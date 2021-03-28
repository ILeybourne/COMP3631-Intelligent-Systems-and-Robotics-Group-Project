#!/usr/bin/env python
# This final piece of skeleton code will be centred around
# to follow a colour and stop upon sight of another one.

from __future__ import division
import cv2
import numpy as np
import rospy
import sys

from geometry_msgs.msg import Twist, Vector3, Pose, Point, Quaternion
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError
from actionlib_msgs.msg import *


class rectangleIdentification():
    def __init__(self):
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber('image_topic', Image, self.imageCallback)
        self.pub_rectangle = rospy.Publisher('rectangle_topic', Bool, queue_size=10)

        # Initialise sensitivity variable for colour detection
        self.hue_sensitivity = 10
        self.sat_sensitivity = 10
        self.val_sensitivity = 50

        # self.pub_circle_based_velocity = rospy.Publisher('mobile_base/commands/velocity', Twist)
        #
        # self.desired_velocity = Twist()
        # self.forward = 0.2
        # self.backwards = -0.2
        # self.left = -0.2
        # self.right = 0.2
        # self.stop = 0
        #
        # self.character_x = 0
        # self.character_y = 0
        # self.character_w = 0
        # self.character_h = 0
        #
        # self.image_x = 0
        # self.image_y = 0

    def imageCallback(self, data):
        # Try to convert from image message to cv2 Image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data)
        except CvBridgeError as e:
            print("Image conversion failed")
            print(e)
            pass

        hue_lower = 220.0 / 2
        hue_upper = 360.0 / 2
        saturation_lower = 0.0 / 100 * 255
        saturation_upper = 15.0 / 100 * 255
        value_lower = 0.0 / 100 * 255
        value_upper = 10.0 / 100 * 255

        hsv_black_lower = np.array([hue_lower - self.hue_sensitivity, saturation_lower - self.sat_sensitivity,
                                    value_lower - self.val_sensitivity])
        hsv_black_upper = np.array([hue_upper + self.hue_sensitivity, saturation_upper + self.sat_sensitivity,
                                    value_upper + self.val_sensitivity])

        # Convert the rgb image into a hsv image
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Filter out everything but predefined colours
        mask_black = cv2.inRange(hsv_image, hsv_black_lower, hsv_black_upper)

        # Apply the mask to the original image
        mask_image_black = cv2.bitwise_and(cv_image, cv_image, mask=mask_black)

        self.findRectangle(self, mask_image_black)

    def findRectangle(self, cI, cv_image):
        # Copy input image
        output = cv_image.copy()

        # Find all fully black pixels
        black_pixels = np.where(
            (cv_image[:, :, 0] == 0) &
            (cv_image[:, :, 1] == 0) &
            (cv_image[:, :, 2] == 0)
        )
        # Change black pixels to white
        cv_image[black_pixels] = [255, 255, 255]

        # Find all none white pixels
        other_pixels = np.where(
            (cv_image[:, :, 0] != 255) &
            (cv_image[:, :, 1] != 255) &
            (cv_image[:, :, 2] != 255)
        )




        print(len(other_pixels[0]))
        rect_flag = False
        # If there exist more than 2 non white pixels draw a rectangle from first to last pixels and then set rectangle flag to true
        if len(other_pixels[0]) >= 50:
            rect_flag = True
            # Find first and last none white pixels
            min_in_x = min(other_pixels[0])
            min_in_y = min(other_pixels[1])
            max_in_x = max(other_pixels[0])
            max_in_y = max(other_pixels[1])
            cv2.rectangle(output, (min_in_y, min_in_x), (max_in_y, max_in_x), (255, 255, 255), 3)

        self.pub_rectangle.publish(rect_flag)

        # Debugging
        # cv2.imshow("output rectangle", np.hstack([cv_image, output]))
        # cv2.waitKey(3)

def main(args):
    rospy.init_node('Circle_Finder', anonymous=True)
    rI = rectangleIdentification()
    print("Initializing rectangle finder")
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


# Check if the node is executing in the main path
if __name__ == '__main__':
    main(sys.argv)
