#!/usr/bin/env python
# This final piece of skeleton code will be centred around
# to follow a colour and stop upon sight of another one.

from __future__ import division
import cv2
import cv
import numpy as np
import rospy
import sys

from geometry_msgs.msg import Twist, Vector3, Pose, Point, Quaternion
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError
from actionlib_msgs.msg import *


class CharacterOutput():
    def __init__(self):
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber('image_topic', Image, self.imageCallback)
        sub = rospy.Subscriber('mustard_topic', Bool, callbackMustard)
        sub = rospy.Subscriber('peacock_topic', Bool, callbackPeacock)
        sub = rospy.Subscriber('plum_topic', Bool, callbackPlum)
        sub = rospy.Subscriber('scarlet_topic', Bool, callbackScarlet)
        sub = rospy.Subscriber('rectangle_topic', Bool, callbackRectangle)

        self.character_printed = False

        # Initialise sensitivity variable for colour detection
        self.r_sensitivity = 5
        self.Ysensitivity = 10
        self.Psensitivity = 10
        self.Bsensitivity = 10
        self.pub_mustard = rospy.Publisher('mustard_topic', Bool, queue_size=10)
        self.pub_peacock = rospy.Publisher('peacock_topic', Bool, queue_size=10)
        self.pub_plum = rospy.Publisher('plum_topic', Bool, queue_size=10)
        self.pub_scarlet = rospy.Publisher('scarlet_topic', Bool, queue_size=10)
        self.pub_circle_based_velocity = rospy.Publisher('mobile_base/commands/velocity', Twist)

        self.character_x = 0
        self.character_y = 0
        self.character_w = 0
        self.character_h = 0
        self.character_center = 0

        self.image_x = 0
        self.image_y = 0

        self.cv_image = np.zeros((480, 640, 3), np.uint8)

    def imageCallback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data)
        except CvBridgeError as e:
            print("Image conversion failed")
            print(e)
            pass

        self.image_x = self.cv_image.shape[1]
        self.image_y = self.cv_image.shape[0]

    def callbackMustard(data):
        print("callbackMustard " + str(data.data))

    def callbackPeacock(data):
        print("callbackPeacock " + str(data.data))

    def callbackPlum(data):
        print("callbackPlum " + str(data.data))

    def callbackScarlet(data):
        print("callbackScarlet " + str(data.data))

    def callbackRectangle(data):
        print("callbackRectangle " + str(data.data))


def main(args):
    rospy.init_node('character_output', anonymous=True)
    cI = CharacterOutput()
    print("Initializing character finder")
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


# Check if the node is executing in the main path
if __name__ == '__main__':
    main(sys.argv)
