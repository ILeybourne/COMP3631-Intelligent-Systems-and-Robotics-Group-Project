#!/usr/bin/env python
import rospy
import sys

from geometry_msgs.msg import Twist, Vector3, Pose, Point, Quaternion
from std_msgs.msg import Bool


class greenNavigation():
    def __init__(self):
        #internal flag for handling execution
        self.in_green_room = False
        self.nav_completed = False
        self.rectangle_flag = False
        #publisher to determine when to start execution
        self.green_room_flag_sub = rospy.Subscriber('inside_green_room', Bool, self.greenRoomFlagCallback)
        self.rectangle_detection_sub = rospy.Subscriber('rectangle_topic', Bool, self.rectangleFlagCallback)

    # inside_green_room subscriber callback
    def greenRoomFlagCallback(self, data):
        self.in_green_room = data.data

    # rectangle_tobpic subscriber callback
    def rectangleFlagCallback(self, data):
        self.rectangle_flag = data.data

    # main navigation function
    def startNavigation(self):
        while(not rospy.is_shutdown()):
            if (self.in_green_room == True):
                print("begin action")
                

            else:
                #debugging print for main function idling
                print("waiting...")


def main (args):
    rospy.init_node('green_room_navigator', anonymous=True)
    gNav = greenNavigation()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main(sys.args)