#!/usr/bin/env python
import rospy
import sys

from geometry_msgs.msg import Twist, Vector3, Pose, Point, Quaternion
from std_msgs.msg import Bool


class greenNavigation():
    def __init__(self):
        # needs a publisher to determine whether the find is in green room 
        self.in_green_room = False
        self.green_room_flag_sub = rospy.Subscriber('inside_green_room', Bool, self.greenRoomFlagCallback)

    def greenRoomFlagCallback(data):
        self.in_green_room = data.data

    

def main (args):
    rospy.init_node('green_room_navigator', anonymous=True)
    gNav = greenNavigation()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main(sys.args)