#!/usr/bin/env python
import rospy
import sys

from geometry_msgs.msg import Twist, Vector3, Pose, Point, Quaternion
from std_msgs.msg import Bool

PI = 3.1415926535897

class greenNavigation():
    def __init__(self):
        #internal flag for handling execution
        self.in_green_room = False
        self.nav_completed = False
        self.rectangle_flag = False

        #subscribers to determine when to start execution
        self.green_room_flag_sub = rospy.Subscriber('inside_green_room', Bool, self.greenRoomFlagCallback)
        self.rectangle_detection_sub = rospy.Subscriber('rectangle_topic', Bool, self.rectangleFlagCallback)

        #Publishers
        self.movement_pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=10)

        #simple movement to rotate around scene
        self.rotate = Twist()
        self.rotate.angular.z = 10*(PI/180)

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
                rotation = 0
                
                
            else:
                #debugging print for main function idling
                print("waiting...")
    



    
    # rotates around current pose to look for character in the scene
    # returns True if found, False otherwise
    def lookForRectange(self):
        rotation = 0
        while (rotation < 360 and self.rectangle_flag == False):
                    self.movement_pub.publish(self.rotate)
                    #sleep after issuing rotation, give chance for message and subsequent processing by other nodes
                    rospy.sleep(1)
        
        if self.rectangle_flag == False:
            return False
        else:
            return True


def main (args):
    rospy.init_node('green_room_navigator', anonymous=True)
    gNav = greenNavigation()
    try:
        gNav.greenNavigation()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main(sys.args)