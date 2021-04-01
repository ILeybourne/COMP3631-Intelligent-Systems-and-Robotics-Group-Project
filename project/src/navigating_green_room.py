#!/usr/bin/env python
import rospy
import sys

from geometry_msgs.msg import Twist, Vector3, Pose, Point, Quaternion
from std_msgs.msg import Bool

PI = 3.1415926535897

# notice about angle handling:
#   All angles shown in numbers are in degrees
#   All rotations performed must be converted to radians using angle*(PI/180)

class greenNavigation():
    def __init__(self):
        # pi constant for conversions
        self.PI = 3.1415926535897

        #internal flag for handling execution
        self.in_green_room = False
        self.nav_completed = False
        self.rectangle_flag = False

        #subscribers to determine when to start execution
        self.green_room_flag_sub = rospy.Subscriber('inside_green_room', Bool, self.greenRoomFlagCallback)
        self.rectangle_detection_sub = rospy.Subscriber('rectangle_topic', Bool, self.rectangleFlagCallback)

        #Publishers
        self.movement_pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=10)

        #simple movement to rotate anticlockwise around a point
        self.rotation_angle = 10 # change for more precision
        self.rotate = Twist()
        self.rotate.angular.z = self.rotation_angle*(self.PI/180)

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
                # check for the portrait from initial position
                seen = lookForRectange()
                if seen:
                    print("I can see It!!!")
                else:
                    print("I cant see it :(")
                
            else:
                #debugging print for main function idling
                print("waiting...")
    




    # rotates around current pose to look for character in the scene
    # CATION: does NOT reset to rotation from before function, exits with new rotation 
    # Return: True if portrait is found, False otherwise
    def lookForRectange(self):
        rotation = 0
        while (rotation < 360 and self.rectangle_flag == False):
                    self.movement_pub.publish(self.rotate)
                    #sleep after issuing rotation, give chance for message and subsequent processing by other nodes
                    rospy.sleep(1)
                    rotation = rotation+rotation_angle
        
        if self.rectangle_flag == False:
            return False
        else:
            return True


    # Find the middle rotation for the portrait, then moves towards it 
    # makes the assumption portrait is in view of turtlebot already AND that rotating anti-clockwise will move it into frame
    # Return: None
    def moveTowardPortrait(self):
        rotation = 0
        while(self.rectange_flag == True):
            self.movement_pub.publish(self.rotate)
            rospy.sleep(1)
            rotation = rotation + rotation_angle
        
        # find angle to focus the portrait
        middle = rotation/2
        
        # build same rotation message but in the clockwise direction
        rotation_msg = Twist()
        rotation_msg.angular.z = - self.rotation_angle*(self.PI/180)
        
        # handles rotations above the rotation angle, rotating the rotation angle x times
        if (middle > rotation_angle):
            while (middle > rotation_angle):
                self.movement_pub.publish(rotation_msg)
                rospy.sleep(1)
                middle = middle - rotation_angle

        # handles rotations under the rotation angle, if they are needed
        if (middle > 0):
            rotation_msg.angular.z = - middle*(self.PI/180)
            self.movement_pub.publish(rotation_msg)
            rospy.sleep(1)

        # MOVE FORWARDS CODE - UNFINISHED

def main (args):
    rospy.init_node('green_room_navigator', anonymous=True)
    gNav = greenNavigation()
    try:
        gNav.greenNavigation()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main(sys.args)