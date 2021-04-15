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
        self.in_green_room = True # Change to True to test code in isolation
        self.nav_completed = False
        self.rectangle_flag = False

        #subscribers to determine when to start execution
        self.green_room_flag_sub = rospy.Subscriber('inside_green_room', Bool, self.greenRoomFlagCallback)
        self.rectangle_detection_sub = rospy.Subscriber('rectangle_topic', Bool, self.rectangleFlagCallback)

        #Publishers
        self.movement_pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=10)

        self.rate = rospy.Rate(10) #10hz

        #simple movement to rotate anticlockwise around a point
        self.rotation_angle = 20 # change for more precision
        self.rotate = Twist()
        self.rotate.angular.z = self.rotation_angle*(self.PI/180)

        self.move_forward = Twist()
        self.move_forward.linear.x = 0.2

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
                seen = self.lookForRectange()
                if seen:
                    print("I can see It!!!")
                    status = self.moveTowardPortrait()
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
                    for i in range(10):
                        self.movement_pub.publish(self.rotate)
                        #sleep after issuing rotation, give chance for message and subsequent processing by other nodes
                        self.rate.sleep()
                    rospy.sleep(20)
                    print("rotated 20 degrees?")
                    rotation = rotation+self.rotation_angle

        
        if self.rectangle_flag == False:
            return False
        else:
            return True


    # Find the middle rotation for the portrait, then moves towards it 
    # makes the assumption portrait is in view of turtlebot already AND that rotating anti-clockwise will move it into frame
    # Return: 0 for when turtle is in correct position for the photo, 1 for successful movement
    def moveTowardPortrait(self):
        rotation = 0
        while(self.rectange_flag == True):
            for i in range(10):
                self.movement_pub.publish(self.rotate)
                self.rate.sleep()
            rospy.sleep(1)
            rotation = rotation + self.rotation_angle
        
        # find angle to focus the portrait
        middle = rotation/2
        
        # build same rotation message but in the clockwise direction
        rotation_msg = Twist()
        rotation_msg.angular.z = - self.rotation_angle*(self.PI/180)
        
        # handles rotations above the rotation angle, rotating the rotation angle x times
        if (middle > rotation_angle):
            while (middle > rotation_angle):
                for i in range(10):
                    self.movement_pub.publish(rotation_msg)
                    self.rate.sleep()
                middle = middle - self.rotation_angle

        # handles rotations under the rotation angle, if they are needed
        if (middle > 0):
            rotation_msg.angular.z = - middle*(self.PI/180)
            for i in range(10):
                self.movement_pub.publish(rotation_msg)
                self.rate.sleep()

        # Move Forward Code
        if (rotate < 30):
            return 0
        else:
            for i in range(10):
                self.movement_pub.publish(self.move_forward)
                self.rate.sleep()
            rospy.sleep(1)
            return 1

        

def main (args):
    rospy.init_node('green_room_navigator', anonymous=True)
    gNav = greenNavigation()
    try:
        gNav.startNavigation()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main(sys.argv)