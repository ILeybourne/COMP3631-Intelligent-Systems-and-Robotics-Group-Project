#!/usr/bin/env python
from __future__ import division
import cv2
import numpy as np
import rospy
import sys
import yaml
from os.path import expanduser


from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion

first_flag = 0
green_discovered = 0
red_discovered = 0
count_track = 0
second_flag = 0

class colourIdentifier():

    def __init__(self):
        self.pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        self.desired_velocity = Twist()

        self.green = 0
        self.red = 0

        sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.callback)
        self.bridge = CvBridge()

        try:
            opencv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as error:
            print(error)

        hsv_green_lower = np.array([65, 60, 60])
        hsv_green_upper = np.array([80, 255, 255])
        hsv_red_lower = np.array([0, 100, 100])
        hsv_red_upper = np.array([10, 255, 255])
        hsv_image = cv2.cvtColor(opencv_image, cv2.COLOR_BGR2HSV)

        mask2 = cv2.inRange(hsv_image, hsv_green_lower, hsv_green_upper)
        mask3 = cv2.inRange(hsv_image, hsv_red_lower, hsv_red_upper)
        apply_mask = cv2.bitwise_and(opencv_image, opencv_image, mask=mask2)
        apply_mask2 = cv2.bitwise_and(opencv_image, opencv_image, mask=mask3)

        contour_green = cv2.findContours(mask2.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
        length = len(contour_green)
        array_green = [0.0] * len(contour_green)
        for i in range(length):
            array_green[i] = cv2.contourArea(contour_green[i])

        contour_red = cv2.findContours(mask3.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
        length2 = len(contour_red)
        array_red = [0.0] * len(contour_red)
        for i in range(length2):
            array_red[i] = cv2.contourArea(contour_red[i])

        if array_green == []:
            self.desired_velocity.angular.z = 0.1
            self.desired_velocity.linear.x = 0.01

            for i in range(1):
                self.pub.publish(self.desired_velocity)
                self.rate.sleep()

        if array_green != []:
            for i in range(length):
                areacount  = array_green[i]

                if (areacount) > 5000:
                    self.green = 1
                else:
                    self.desired_velocity.angular.z = 0.1
                    self.desired_velocity.linear.x = 0.01

                    for i in range (1):
                        self.pub.publish(self.desired_velocity)
                        self.rate.sleep()

        if array_red != []:
            for i in range(length2):
                areacount2 = array_red[i]

                if (areacount2) > 5000:
                    self.red = 1

        if self.red == 1:
            self.desired_velocity.angular.z = 0
            self.desired_velocity.linear.x = 0.0
            self.pub.publish(self.desired_velocity)
            self.rate.sleep()

            red_discovered = 1
            self.red = 1

        if self.green ==1:
            self.desired_velocity.angular.z = 0
            self.desired_velocity.linear.x = 0.0
            self.pub.publish(self.desired_velocity)

            cv2.imwrite('green.png', opencv_image)

            for i in range(10):
                self.rate.sleep()

            green_discovered = 1
            self.green = 0

        cv2.namedWindow('Camera-Feed')
        cv2.imshow('Camera-Feed', opencv_image)
        cv2.waitKey(3)

class GoToPose():#x
    def __init__(self):
        self.goal_sent = False


        rospy.on_shutdown(self.shutdown)


        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Wait for the action server to come up")


        self.move_base.wait_for_server()

    def goto(self, pos, quat):#x

        self.goal_sent = True
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000), Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

        self.move_base.send_goal(goal)


        success = self.move_base.wait_for_result(rospy.Duration(60))

        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:

            result = True
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False
        return result

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)

def center_of_room_2():#x
    navigator = GoToPose()

    # Get home directory
    home = expanduser("~")
    # and open input_points.yaml file
    with open(home + "/catkin_ws/src/group_project/project/example/input_points.yaml", 'r') as stream:
        points = yaml.safe_load(stream)

    x = points['room2_centre_xy'][0]
    y = points['room2_centre_xy'][1]
    theta = 0
    position = {'x': x, 'y': y}
    quaternion = {'r1' : 0.000, 'r2': 0.000, 'r3' : np.sin(theta/2.0), 'r4' : np.cos(theta/2.0)}

    rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
    success = navigator.goto(position, quaternion)

    if success:
        rospy.loginfo("Hooray, reached the desired pose")
    else:
        rospy.loginfo("The base has failed to reach the desired pose")

    rospy.sleep(1)

def center_of_room_1():#x
    navigator = GoToPose()
    with open("../example/input_points.yaml", 'r') as stream:
        points = yaml.safe_load(stream)

    x = points['room1_centre_xy'][0]
    y = points['room1_centre_xy'][1]
    theta = 0
    position = {'x': x, 'y': y}
    quaternion = {'r1' : 0.000, 'r2': 0.000, 'r3' : np.sin(theta/2.0), 'r4' : np.cos(theta/2.0)}

    rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
    success = navigator.goto(position, quaternion)

    if success:
        rospy.loginfo("Hooray, reached the desired pose")
    else:
        rospy.loginfo("The base has failed to reach the desired pose")

    rospy.sleep(1)


def entrance_of_room_1():#x
    navigator = GoToPose()
    with open("../example/input_points.yaml", 'r') as stream:
        points = yaml.safe_load(stream)

    x = points['room1_entrance_xy'][0]
    y = points['room1_entrance_xy'][1]
    theta = 0
    position = {'x': x, 'y': y}
    quaternion = {'r1' : 0.000, 'r2': 0.000, 'r3' : np.sin(theta/2.0), 'r4' : np.cos(theta/2.0)}

    rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
    success = navigator.goto(position, quaternion)

    if success:
        rospy.loginfo("Hooray, reached the desired pose")
    else:
        rospy.loginfo("The base has failed to reach the desired pose")

    rospy.sleep(1)
def entrance_of_room_2():#x
    navigator = GoToPose()
    with open("../example/input_points.yaml", 'r') as stream:
        points = yaml.safe_load(stream)

    x = points['room2_entrance_xy'][0]
    y = points['room2_entrance_xy'][1]
    theta = 0
    position = {'x': x, 'y': y}
    quaternion = {'r1' : 0.000, 'r2': 0.000, 'r3' : np.sin(theta/2.0), 'r4' : np.cos(theta/2.0)}

    rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
    success = navigator.goto(position, quaternion)

    if success:
        rospy.loginfo("Hooray, reached the desired pose")
    else:
        rospy.loginfo("The base has failed to reach the desired pose")

    rospy.sleep(1)

def main(args):
    rospy.init_node('colourIdentifier', anonymous = True)
    global count_track
    global second_flag
    global red_discovered
    global first_flag
    global green_discovered

    while not rospy.is_shutdown():
        if first_flag == 0:
            entrance_of_room_1()
            first_flag = 1

        if second_flag == 0:
            cI = colourIdentifier()
            count_track +=1
            second_flag = 1

        if green_discovered == 1 and count_track == 1:
            center_of_room_1()

        if green_discovered == 1 and count_track == 2:
            center_of_room_2()

        if red_discovered == 1 and count_track == 2:
            entrance_of_room_2()
            count_track +=1

    cv2.destroyAllWindows()
if __name__ == '__main__':
    main(sys.argv)
