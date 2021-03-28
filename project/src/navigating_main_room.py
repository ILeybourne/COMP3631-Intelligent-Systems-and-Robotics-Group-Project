#!/usr/bin/env python
from __future__ import division
import cv2
import numpy as np
import rospy
import sys
import yaml
from os.path import expanduser
import math
from std_msgs.msg import Bool

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion


green_discovered = False
green_discovered_at_least_once = False
red_discovered = False
moving = False

class Navigator():
    def __init__(self):
        self.red_circle_sub = rospy.Subscriber('red_circle_topic', Bool, self.callbackRedCircle)
        self.green_circle_sub = rospy.Subscriber('green_circle_topic', Bool, self.callbackGreenCircle)
        self.image_sub = rospy.Subscriber('image_topic', Image, self.imageCallback)
        self.moving_pub = rospy.Publisher('turtle_bot_main_room_moving_topic', Bool, queue_size=10)

        # Get home directory
        home = expanduser("~")
        # and open input_points.yaml file
        with open(home + "/catkin_ws/src/group_project/project/example/input_points.yaml", 'r') as stream:
            points = yaml.safe_load(stream)

        # Use file data to set our variables
        self.room1_entrance_x = points['room1_entrance_xy'][0]
        self.room1_entrance_y = points['room1_entrance_xy'][1]
        self.room2_entrance_x = points['room2_entrance_xy'][0]
        self.room2_entrance_y = points['room2_entrance_xy'][1]

        self.room1_centre_x = points['room1_centre_xy'][0]
        self.room1_centre_y = points['room1_centre_xy'][1]
        self.room2_centre_x = points['room2_centre_xy'][0]
        self.room2_centre_y = points['room2_centre_xy'][1]

        self.bridge = CvBridge()
        self.cv_image = np.zeros((480, 640, 3), np.uint8)

    def imageCallback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data)
        except CvBridgeError as e:
            print("Image conversion failed")
            print(e)
            pass

    def callbackRedCircle(self, data):
        global red_discovered
        global moving
        if not moving:
            red_discovered = data.data
        # For Debugging
        # if not moving:
        #     print("red found " + str(data.data))

    def callbackGreenCircle(self, data):
        global green_discovered
        global green_discovered_at_least_once
        global moving
        if green_discovered_at_least_once == False and data.data:
            cv2.imwrite('green_circle.png', self.cv_image)
            green_discovered_at_least_once = True
        if not moving:
            green_discovered = data.data
        # For Debugging
        # if not moving:
        #     print("green found " + str(data.data))

    def center_of_room_1(self):  # x
        room_1_navigator = GoToPose()

        x = self.room1_centre_x
        y = self.room1_centre_y
        theta = 0

        position = {'x': x, 'y': y}
        quaternion = {'r1': 0.000, 'r2': 0.000, 'r3': np.sin(theta / 2.0), 'r4': np.cos(theta / 2.0)}

        rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
        self.moving_pub.publish(True)
        success = room_1_navigator.goto(position, quaternion)

        if success:
            rospy.loginfo("Hooray, reached the desired pose")
        else:
            rospy.loginfo("The base has failed to reach the desired pose")
        self.moving_pub.publish(False)
        rospy.sleep(1)

    def center_of_room_2(self):  # x
        room_2_navigator = GoToPose()

        x = self.room2_centre_x
        y = self.room2_centre_y
        theta = 0

        position = {'x': x, 'y': y}
        quaternion = {'r1': 0.000, 'r2': 0.000, 'r3': np.sin(theta / 2.0), 'r4': np.cos(theta / 2.0)}

        rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
        self.moving_pub.publish(True)
        success = room_2_navigator.goto(position, quaternion)

        if success:
            rospy.loginfo("Hooray, reached the desired pose")
        else:
            rospy.loginfo("The base has failed to reach the desired pose")
        self.moving_pub.publish(False)
        rospy.sleep(1)

    def entrance_of_room_1(self):  # x
        navigator = GoToPose()

        x = self.room1_entrance_x
        y = self.room1_entrance_y

        theta = 0
        if x - self.room1_centre_x != 0:
            theta = math.atan((y - self.room1_centre_y) / (x - self.room1_centre_x))
        else:
            theta = math.atan((y - self.room1_centre_y))
        theta -= math.pi

        position = {'x': x, 'y': y}
        quaternion = {'r1': 0.000, 'r2': 0.000, 'r3': np.sin(theta / 2.0), 'r4': np.cos(theta / 2.0)}

        rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
        self.moving_pub.publish(True)
        success = navigator.goto(position, quaternion)

        if success:
            rospy.loginfo("Hooray, reached the desired pose")
        else:
            rospy.loginfo("The base has failed to reach the desired pose")

        self.moving_pub.publish(False)
        rospy.sleep(1)

    def entrance_of_room_2(self):  # x
        room_2_navigator = GoToPose()

        x = self.room2_entrance_x
        y = self.room2_entrance_y

        theta = 0
        if x - self.room2_centre_x != 0:
            theta = math.atan((y - self.room2_centre_y) / (x - self.room2_centre_x))
        else:
            theta = math.atan((y - self.room2_centre_y))
        theta -= math.pi

        position = {'x': x, 'y': y}
        quaternion = {'r1': 0.000, 'r2': 0.000, 'r3': np.sin(theta / 2.0), 'r4': np.cos(theta / 2.0)}

        rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
        self.moving_pub.publish(True)
        success = room_2_navigator.goto(position, quaternion)

        if success:
            rospy.loginfo("Hooray, reached the desired pose")
        else:
            rospy.loginfo("The base has failed to reach the desired pose")

        self.moving_pub.publish(False)
        rospy.sleep(1)


class GoToPose():  # x
    def __init__(self):
        self.goal_sent = False
        rospy.on_shutdown(self.shutdown)
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Wait for the action server to come up")
        self.move_base.wait_for_server()

    def goto(self, pos, quat):  # x
        global moving
        self.goal_sent = True
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
                                     Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))
        moving = True
        self.move_base.send_goal(goal)

        success = self.move_base.wait_for_result(rospy.Duration(60))

        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            result = True
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False

        moving = False

        return result

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)


def main(args):
    rospy.init_node('colourIdentifier', anonymous=True)
    count_track = 0
    global red_discovered
    first_flag = False
    second_flag = False
    global green_discovered
    global moving
    navigator = Navigator()
    in_room = False

    while not rospy.is_shutdown() and not in_room:
        if first_flag == False:
            print("sending to room 1")
            navigator.entrance_of_room_1()
            count_track += 1
            first_flag = True

        if green_discovered == True and count_track == 1:
            print("moving to to room 1 center")
            navigator.center_of_room_1()
            in_room = True

        if second_flag == False and green_discovered == False:
            print("sending to to room 2")
            navigator.entrance_of_room_2()
            in_room = True
            count_track += 1
            second_flag = True

        if green_discovered == True and count_track == 2:
            navigator.center_of_room_2()
            in_room = True
        else:
            navigator.center_of_room_1()
            in_room = True

if __name__ == '__main__':
    main(sys.argv)
