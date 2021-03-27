#!/usr/bin/env python
from __future__ import division
import numpy as np
import rospy
import sys
import yaml
from os.path import expanduser
import math

from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Bool
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion

first_flag = 0
green_discovered = 0
red_discovered = 0
count_track = 0
second_flag = 0


class Navigator():
    def __init__(self):
        self.pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        self.desired_velocity = Twist()

        self.red_circle_sub = rospy.Subscriber('red_circle_topic', Bool, self.callbackRedCircle)
        self.green_circle_sub = rospy.Subscriber('green_circle_topic', Bool, self.callbackGreenCirle)

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

    def center_of_room_1(self):  # x
        room_1_navigator = GoToPose()

        x = self.room1_centre_x
        y = self.room1_centre_y
        theta = 0

        position = {'x': x, 'y': y}
        quaternion = {'r1': 0.000, 'r2': 0.000, 'r3': np.sin(theta / 2.0), 'r4': np.cos(theta / 2.0)}

        rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
        success = room_1_navigator.goto(position, quaternion)

        if success:
            rospy.loginfo("Hooray, reached the desired pose")
        else:
            rospy.loginfo("The base has failed to reach the desired pose")
        rospy.sleep(1)

    def center_of_room_2(self):  # x
        room_2_navigator = GoToPose()

        x = self.room2_centre_x
        y = self.room2_centre_y
        theta = 0

        position = {'x': x, 'y': y}
        quaternion = {'r1': 0.000, 'r2': 0.000, 'r3': np.sin(theta / 2.0), 'r4': np.cos(theta / 2.0)}

        rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
        success = room_2_navigator.goto(position, quaternion)

        if success:
            rospy.loginfo("Hooray, reached the desired pose")
        else:
            rospy.loginfo("The base has failed to reach the desired pose")
        rospy.sleep(1)

    def entrance_of_room_1(self):  # x
        navigator = GoToPose()
        with open("../example/input_points.yaml", 'r') as stream:
            points = yaml.safe_load(stream)

        x = points['room1_entrance_xy'][0]
        y = points['room1_entrance_xy'][1]

        theta = 0
        if x - self.room1_centre_x != 0:
            theta = math.atan((y - self.room1_centre_y) / (x - self.room1_centre_x))
        else:
            theta = math.atan((y - self.room1_centre_y))
        theta -= math.pi

        position = {'x': x, 'y': y}
        quaternion = {'r1': 0.000, 'r2': 0.000, 'r3': np.sin(theta / 2.0), 'r4': np.cos(theta / 2.0)}

        rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
        success = navigator.goto(position, quaternion)

        if success:
            rospy.loginfo("Hooray, reached the desired pose")
        else:
            rospy.loginfo("The base has failed to reach the desired pose")

        rospy.sleep(1)

    def entrance_of_room_2(self):  # x
        room_2_navigator = GoToPose()
        with open("../example/input_points.yaml", 'r') as stream:
            points = yaml.safe_load(stream)

        x = points['room2_entrance_xy'][0]
        y = points['room2_entrance_xy'][1]

        theta = 0
        if x - self.room2_centre_x != 0:
            theta = math.atan((y - self.room2_centre_y) / (x - self.room2_centre_x))
        else:
            theta = math.atan((y - self.room2_centre_y))
        theta -= math.pi

        position = {'x': x, 'y': y}
        quaternion = {'r1': 0.000, 'r2': 0.000, 'r3': np.sin(theta / 2.0), 'r4': np.cos(theta / 2.0)}

        rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
        success = room_2_navigator.goto(position, quaternion)

        if success:
            rospy.loginfo("Hooray, reached the desired pose")
        else:
            rospy.loginfo("The base has failed to reach the desired pose")

        rospy.sleep(1)


class GoToPose():  # x
    def __init__(self):
        self.goal_sent = False

        rospy.on_shutdown(self.shutdown)

        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Wait for the action server to come up")

        self.move_base.wait_for_server()

    def goto(self, pos, quat):  # x

        self.goal_sent = True
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
                                     Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

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


def main(args):
    rospy.init_node('colourIdentifier', anonymous=True)
    global count_track
    global second_flag
    global red_discovered
    global first_flag
    global green_discovered

    navigator = Navigator()

    while not rospy.is_shutdown():
        if first_flag == 0:
            navigator.entrance_of_room_1()
            first_flag = 1

        if second_flag == 0:
            count_track += 1
            second_flag = 1

        if green_discovered == 1 and count_track == 1:
            navigator.center_of_room_1()

        if green_discovered == 1 and count_track == 2:
            navigator.center_of_room_2()

        if red_discovered == 1 and count_track == 2:
            navigator.entrance_of_room_2()
            count_track += 1


if __name__ == '__main__':
    main(sys.argv)
