import rospy
import numpy as np
import std_msgs.msg
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
import geometry_msgs.msg
from nav_msgs.msg import Odometry
import yaml

class GoToPose:
    def __init__(self):
        rospy.on_shutdown(self.shutdown)


        with open("../example/input_points.yaml","r") as stream:
            self.points = yaml.safe_load(stream) 

        odom_sub = rospy.Subscriber('/odom', Odometry, self.updatePos)

        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(5))

        self.pos = None
    def updatePos(self,data):
        print(data)
        self.pos = data.pose.pose


    def goToCentre(self):
        rospy.sleep(1)

        print("\n\n\n")

        current_poseStamp = geometry_msgs.msg.PoseStamped()
        current_poseStamp.header.stamp = rospy.Time.now()
        current_poseStamp.header.frame_id = "odom"
        current_poseStamp.pose = self.pos

        room1_poseStamp = geometry_msgs.msg.PoseStamped()
        room1_poseStamp.header.stamp = rospy.Time.now()
        room1_poseStamp.header.frame_id = "odom"
        room1_poseStamp.pose.position.x = self.points['room1_centre_xy'][0]
        room1_poseStamp.pose.position.y = self.points['room1_centre_xy'][1]

        room2_poseStamp = geometry_msgs.msg.PoseStamped()
        room2_poseStamp.header.stamp = rospy.Time.now()
        room2_poseStamp.header.frame_id = "odom"
        room2_poseStamp.pose.position.x = self.points['room2_centre_xy'][0]
        room2_poseStamp.pose.position.y = self.points['room2_centre_xy'][1]

        print(current_poseStamp)
        print("\n\n\n")
        print(room1_poseStamp)
        print("\n\n\n")
        print(room2_poseStamp)

    def shutdown(self):
        rospy.loginfo("Stop")
        rospy.sleep(1)


if __name__ == "__main__":
    try:
        rospy.init_node('nav_test', anonymous=True)
        navigation = GoToPose()

        navigation.goToCentre()

        rospy.sleep(1)
    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")