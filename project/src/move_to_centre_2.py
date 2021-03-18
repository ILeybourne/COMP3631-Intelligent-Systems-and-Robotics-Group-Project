import rospy
import numpy as np
import std_msgs.msg
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
import geometry_msgs.msg
from nav_msgs.msg import Odometry
import nav_msgs.srv
import yaml

class GoToPose:
    def __init__(self):
        rospy.on_shutdown(self.shutdown)


        with open("../example/input_points.yaml","r") as stream:
            self.points = yaml.safe_load(stream) 

        odom_sub = rospy.Subscriber('/odom', Odometry, self.updatePos)

        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(5))

        self.path_tolerance = 0.1
        self.pos = None
    def updatePos(self,data):
        #print(data)
        self.pos = data.pose.pose


    def goToCentre(self):
        #sleep to ensure the pos variable is set by the subscriber
        rospy.sleep(1)


        # constant for frame id of movement
        frame_id = "map"

        #create pose stamp for current location
        current_poseStamp = geometry_msgs.msg.PoseStamped()
        current_poseStamp.header.stamp = rospy.Time.now()
        current_poseStamp.header.frame_id = frame_id
        current_poseStamp.pose = self.pos

        #create pose stamp for room 1 centre
        room1_poseStamp = geometry_msgs.msg.PoseStamped()
        room1_poseStamp.header.stamp = rospy.Time.now()
        room1_poseStamp.header.frame_id = frame_id
        room1_poseStamp.pose.position.x = self.points['room1_centre_xy'][0]
        room1_poseStamp.pose.position.y = self.points['room1_centre_xy'][1]
        room1_poseStamp.pose.orientation.w = 1 

        #create pose stamp for room 2 centre
        room2_poseStamp = geometry_msgs.msg.PoseStamped()
        room2_poseStamp.header.stamp = rospy.Time.now()
        room2_poseStamp.header.frame_id = frame_id
        room2_poseStamp.pose.position.x = self.points['room2_centre_xy'][0]
        room2_poseStamp.pose.position.y = self.points['room2_centre_xy'][1]
        room2_poseStamp.pose.orientation.w = 1

        #get the route plans from the current position to both room centres
        get_plan = rospy.ServiceProxy('move_base/make_plan', nav_msgs.srv.GetPlan)
        path_to_room1 = get_plan(current_poseStamp,room1_poseStamp,self.path_tolerance)
        path_to_room2 = get_plan(current_poseStamp, room2_poseStamp, self.path_tolerance)
        
        room1_distance = 0
        room2_distance = 0

        # calculate the route lengths for both route plans
        for i in range (len(path_to_room1.plan.poses)-1):
            a = path_to_room1.plan.poses[i].pose.position
            b = path_to_room1.plan.poses[i+1].pose.position

            room1_distance += np.sqrt(np.power((a.x - b.x),2) + np.power((a.y - b.y),2))

        for i in range (len(path_to_room2.plan.poses)-1):
            a = path_to_room2.plan.poses[i].pose.position
            b = path_to_room2.plan.poses[i+1].pose.position

            room2_distance += np.sqrt(np.power((a.x - b.x),2) + np.power((a.y - b.y),2))

        print("\n\n")
        print("distance to room 1: %d" % room1_distance)
        print("distance to room 2: %d" % room2_distance)
        print("\n")

        # move to the closest room centre
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = frame_id
        if room1_distance <= room2_distance:
            goal.target_pose.pose = room1_poseStamp.pose
            self.move_base.send_goal(goal)
            success = self.move_base.wait_for_result(rospy.Duration(60))
            print("now at room 1 centre")
        else:
            goal.target_pose.pose = room2_poseStamp.pose
            self.move_base.send_goal(goal)
            success = self.move_base.wait_for_result(rospy.Duration(60))
            print("now at room 2 centre")

        print(success)

        
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