import rospy
import numpy as np
import yaml
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from tf import TransformListener 

class GoToPose:
    def __init__(self):
        rospy.on_shutdown(self.shutdown)


        with open("../example/input_points.yaml","r") as stream:
            self.points = yaml.safe_load(stream) 

        self.listener = TransformListener()

        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(5))

    def goToCentre(self):
        time = rospy.Time(0)
        position, quaternion = self.listener.lookupTransform('/map','/base_link',time)
        print(position)

    def shutdown(self):
        rospy.loginfo("Stop")
        rospy.sleep(1)

if __name__ =='__main__':
    try:
        rospy.init_node('nav_test', anonymous=True)
        navigation = GoToPose()

        navigation.goToCentre()

        rospy.sleep(1)
    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")
