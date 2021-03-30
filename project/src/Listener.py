#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool, Int32MultiArray


def callbackRed(data):
    print("Red flag is " + str(data.data))


def callbackGreen(data):
    print("Green flag is " + str(data.data))


def callbackMustard(data):
    print("callbackMustard " + str(data.data))


def callbackPeacock(data):
    print("callbackPeacock " + str(data.data))


def callbackPlum(data):
    print("callbackPlum " + str(data.data))


def callbackScarlet(data):
    print("callbackScarlet " + str(data.data))


def callbackRectangle(data):
    print("callbackRectangle " + str(data.data))


def callbackRectangleInBounds(data):
    print("callbackRectangleInBounds " + str(data.data))


def callbackRectangleInts(data):
    print("callbackRectangleInts " + str(data.data))


def callbackShutdown(data):
    print("callbackShutdown " + str(data.data[0]) + " " + str(data.data[1]) + " " + str(data.data[2]) + " " + str(
        data.data[3]))


def listener():
    rospy.init_node('listener', anonymous=True)
    sub = rospy.Subscriber('red_circle_topic', Bool, callbackRed)
    sub = rospy.Subscriber('green_circle_topic', Bool, callbackGreen)
    sub = rospy.Subscriber('mustard_topic', Bool, callbackMustard)
    sub = rospy.Subscriber('peacock_topic', Bool, callbackPeacock)
    sub = rospy.Subscriber('plum_topic', Bool, callbackPlum)
    sub = rospy.Subscriber('scarlet_topic', Bool, callbackScarlet)
    sub = rospy.Subscriber('rectangle_topic', Bool, callbackRectangle)
    sub = rospy.Subscriber('rectangle_in_bounds_topic', Bool, callbackRectangleInBounds)
    sub = rospy.Subscriber('rectangle_ints_topic', Int32MultiArray, callbackRectangleInts)
    sub = rospy.Subscriber('shutdown_topic', Bool, callbackShutdown)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
