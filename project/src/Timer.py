#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Bool
import sys


class Timer():
    def __init__(self):
        # Get system start time until clock actually starts
        self.start_time = rospy.get_time()
        while self.start_time == 0.0:
            self.start_time = rospy.get_time()

        self.end_time = rospy.get_time()
        self.ended = False
        self.sub_shutdown = rospy.Subscriber('shutdown_topic', Bool, self.printFinishedTimer)
        self.pub_timer = rospy.Publisher('timer_topic', String, queue_size=10)

    def printFinishedTimer(self, data):
        # If the program has finished running
        if data.data:
            # Calculate end time once
            if not self.ended:
                self.end_time = rospy.get_time()

            # Get total duration
            total_time = self.end_time - self.start_time

            # Publish duration as seconds or minutes
            if total_time < 60:
                self.pub_timer.publish("Completed in " + str(total_time) + " seconds")
            else:
                mins = int(total_time / 60)
                secs = total_time % 60
                self.pub_timer.publish("Completed in " + str(mins) + ":" + str(secs))
            self.ended = True


def main(args):
    rospy.init_node('timer', anonymous=True)
    t = Timer()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


# Check if the node is executing in the main path
if __name__ == '__main__':
    main(sys.argv)

# Testing: Timer accuratley times from start to completion of task
# Ensure all worlds are completed within the 5 minute time allocation provided
# project.world: Test Successful. Task completed in 3:43:765
# project_1.world: Test Successful Task completed in 2:23:876
# project_2.world: Test Successful Task completed within 5 minutes
# project_3.world: Test Successful Task completed in 2:01:546
# project_4.world: Test Successful
# project_5.world: Test Successful
# project_6.world: Test UNSUCESSFUL Task completed in 5:43:765
# project_7.world: World only used for testing cluedo characters
# project_9.world: Test Successful Task completed within 5 minutes
# project_10.world: Test Successful Task completed within 5 minutes
# project_11.world: Test Successful
# project_12.world: Test Successful
