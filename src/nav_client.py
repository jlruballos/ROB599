#!/usr/bin/env python3

import rospy
import actionlib
from rob599_hw1.msg import navAction, navGoal, navFeedback, navResult  

def feedback_cb(feedback):
    rospy.loginfo('Current Distance: %f' % feedback.current_distance)

if __name__ == '__main__':
    rospy.init_node('nav_client')

    # Initialize the action client with the 'NavAction' action type
    client = actionlib.SimpleActionClient('nav', navAction)

    rospy.loginfo("Waiting for action server to start...")
    client.wait_for_server()

    goal = navGoal(target_distance=2.0)

    rospy.loginfo("Sending goal...")
    client.send_goal(goal, feedback_cb=feedback_cb)

    rospy.loginfo("Waiting for result...")
    client.wait_for_result()

    result = client.get_result()

    rospy.loginfo("Action Completed. Result: success = %s, message = '%s'" % (result.success, result.message))
