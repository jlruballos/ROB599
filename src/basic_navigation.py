#!/usr/bin/env python3

import rospy
import actionlib
from rob599_hw1.srv import SetStoppingDistance, SetStoppingDistanceResponse
from rob599_hw1.msg import navAction, navFeedback,navResult # Import the custom action message
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

stopping_distance = 6.0  # Default stopping distance
action_active = False
stopped = False


def callback(msg):
    global stopping_distance, action_active, stopped
    
    min_range = min(msg.ranges)
    twist_msg = Twist()
    scaling_factor = 0.25

    if min_range <= round(stopping_distance,1):
        twist_msg.linear.x = 0
        stopped = True
        rospy.loginfo("I'm Stopped")
    else:
        stopped = False
        vel = scaling_factor * min_range
        twist_msg.linear.x = vel
        rospy.loginfo("I'm moving! speed {0}".format(vel))

    publisher.publish(twist_msg)

def set_stopping_distance(req):
    global stopping_distance, action_active
    
    if action_active: # If the action is active, don't allow the stopping distance to be changed
        return SetStoppingDistanceResponse(False, "Action is active. Cannot set stopping distance")

    if req.distance < 0:
        return SetStoppingDistanceResponse(False, "Negative distance not allowed")
    stopping_distance = req.distance
    return SetStoppingDistanceResponse(True, "Stopping distance updated successfully")

def nav_to_goal(goal):
    global stopping_distance, action_active, stopped
    action_active = True
    feedback = navFeedback()
    result = navResult()

    

    if action_active:
        stopping_distance = goal.target_distance  # Set stopping distance from goal
        rospy.loginfo("Navigating to goal at distance {0}".format(stopping_distance))
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            min_range = rospy.wait_for_message('filtered_scan', LaserScan).ranges[0]
            feedback.current_distance = min_range
            action_server.publish_feedback(feedback)

            #if min_range <= round(stopping_distance,1):
            if stopped:
                result.success = True
                result.message = "Reached stopping distance"
                rospy.Rate(1)
                action_server.set_succeeded(result)
                break
                

    action_active = False

if __name__ == '__main__':
    rospy.init_node('basic_navigation')
    subscriber = rospy.Subscriber('filtered_scan', LaserScan, callback)
    publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    service = rospy.Service('set_stopping_distance', SetStoppingDistance, set_stopping_distance)
    action_server = actionlib.SimpleActionServer('nav', navAction, nav_to_goal, False)
    action_server.start()
    rospy.loginfo('Navigation action server started')
    rospy.spin()