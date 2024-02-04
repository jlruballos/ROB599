#!/usr/bin/env python3


# Basic topic subscriber example.
#
# subscriber.py
#
# Bill Smart
#
# This example shows the basic code for subscribing to a topic.


# Import ROS Python basic API and sys
import rospy
import sys

# We're going to subscribe to 64-bit integers, so we need to import the defintion
# for them.
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# This is a function that is called whenever a new message is received.  The
# message is passed to the function as a parameter.
def callback(msg):
	min_range = min(msg.ranges)

	twist_msg = Twist()

	scaling_factor =0.25

	if min_range <= 0.9:
		twist_msg.linear.x = 0
		rospy.loginfo('Im Stopped')
		#publisher.publish(twist_msg)
	else:
		vel = scaling_factor * min_range
		#twist_msg.linear.x = min(vel, 1.0)
		twist_msg.linear.x = vel
		rospy.loginfo('Im moving!')
		rospy.loginfo('speed {0}'.format(vel))
		#publisher.publish(twist_msg)
	
	publisher.publish(twist_msg)

	rate = rospy.Rate(1)
	"""
	Callback function to deal with incoming messages.
	:param msg: The message.
	"""
	# The value of the integer is stored in the data attribute of the message.
	rospy.loginfo('Got {0}'.format(min_range))


if __name__ == '__main__':
	# Initialize the node.
	rospy.init_node('basic_navigation', argv=sys.argv)

	# Set up a subscriber.  We're going to subscribe to the topic "counter",
	# looking for Int64 messages.  When a message comes in, ROS is going to pass
	# it to the function "callback" automatically.
	subscriber = rospy.Subscriber('filtered_scan', LaserScan, callback)
	publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
	# Give control to ROS.  This will allow the callback to be called whenever new
	# messages come in.  If we don't put this line in, then the node will not work,
	# and ROS will not process any messages.
	rospy.spin()
