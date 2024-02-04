#!/usr/bin/env python3

# Import ROS Python API and sys
import rospy
import sys

# Import the LaserScan message definition
from sensor_msgs.msg import LaserScan

# Function to process and filter LaserScan messages
def callback(msg):
    # Create a new LaserScan message
    filtered_scan = LaserScan()

    # Copy the header and other necessary fields
    filtered_scan.header = msg.header
    filtered_scan.angle_min = msg.angle_min
    filtered_scan.angle_max = msg.angle_max
    filtered_scan.angle_increment = msg.angle_increment
    filtered_scan.time_increment = msg.time_increment
    filtered_scan.scan_time = msg.scan_time
    filtered_scan.range_min = msg.range_min
    filtered_scan.range_max = msg.range_max

    # Calculate the index range for 1m wide in front of the robot
    # This assumes the front of the robot is centered in the LaserScan field of view
    front_start_index = int((-0.5 - msg.angle_min)/ msg.angle_increment)
    front_end_index = int((0.5 - msg.angle_min)/ msg.angle_increment)

    # Filter the ranges to only include points directly in front of the robot
    filtered_scan.ranges = msg.ranges[front_start_index:front_end_index]
	
    filtered_scan.header.stamp = rospy.Time.now()
    filtered_scan.angle_min = -0.5
    filtered_scan.angle_max = 0.5

    # Publish the filtered LaserScan message
    publisher.publish(filtered_scan)

    # Log information
    rospy.loginfo('Published filtered LaserScan')

if __name__ == '__main__':
    # Initialize the node
    rospy.init_node('laser_filter', argv=sys.argv)

    # Set up a subscriber for LaserScan messages
    subscriber = rospy.Subscriber('base_scan', LaserScan, callback)

    # Set up a publisher for the filtered LaserScan messages
    publisher = rospy.Publisher('filtered_scan', LaserScan, queue_size=10)

    # Give control to ROS
    rospy.spin()
