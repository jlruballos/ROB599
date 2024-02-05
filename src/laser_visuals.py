#!/usr/bin/env python3

import rospy
import sys
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan
import math
from geometry_msgs.msg import Point, Quaternion

def process_scan(data):
    # Find the shortest distance in the LaserScan data and its index
    shortest_distance = min(data.ranges)
    shortest_index = data.ranges.index(shortest_distance)

    # Calculate the angle corresponding to the shortest distance
    angle = data.angle_min + shortest_index * data.angle_increment

    # Calculate the x, y coordinates of the shortest contact point
    x = shortest_distance * math.cos(angle)
    y = shortest_distance * math.sin(angle)

    # Update the markers with the new data
    update_markers(x, y, shortest_distance)

def update_markers(x, y, distance):
    # Point marker for the shortest contact
    point = Marker()
    point.header.frame_id = 'laser_link'
    point.type = point.SPHERE
    point.id = 3
    point.action = point.ADD
    point.scale.x = 0.1
    point.scale.y = 0.1
    point.scale.z = 0.1
    point.color.r = 0.0
    point.color.g = 1.0
    point.color.b = 0.0
    point.color.a = 1.0
    point.pose.position = Point(x, y, 0)
    point.pose.orientation = Quaternion(0, 0, 0, 1) 

    # Line marker from the laser to the contact point
    line = Marker()
    line.header.frame_id = 'laser_link'
    line.type = line.LINE_STRIP
    line.id = 4
    line.action = line.ADD
    line.scale.x = 0.05
    line.color.r = 1.0
    line.color.g = 1.0
    line.color.b = 0.0
    line.color.a = 1.0
    line.points.append(Point(0, 0, 0))
    line.points.append(Point(x, y, 0))
    line.pose.orientation = Quaternion(0, 0, 0, 1) 

    # Text marker for displaying the distance
    text = Marker()
    text.header.frame_id = 'laser_link'
    text.type = text.TEXT_VIEW_FACING
    text.id = 5
    text.action = text.ADD
    text.scale.z = 0.2
    text.color.r = 1.0
    text.color.g = 1.0
    text.color.b = 1.0
    text.color.a = 1.0
    text.pose.position = Point(x, y, 0.2)  
    text.pose.orientation = Quaternion(0, 0, 0, 1)  
    text.text = str(round(distance, 2))

    # Publish the markers
    publisher.publish(point)
    publisher.publish(line)
    publisher.publish(text)

if __name__ == '__main__':
    rospy.init_node('laser_distance_visualizer', argv=sys.argv)

    # Subscribe to the LaserScan topic
    rospy.Subscriber('filtered_scan', LaserScan, process_scan)

    # Set up a publisher for the markers
    publisher = rospy.Publisher('visualization_marker', Marker, queue_size=10)

    # Spin to keep the script for exiting
    rospy.spin()
