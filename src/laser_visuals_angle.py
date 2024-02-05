#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from geometry_msgs.msg import Point, Quaternion 

def process_scan(data):
    # Number of scan points
    num_points = len(data.ranges)

    # Convert LaserScan points to Cartesian coordinates
    # Use linspace to ensure the angles array matches the number of distance measurements exactly
    angles = np.linspace(data.angle_min, data.angle_max, num_points)
    distances = np.array(data.ranges)

    x = distances * np.cos(angles)
    y = distances * np.sin(angles)

    # Filter out 'inf' and 'nan' values from x and y
    finite_indices = np.isfinite(x) & np.isfinite(y)
    x = x[finite_indices]
    y = y[finite_indices]

    # Fit a line using linear regression
    A = np.vstack([x, np.ones(len(x))]).T
    slope, intercept = np.linalg.lstsq(A, y, rcond=None)[0]

    # Calculate the angle of the wall
    # The angle of the wall is the arctan of the slope
    wall_angle = np.arctan(slope)
    
    # Publish the angle
    angle_pub.publish(Float32(wall_angle))

    # Visualize the line in RViz
    line = Marker()
    line.header.frame_id = 'laser_link'
    line.type = Marker.LINE_STRIP
    line.id = 1
    line.action = Marker.ADD
    line.scale.x = 0.05
    line.color.r = 1.0
    line.color.a = 1.0
    line.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0) 
    line.points.append(Point(min(x), min(x)*slope + intercept, 0))
    line.points.append(Point(max(x), max(x)*slope + intercept, 0))
    

     # Visualize the angle as text in RViz
    text = Marker()
    text.header.frame_id = 'base_link'
    text.type = Marker.TEXT_VIEW_FACING
    text.id = 2
    text.action = Marker.ADD
    text.scale.z = 0.4  # Text size
    text.color.r = 1.0
    text.color.g = 1.0
    text.color.b = 1.0
    text.color.a = 1.0
    text.pose.position = Point(0, 0, 1.5)  # Position the text above the robot
    text.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)  # Identity quaternion
    text.text = str(wall_angle)  # Display the angle in radians

    marker_pub.publish(line)
    marker_pub.publish(text)

if __name__ == '__main__':
    rospy.init_node('wall_angle_estimator')

    # Publishers
    angle_pub = rospy.Publisher('wall_angle', Float32, queue_size=10)
    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

    # Subscriber
    rospy.Subscriber('filtered_scan', LaserScan, process_scan)

    rospy.spin()
