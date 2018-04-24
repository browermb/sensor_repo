#!/usr/bin/env python 
import rospy
import math
import numpy as np
import requests

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray

class rplidar_node(object):

    def __init__(self):
        rospy.init_node('rplidar_node')

        self.box_width = 0.95
        self.min_dist = 1

        print "done loop"
        
        #publish to detect topic
        self.rp_sub = rospy.Subscriber('/rp/scan', LaserScan, self.scan_callback)
        self.rp_pub = rospy.Publisher('/rp_distance', Float32, queue_size = 10)
        self.points_pub = rospy.Publisher('/lidar_points', PoseArray, queue_size=10)

        rospy.sleep(1)
        r = rospy.Rate(50)

        ''' ================================================== NEW =========================='''
        ''' SEND STATUS IN THE CALLBACK OF EACH UPDATE! '''


        while not rospy.is_shutdown():
            r.sleep()

    def scan_callback(self, scan):
        theta_beam = np.arange(scan.angle_min, scan.angle_max, scan.angle_increment)
        #print theta_beam

        scans = np.array(scan.ranges)
        scans = scans[1:]
        
        #X
        ranges = scans * np.cos(theta_beam)
        ranges = ranges[180:355]

        #Y
        widths = scans * np.sin(theta_beam)
        widths = widths[180:355]

        widths = widths[np.absolute(ranges) <= (self.box_width/2.0)]
        ranges = ranges[np.absolute(ranges) <= (self.box_width/2.0)]
        

        ranges = ranges.reshape((ranges.shape[0], 1))
        widths = widths.reshape((widths.shape[0], 1))

        marker_array_msg = PoseArray()
        marker_array_msg.header.stamp = rospy.Time.now()
        marker_array_msg.header.frame_id = 'laser'
        marker_list = []

        points = np.append(ranges, widths, axis=1)
        points = points[np.logical_not(np.isinf(np.max(points, axis=1)))]
        print points
        points = points.tolist()
        print type(points)
        for point in points:
            my_marker = Pose()
            
            my_marker.position.x = point[0]
            my_marker.position.y = point[1]

            marker_list.append(my_marker)

        marker_array_msg.poses = marker_list
        self.points_pub.publish(marker_array_msg)

        valid_ranges = widths[np.absolute(ranges) <= (self.box_width/2.0)]

        #valid_ranges = valid_ranges[np.logical_not(np.isinf(valid_ranges))]
        #print valid_ranges.shape
        if (valid_ranges.shape[0] != 0):
            min_range = min(valid_ranges)
        else:
            min_range = 0
        #print min_range
        msg = Float32()
        msg.data = min_range
        self.rp_pub.publish(msg)

if __name__ == "__main__":
    try:
	    rplidar_node()
    except rospy.ROSInterruptException:
        pass
