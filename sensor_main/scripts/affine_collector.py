#!/usr/bin/env python

import rospy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

from sensor_msgs.msg import NavSatFix

import message_filters
from message_filters import ApproximateTimeSychronizer

class Pose_Vel_Node(object):

    def __init__(self):
        rospy.init_node('affine_node')

        pose_sub = rospy.Subscriber('/ndt_pose', PoseStamped)
        gps_sub = rospy.Subscriber 

        rospy.spin()

    def pose_callback(self, pose_msg):
        pose_vel_msg = Odometry()
        pose_vel_msg.header.stamp = pose_msg.header.stamp
        pose_vel_msg.header.frame_id = 'map'
        pose_vel_msg.pose.pose = pose_msg.pose

        t = pose_msg.header.stamp.to_sec()
        x = pose_msg.pose.position.x
        y = pose_msg.pose.position.y

        # Determine the velocity
        vel_x = (x - self.prev_x) / (t - self.prev_t)
        vel_y = (y - self.prev_y) / (t - self.prev_t)

        pose_vel_msg.twist.twist.linear.x = vel_x
        pose_vel_msg.twist.twist.linear.y = vel_y

        # Store old values
        self.prev_t = t
        self.prev_x = x
        self.prev_y = y

        self.pose_vel_pub.publish(pose_vel_msg)

if __name__ == "__main__":
    Pose_Vel_Node()

        


        

