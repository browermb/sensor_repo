#!/usr/bin/env python

import rospy

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import TwistWithCovariance
from nav_msgs.msg import Odometry

class Pose_Vel_Node(object):

    def __init__(self):
        rospy.init_node('pos_vel_node')

        # Info from previous time-step
        self.prev_t = 0
        self.prev_x = 0
        self.prev_y = 0

        # Pose
        self.my_pose = Pose()

        self.pose_vel_pub = rospy.Publisher('/pose_and_speed', Odometry, queue_size=10)
        rospy.Subscriber('/zed/odom', Odometry, self.pose_callback)
        rospy.Subscriber('/ndt_pose', PoseStamped, self.ndt_callback)

        rospy.spin()

    def ndt_callback(self, ndt_msg):
        self.my_pose = ndt_msg.pose

    def pose_callback(self, pose_msg):
        pose_vel_msg = Odometry()
        pose_vel_msg.header.stamp = pose_msg.header.stamp
        pose_vel_msg.header.frame_id = 'map'
        pose_vel_msg.pose.pose = self.my_pose

        t = pose_msg.header.stamp.to_sec()
        x = pose_msg.pose.pose.position.x
        y = pose_msg.pose.pose.position.y

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
