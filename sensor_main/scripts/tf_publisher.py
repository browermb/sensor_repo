#!/usr/bin/env python

import rospy

class TFPublisher(object):

    def __init__(self):
        rospy.init_node('tf_publisher')
        rospy.Subscriber('/ndt_pose', PoseStamped, self.ndt_pose_callback)
        #some publisher

    def ndt_post_callback(self):
        pass

if __name__ == "__main__":
    TFPublisher()

