#!/usr/bin/env python

import rospy

def handle_ndt_pose(self):
    br = tf.TransformBroadcaster()
    """
    br.sendTransform((msg.x, msg.y, 0),
  12                      tf.transformations.quaternion_from_euler(0, 0, msg.theta),
  13                      rospy.Time.now(),
  14                      turtlename,
  15                      "world")"""

if __name__ == "__main__":
    rospy.init_node('tf_publisher')
    rospy.Subscriber('/ndt_pose', PoseStamped, self.handle_ndt_pose)
    rospy.spin()

