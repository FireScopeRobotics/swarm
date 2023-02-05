#!/usr/bin/env python  
import roslib
import rospy

import tf
import turtlesim.msg

def handle_turtle_pose():
    br = tf.TransformBroadcaster()
    br.sendTransform((0, 0, 0.1),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     "rplidar_link",
                     "base_link")

if __name__ == '__main__':
    rospy.init_node('turtle_tf_broadcaster')
    # turtlename = rospy.get_pacram('~turtle')
    while True:
        handle_turtle_pose()
        