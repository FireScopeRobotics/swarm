#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

import sys

import numpy as np

class TF_inject:
    def __init__(self, namespace):

        self.namespace = namespace

        rospy.Subscriber("/{name}/tf_docker".format(name=self.namespace), TFMessage, self.tf_callback)
        rospy.Subscriber("/{name}/odom_docker".format(name=self.namespace), Odometry, self.odom_callback)
        rospy.Subscriber("/{name}/scan_docker".format(name=self.namespace), LaserScan, self.scan_callback)

        self.pub_tf = rospy.Publisher('/tf', TFMessage, queue_size=10)
        self.pub_odom = rospy.Publisher('/{name}/odom'.format(name=self.namespace), Odometry, queue_size=10)
        self.pub_scan = rospy.Publisher('/{name}/scan'.format(name=self.namespace), LaserScan, queue_size=10)


    def tf_callback(self, msg):
        msg_pub = msg
        
        tf_val = TransformStamped()

        for tf in msg_pub.transforms:
            tf.header.frame_id = self.namespace + "/" + tf.header.frame_id
            tf.child_frame_id = self.namespace + "/" + tf.child_frame_id
            tf_val.header.seq = tf.header.seq
            tf_val.header.stamp = tf.header.stamp

        # TF between base_link and rplidar_link
        #(x=-0.04, y=0.0, z=0.192915), rotation=geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=0.7071067811865475, w=0.7071067811865476)))
        
        tf_val.header.frame_id = self.namespace + "/" + "base_link"
        tf_val.child_frame_id = self.namespace + "/" + "rplidar_link"

        tf_val.transform.translation.x = -0.04
        tf_val.transform.translation.y = 0.0
        tf_val.transform.translation.z = 0.192915

        tf_val.transform.rotation.x = 0.0
        tf_val.transform.rotation.y = 0.0
        tf_val.transform.rotation.z = 0.7071067811865475 
        tf_val.transform.rotation.w = 0.7071067811865476
        
        msg_pub.transforms.append(tf_val)
        
        self.pub_tf.publish(msg)


    def odom_callback(self, msg):
        msg_pub = msg

        msg_pub.header.frame_id = self.namespace + "/" + msg_pub.header.frame_id
        msg_pub.child_frame_id = self.namespace + "/" + msg_pub.child_frame_id

        self.pub_odom.publish(msg)

    def scan_callback(self, msg):
        msg_pub = msg

        msg_pub.header.frame_id = self.namespace + "/" + msg_pub.header.frame_id

        self.pub_scan.publish(msg)

def main(ns):

    rospy.init_node('{name}_tf_inject'.format(name=ns), anonymous=True, disable_signals=True)
    node = TF_inject(ns)

    rospy.spin()


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("usage: TB_topic_inject.py namespace")
    else:
        main(sys.argv[1])  