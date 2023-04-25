#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
import tf

import sys

import numpy as np

first = True
qr = tf.transformations.quaternion_from_euler(0, 0, 0)
first_translation = np.array([0.0, 0.0, 0.0])
tf_found = False

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

        global qr, first_translation, tf_found

        for tf_obj in msg_pub.transforms:
            if tf_obj.header.frame_id == "odom" and (tf_obj.child_frame_id == "base_link" or tf_obj.child_frame_id == "base_footprint"):
                q2 = tf.transformations.quaternion_from_euler(0, 0, 0)
                q2[0] = tf_obj.transform.rotation.x
                q2[1] = tf_obj.transform.rotation.y
                q2[2] = tf_obj.transform.rotation.z
                q2[3] = tf_obj.transform.rotation.w
                
                q_corrected = tf.transformations.quaternion_multiply(qr, q2)

                tf_obj.transform.rotation.x = q_corrected[0]
                tf_obj.transform.rotation.y = q_corrected[1]
                tf_obj.transform.rotation.z = q_corrected[2]
                tf_obj.transform.rotation.w = q_corrected[3] 
                        
                tf_obj.transform.translation.x -= first_translation[0]
                tf_obj.transform.translation.y -= first_translation[1]
                # tf_obj.transform.translation.z -= first_translation[2] 
            
            tf_obj.header.frame_id = self.namespace + "/" + tf_obj.header.frame_id
            tf_obj.child_frame_id = self.namespace + "/" + tf_obj.child_frame_id
            tf_val.header.seq = tf_obj.header.seq
            tf_val.header.stamp = tf_obj.header.stamp

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
        

        self.pub_tf.publish(msg_pub)


    def odom_callback(self, msg):
        msg_pub = msg

        msg_pub.header.frame_id = self.namespace + "/" + msg_pub.header.frame_id
        msg_pub.child_frame_id = self.namespace + "/" + msg_pub.child_frame_id

        global first, qr, first_translation, tf_found

        if first:
            # Going from q2 to q1 and finidng relative qr
            q1 = tf.transformations.quaternion_from_euler(0, 0, 0)
            q2_inv = tf.transformations.quaternion_from_euler(0, 0, 0)
            q2_inv[0] = msg_pub.pose.pose.orientation.x
            q2_inv[1] = msg_pub.pose.pose.orientation.y
            q2_inv[2] = msg_pub.pose.pose.orientation.z
            q2_inv[3] = -msg_pub.pose.pose.orientation.w # Negate for inverse

            qr = tf.transformations.quaternion_multiply(q1, q2_inv)
            
            first_translation = [msg_pub.pose.pose.position.x, msg_pub.pose.pose.position.y, msg_pub.pose.pose.position.z]
            
            first = False
            tf_found = True

        q2 = tf.transformations.quaternion_from_euler(0, 0, 0)
        q2[0] = msg_pub.pose.pose.orientation.x
        q2[1] = msg_pub.pose.pose.orientation.y
        q2[2] = msg_pub.pose.pose.orientation.z
        q2[3] = msg_pub.pose.pose.orientation.w
        
        q_corrected = tf.transformations.quaternion_multiply(qr, q2)

        msg_pub.pose.pose.orientation.x = q_corrected[0]
        msg_pub.pose.pose.orientation.y = q_corrected[1]
        msg_pub.pose.pose.orientation.z = q_corrected[2]
        msg_pub.pose.pose.orientation.w = q_corrected[3] 
                
        msg_pub.pose.pose.position.x -= first_translation[0]
        msg_pub.pose.pose.position.y -= first_translation[1]
        # msg_pub.pose.pose.position.z -= first_translation[2] 

        self.pub_odom.publish(msg_pub)

    def scan_callback(self, msg):
        msg_pub = msg

        msg_pub.header.frame_id = self.namespace + "/" + msg_pub.header.frame_id

        self.pub_scan.publish(msg_pub)

def main(ns):

    rospy.init_node('{name}_tf_inject'.format(name=ns), anonymous=True, disable_signals=True)
    node = TF_inject(ns)

    rospy.spin()


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("usage: topic_inject.py namespace")
    else:
        main(sys.argv[1])  