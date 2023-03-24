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


first_translation_dict = {}
first_translation_dict["carter1"] = [-1.5, -7.8]
first_translation_dict["carter2"] = [0.0, 0.0]
first_translation_dict["carter3"] = [3.8, 0.1]


class TF_inject:
    def __init__(self, namespace):

        self.namespace = namespace


        rospy.Subscriber("/tf".format(name=self.namespace), TFMessage, self.tf_callback)
        
        self.pub_tf = rospy.Publisher('/tf', TFMessage, queue_size=10)

    def tf_callback(self, msg):
        msg_pub = TFMessage()
        global first_translation_dict
        

        # global qr, first_translation, tf_found

        for tf_obj in msg.transforms:
            if tf_obj.header.frame_id == f"{self.namespace}/map" and (tf_obj.child_frame_id == f"{self.namespace}/odom_slam"):
                tf_val = TransformStamped()
                qr = tf.transformations.quaternion_from_euler(0,0, np.pi/2)
                # qr[0] = tf_obj.transform.rotation.x
                # qr[1] = tf_obj.transform.rotation.y
                # qr[2] = tf_obj.transform.rotation.z
                # qr[3] = tf_obj.transform.rotation.w
                
                q1 = tf.transformations.quaternion_from_euler(0, 0, 0)
                q1[0] = tf_obj.transform.rotation.x
                q1[1] = tf_obj.transform.rotation.y
                q1[2] = tf_obj.transform.rotation.z
                q1[3] = tf_obj.transform.rotation.w

                q_corrected = tf.transformations.quaternion_multiply(qr, q1)

                tf_val.transform.rotation.x = q_corrected[0]
                tf_val.transform.rotation.y = q_corrected[1]
                tf_val.transform.rotation.z = q_corrected[2]
                tf_val.transform.rotation.w = q_corrected[3] 
                
                name = f"{self.namespace}"

                trans = first_translation_dict[name]
                tf_val.transform.translation.x = tf_obj.transform.translation.x + trans[0]
                tf_val.transform.translation.y = tf_obj.transform.translation.y + trans[1]
                
                # tf_obj.transform.translation.z -= first_translation[2] 
                
                tf_val.header.frame_id = self.namespace + "/map"
                tf_val.child_frame_id = self.namespace + "/odom"

                tf_val.header.seq = tf_obj.header.seq
                tf_val.header.stamp = tf_obj.header.stamp

                msg_pub.transforms.append(tf_val)
                self.pub_tf.publish(msg_pub)
                return


def main(ns):

    rospy.init_node('{name}_tf_inject'.format(name=ns), anonymous=True, disable_signals=True)
    node = TF_inject(ns)

    rospy.spin()


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("usage: TB_topic_inject_slam.py namespace")
    else:
        main(sys.argv[1])  