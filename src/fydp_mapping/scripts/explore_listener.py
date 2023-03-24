#!/usr/bin/env python
import rospy
from std_msgs.msg import Empty
import numpy as np
import time
import subprocess

ack_count = 0

def run_process ():
    time.sleep(2)
    print ("start")
    subprocess.call("/home/nick/swarm/src/fydp_mapping/scripts/run_explore.sh")
    print ("end")

def callback(msg):
    global ack_count
    
    ack_count += 1

    if ack_count == 3:
        run_process()
    
def listener():
    rospy.init_node('explore_initializer', anonymous=True, disable_signals=True)
    
    rospy.Subscriber("/robot_undock_complete", Empty, callback)

    rospy.spin()

if __name__ == '__main__':
    while not rospy.is_shutdown():
        listener()
