#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import re

import sys

import numpy as np


class Robot_Handler:
    def __init__(self, namespace):
        self.first_stage_passed = False
        self.namespace = namespace
        self.command_idx = -1
        self.command_list = ["u", "d"]
        
        self.sub = rospy.Subscriber("/{name}/tb_dock_commands_result".format(name=self.namespace), String, self.tb_dock_callback)

        self.pub_dock = rospy.Publisher("/{name}/tb_dock_commands".format(name=self.namespace), String, queue_size=10)


    def tb_dock_send_command(self, command):
        if command == "u":
            self.command_idx = 0
        elif command == "d":
            self.command_idx = 1

        msg_pub = String()
        msg_pub.data=self.command_list[self.command_idx]
        self.pub_dock.publish(msg_pub)


    def tb_dock_callback(self, msg):
        if not self.first_stage_passed:
            if msg.data == "Goal Accepted":
                self.first_stage_passed = True
            else:
                rospy.loginfo(f"{self.namespace}: {msg.data}") 
                msg_pub = String()
                msg_pub.data=self.command_list[self.command_idx]
                self.pub_dock.publish(msg_pub)

        elif self.first_stage_passed:

            # define a regular expression pattern to match the value of is_docked
            pattern = r"is_docked=(\w+)"

            # use re.search() to find the first occurrence of the pattern in the string
            match = re.search(pattern, msg.data)

            # check if a match was found
            if match:
                # get the captured group from the match object and convert it to a boolean value
                is_docked = bool(match.group(1))
                if is_docked:
                    rospy.loginfo(f"{self.namespace}: Robot is docked!")
                else:
                    rospy.loginfo(f"{self.namespace}: Robot is not docked!")
                
                rospy.signal_shutdown("Dock maneuver complete")

            else:
                rospy.loginfo(f"{self.namespace}: No match found")
            

def main(ns, command):

    rospy.init_node(f'{ns}tb_dock_handles_node', anonymous=True, disable_signals=True)
    node = Robot_Handler(ns)

    from time import sleep
    sleep(0.5)
    # while not rospy.is_shutdown():
    node.tb_dock_send_command(command)
    rospy.spin()

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("usage: tb_dock_commands.py namespace command")
    else:
        main(sys.argv[1], sys.argv[2])  
