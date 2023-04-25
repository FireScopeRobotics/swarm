#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Empty
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
        
        self.pub_dock_ack = rospy.Publisher("/robot_undock_complete", Empty, queue_size=10)

        self.velocity_publisher = rospy.Publisher('/{name}/cmd_vel'.format(name=self.namespace), Twist, queue_size=10)

        print("TEST", self.namespace)

    def move_forward(self, dist):
        speed = 0.1
        vel_msg = Twist()

        vel_msg.linear.x = speed
        
        elapsed_dist = 0.0
        
        hz = 10.0

        r = rospy.Rate(hz)

        while elapsed_dist < dist:
            self.velocity_publisher.publish(vel_msg)
            
            elapsed_dist += speed*1.0/hz
            
            r.sleep()
        
        vel_msg.linear.x = 0.0
        self.velocity_publisher.publish(vel_msg)

    def run_final_manuever(self):
        self.move_forward(0.5)
        msg = Empty()
        self.pub_dock_ack.publish(msg)


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

            is_docked_str = msg.data.split("=")[1].strip(")")
            is_docked = True if is_docked_str == "True" else False

            if is_docked:
                rospy.loginfo(f"{self.namespace}: Robot is docked!")
            else:
                rospy.loginfo(f"{self.namespace}: Robot is not docked!")
            
            if self.command_list[self.command_idx] == "u" and not is_docked:
                self.run_final_manuever()

            rospy.signal_shutdown("Dock maneuver complete")


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
        print("usage: dock_commands.py namespace command")
    else:
        main(sys.argv[1], sys.argv[2])  
