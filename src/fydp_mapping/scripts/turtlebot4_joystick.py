
import numpy as np
import rospy
import roslib
import subprocess
import time
from geometry_msgs.msg  import Twist
from sensor_msgs.msg import Joy
import sys
import signal

def signal_handler(signal, frame): # ctrl + c -> exit program
        print('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)
''' class '''
class robot():
    def __init__(self):
        rospy.init_node('robot_controller', anonymous=True)
        self.vel_pub_1 = rospy.Publisher('/carter1/cmd_vel', Twist, queue_size=1)
        self.vel_pub_2 = rospy.Publisher('/carter2/cmd_vel', Twist, queue_size=1)
        self.vel_pub_3 = rospy.Publisher('/carter3/cmd_vel', Twist, queue_size=1)

        self.joy_sub = rospy.Subscriber('/joy',Joy,self.callback)
        self.vel_msg = Twist()
        self.rate = rospy.Rate(20)
        self.active_1 = 0
        self.active_2 = 0
        self.active_3 = 0

    def callback(self, data):
        self.buttons = data.buttons
        self.axes= data.axes
        self.active_1=self.buttons[2]
        self.active_2=self.buttons[1]
        self.active_3=self.buttons[0]
        if not self.active_1:
            self.vel_msg.linear.x=0
            self.vel_msg.angular.z=0
            self.moving(1)
        if not self.active_2:
            self.vel_msg.linear.x=0
            self.vel_msg.angular.z=0
            self.moving(2)
        if not self.active_3:
            self.vel_msg.linear.x=0
            self.vel_msg.angular.z=0
            self.moving(3)
        self.linear=self.axes[1]
        self.angular=self.axes[0]
        self.vel_msg.linear.x=turtle.linear*0.3
        self.vel_msg.angular.z=turtle.angular
        
    def moving(self,robot_num):
        if robot_num == 1:     
            self.vel_pub_1.publish(self.vel_msg)
        elif robot_num == 2:       
            self.vel_pub_2.publish(self.vel_msg) 
        elif robot_num == 3:
            self.vel_pub_3.publish(self.vel_msg)

if __name__ == '__main__':
    turtle = robot()

    while 1:
        if turtle.active_1==1:
            turtle.moving(1)

        if turtle.active_2==1:
            turtle.moving(2)

        if turtle.active_3==1:
            turtle.moving(3)
        pass
    turtle.rate.sleep()