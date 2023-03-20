#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
import tf2_ros

import numpy as np

global_robot_x_list = []
global_robot_y_list = []

def callback(msg):
    
    global pub, tfBuffer, listener, global_robot_x_list, global_robot_y_list

    origin = [msg.info.origin.position.x, msg.info.origin.position.y]
    

    height = msg.info.height
    width = msg.info.width
    map_res = msg.info.resolution
    occupany_map = np.asarray(msg.data)
    
    occupany_map = occupany_map.reshape(height, width)

    rate = rospy.Rate(10.0)
    not_found = True

    while not_found: 
        try:
            trans1 = tfBuffer.lookup_transform('carter1/map', 'carter1/base_link',  rospy.Time())
            trans2 = tfBuffer.lookup_transform('carter2/map', 'carter2/base_link',  rospy.Time())
            trans3 = tfBuffer.lookup_transform('carter3/map', 'carter3/base_link',  rospy.Time())
            not_found = False
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue


    
    robot1_x = trans1.transform.translation.x
    robot1_y = trans1.transform.translation.y

    robot2_x = trans2.transform.translation.x
    robot2_y = trans2.transform.translation.y

    robot3_x = trans3.transform.translation.x
    robot3_y = trans3.transform.translation.y

    global_robot_x_list.append(robot1_x)
    global_robot_x_list.append(robot2_x)
    global_robot_x_list.append(robot3_x)

    global_robot_y_list.append(robot1_y)
    global_robot_y_list.append(robot2_y)
    global_robot_y_list.append(robot3_y)

    # round((robot_x - origin[0])/map_res)

    update_msg = OccupancyGrid()

    # update_msg.x = int((robot_x - 0.15 - origin[0])/map_res)
    # update_msg.y = int((robot_y + 0 - origin[1])/map_res)
    for robot_x, robot_y in zip(global_robot_x_list, global_robot_y_list):

        x = int((robot_x - origin[0])/map_res)
        y = int((robot_y - origin[1])/map_res)
        occupany_map[y - 7: y + 7, x - 7 : x + 7] = np.zeros((14, 14), dtype=np.int8)

    # x = int((robot2_x - origin[0])/map_res)
    # y = int((robot2_y - origin[1])/map_res)
    # occupany_map[y - 7: y + 7, x - 7 : x + 7] = np.zeros((14, 14), dtype=np.int8)

    # x = int((robot3_x - origin[0])/map_res)
    # y = int((robot3_y - origin[1])/map_res)
    # occupany_map[y - 7: y + 7, x - 7 : x + 7] = np.zeros((14, 14), dtype=np.int8)
    # update_msg.width = 6
    # update_msg.height = 6
    # update_msg.data = 100* np.ones(25, dtype=np.int8)

    update_msg = msg

    update_msg.data = occupany_map.flatten()
    pub.publish(update_msg)

    # mapped_footage = (occupany_map != -1).sum() * (map_res**2)

    # print(f"Map is: {mapped_footage} m^2")
    # print(f"Map is: {mapped_footage * 10.76391042} ft^2")
    # print("-----")

pub = None
tfBuffer = None
listener = None

def listener():
    rospy.init_node('map_measure', anonymous=True, disable_signals=True)
    
    global pub, tfBuffer, listener

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    
    pub = rospy.Publisher('/map', OccupancyGrid, queue_size=10)

    rospy.Subscriber("/map_merge_topic", OccupancyGrid, callback)

    rospy.spin()

if __name__ == '__main__':
    while not rospy.is_shutdown():
        listener()
