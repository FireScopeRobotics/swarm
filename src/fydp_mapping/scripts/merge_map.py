#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid
import numpy as np

map1 = None
map2 = None
map3 = None
# width = None
# height = None
# origin = None
# map_res = None
pub = None

# def combine_maps():
#     map1

#     result = np.maximum(map1, np.maximum(map2, map3))

def map1_callback(msg):
    global map1, width, height, origin, map_res

    # origin = [msg.info.origin.position.x, msg.info.origin.position.y]
    # width = msg.info.width
    # height = msg.info.height
    # map_res = msg.info.resolution

    map1 = np.array(msg.data)


def map2_callback(msg):
    global map2

    map2 = np.array(msg.data)


def map3_callback(msg):
    global map1, map2, map3, pub
    
    map3 = np.array(msg.data)

    if (map1 is None) or (map2 is None) or (map3 is None):
        return
    
    map_msg = msg
    map_msg.data = np.maximum(map1, np.maximum(map2, map3))
    pub.publish(map_msg)
    

def listener():

    global pub
    rospy.init_node('merge_map', anonymous=True)

    pub = rospy.Publisher('map_merge_topic_lol', OccupancyGrid, queue_size=10)

    rospy.Subscriber("/carter1/map", OccupancyGrid, map1_callback)
    rospy.Subscriber("/carter2/map", OccupancyGrid, map2_callback)
    rospy.Subscriber("/carter3/map", OccupancyGrid, map3_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()