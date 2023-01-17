#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
import numpy as np

first_time = True

start_time = None


def callback(msg):
    global first_time, start_time
    if first_time:
        first_time = False
        start_time = rospy.get_time() # in secs
    

    origin = [msg.info.origin.position.x, msg.info.origin.position.y]
    
    height = msg.info.height
    width = msg.info.width
    map_res = msg.info.resolution
    occupany_map = np.asarray(msg.data)

    # In square meters
    mapped_footage = (occupany_map != -1).sum() * (map_res**2)

    #if (rospy.get_time() - start_time) < 180.0:
    print(f"Map is: {mapped_footage} m^2")
    print(f"Map is: {mapped_footage * 10.76391042} ft^2")
    print("-----")
    # else:
    #     print("3 mins complete: Saving map and shutting down")
    #     rospy.signal_shutdown("3 mins complete: Saving map and shutting down")
    
    
def listener():
    rospy.init_node('map_measure', anonymous=True, disable_signals=True)
    
    rospy.Subscriber("/map_merge_topic", OccupancyGrid, callback)

    rospy.spin()

if __name__ == '__main__':
    while not rospy.is_shutdown():
        listener()
