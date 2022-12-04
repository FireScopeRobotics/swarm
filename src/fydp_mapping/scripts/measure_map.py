#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
import numpy as np

def callback(msg):

   origin = [msg.info.origin.position.x, msg.info.origin.position.y]
    
   height = msg.info.height
   width = msg.info.width
   map_res = msg.info.resolution
   occupany_map = np.asarray(msg.data)
   
   # In square meters
   mapped_footage = (occupany_map != -1).sum() * (map_res**2)
   
   print(f"Map is: {mapped_footage} m^2")
   print(f"Map is: {mapped_footage * 10.76391042} ft^2")
   print("-----")
    
def listener():
    rospy.init_node('map_measure', anonymous=True)
    
    rospy.Subscriber("map", OccupancyGrid, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
