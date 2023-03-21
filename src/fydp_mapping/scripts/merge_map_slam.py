#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid
import numpy as np
from scipy import ndimage


class Map:
    def __init__(self):
        self.data = None
        self.origin = None
        self.width = None
        self.height = None
        self.map_res = None
        self.rotated = False
        self.frame = None
    def set_data(self, msg):
        self.data = np.array(msg.data)
        self.origin = [msg.info.origin.position.x, msg.info.origin.position.y]
        self.width = msg.info.width
        self.height = msg.info.height
        self.data = self.data.reshape(self.height, self.width)
        self.rotated = False
        self.frame = msg.header.frame_id

        self.map_res = msg.info.resolution  
class Map_Merge:
    def __init__(self):
        pix_width = 100
        self.origin = [-pix_width/2, -pix_width/2]
        self.width = int(pix_width/0.05)
        self.height = int(pix_width/0.05)
        self.data = 0 * np.ones((self.height, self.width), dtype=np.int8)
        self.map_res = 0.05

map_merge = Map_Merge()

map1 = Map()
map2 = Map()
map3 = Map()
# def combine_maps():
#     map1

#     result = np.maximum(map1, np.maximum(map2, map3))

def map1_callback(msg):
    global map1

    map1.set_data(msg)
    # origin = [msg.info.origin.position.x, msg.info.origin.position.y]
    # width = msg.info.width
    # height = msg.info.height
    # map_res = msg.info.resolution

    # map1 = np.array(msg.data)


def map2_callback(msg):
    global map2

    # map2 = np.array(msg.data)
    map2.set_data(msg)

def map3_callback(msg):
    global map3
    
    # map3 = np.array(msg.data)
    # msg_save = msg

    map3.set_data(msg)
    print (map3.data.shape)
    
def post_map_callback(event):
    global map_merge, map1, map2, map3, pub
    try:
        def update_map(map_slice, map):
            try:
                map_merge_topic = rospy.get_param('~map_merge_topic')
                if map_merge_topic == "/map_merge_topic_thin":
                    mask = (map.data != -1)
                else:    
                    mask = (map.data != 0)
                
                # print(map.frame, map_slice.shape, map.data.shape)
                map_slice[mask] = map.data[mask]
                return map_slice
            except:
                pass
        
        if (map1.width is not None):
            map1_x = int((map1.origin[0]  - 0 - map_merge.origin[0])/map_merge.map_res)
            map1_y = int((map1.origin[1] - 0 - map_merge.origin[1])/map_merge.map_res)
            map1_slice = map_merge.data[map1_y: (map1_y + map1.height), map1_x: (map1_x + map1.width)]
            map_merge.data[map1_y: map1_y + map1.height, map1_x: map1_x + map1.width] = update_map(
                map1_slice, map1
            )

        
        if (map2.width is not None):
            map2_x = int((map2.origin[0] - map_merge.origin[0])/map_merge.map_res)
            map2_y = int((map2.origin[1] - map_merge.origin[1])/map_merge.map_res)
            map2_slice = map_merge.data[map2_y: map2_y + map2.height, map2_x: map2_x + map2.width]
            map_merge.data[map2_y: map2_y + map2.height, map2_x: map2_x + map2.width] = update_map(
                map2_slice, map2
            )

        if (map3.width is not None):
            map3_x = int((map3.origin[0] - 0 - map_merge.origin[0])/map_merge.map_res)
            map3_y = int((map3.origin[1] - map_merge.origin[1])/map_merge.map_res)
            map3_slice = map_merge.data[map3_y: map3_y + map3.height, map3_x: map3_x + map3.width]
            # print(map_merge.data.shape, map3_slice.shape, map3.data.shape, map3.height, map3.width, map3_x, map3_y)
            map_merge.data[map3_y: map3_y + map3.height, map3_x: map3_x + map3.width] = update_map(
                map3_slice, map3
            )

            
            

        # def rotate_image(map, angle):

        #     image = map.data
        #     pivot = [int(-map.origin[0]/map.map_res), int(-map.origin[1]/map.map_res)]
        #     # Define the rotation matrix
        #     theta = np.radians(angle)
        #     c, s = np.cos(theta), np.sin(theta)
        #     rotation_matrix = np.array([[c, -s], [s, c]])

        #     # Translate the image so that the pivot point is at the origin
        #     shifted_image = shift(image, (-pivot[0], -pivot[1]))

        #     # Apply the rotation matrix to the translated image
        #     rotated_image = np.zeros_like(shifted_image)
        #     for i in range(shifted_image.shape[0]):
        #         for j in range(shifted_image.shape[1]):
        #             rotated_coords = np.dot(rotation_matrix, np.array([i, j]))
        #             rotated_image[int(rotated_coords[0]), int(rotated_coords[1])] = shifted_image[i, j]

        #     # Translate the image back to its original position
        #     final_image = shift(rotated_image, (pivot[0], pivot[1]))

        #     return final_image

        # def rotate_image (map, rotate):
        #     img = map.data
        #     pivot = [int(-map.origin[0]/map.map_res), int(-map.origin[1]/map.map_res)]
        #     padX = [img.shape[1] - pivot[0], pivot[0]]
        #     padY = [img.shape[0] - pivot[1], pivot[1]]
        #     imgP = np.pad(img+100, [padY, padX], 'constant')
        #     imgR = ndimage.rotate(imgP, rotate, reshape=False)
        #     imgC = imgR[padY[0] : -padY[1], padX[0] : -padX[1]]
        
        #     imgC -= 100

        #     imgC[imgC < 0] = -1
        #     map.rotated = True

        #     return imgC
        
        # if map1.rotated == False:
        #     map1.data = rotate_image(map1, -90)
        # if map2.rotated == False:
        #     map2.data = rotate_image(map2, -90)
        # if map3.rotated == False:
        #     map3.data = rotate_image(map3, -90)
            
        # map1_x = int((map1.origin[0]  - 0 - map_merge.origin[0])/map_merge.map_res)
        # map1_y = int((map1.origin[1] - 0 - map_merge.origin[1])/map_merge.map_res)
        
        # map2_x = int((map2.origin[0] - map_merge.origin[0])/map_merge.map_res)
        # map2_y = int((map2.origin[1] - map_merge.origin[1])/map_merge.map_res)

        # map3_x = int((map3.origin[0] - 0 - map_merge.origin[0])/map_merge.map_res)
        # map3_y = int((map3.origin[1] - map_merge.origin[1])/map_merge.map_res)

        # map1_slice = map_merge.data[map1_y: map1_y + map1.height, map1_x: map1_x + map1.width]
        # map2_slice = map_merge.data[map2_y: map2_y + map2.height, map2_x: map2_x + map2.width]
        # map3_slice = map_merge.data[map3_y: map3_y + map3.height, map3_x: map3_x + map3.width]
        
        
        # map_merge.data[map1_y: map1_y + map1.height, map1_x: map1_x + map1.width] = update_map(
        #     map1_slice, map1
        # )
        
        # map_merge.data[map2_y: map2_y + map2.height, map2_x: map2_x + map2.width] = update_map(
        #     map2_slice, map2
        # )
        
        # map_merge.data[map3_y: map3_y + map3.height, map3_x: map3_x + map3.width] = update_map(
        #     map3_slice, map3
        # )
        
        # map_merge.data[map1_y: map1_y + map1.height, map1_x: map1_x + map1.width] = np.where(
        #     map_merge.data[map1_y: map1_y + map1.height, map1_x: map1_x + map1.width], map1.data == -1, 
        #     map1.data, 
        #     map_merge.data[map1_y: map1_y + map1.height, map1_x: map1_x + map1.width]
        # )
        # np.maximum(
        #     map_merge.data[map1_y: map1_y + map1.height, map1_x: map1_x + map1.width], map1.data
        #     )
        # map_merge.data[map2_y: map2_y + map2.height, map2_x: map2_x + map2.width] = np.maximum(
        #     map_merge.data[map2_y: map2_y + map2.height, map2_x: map2_x + map2.width], map2.data
        #     )
        
        # map_merge.data[map3_y: map3_y + map3.height, map3_x: map3_x + map3.width] = np.maximum(
        #     map_merge.data[map3_y: map3_y + map3.height, map3_x: map3_x + map3.width], map3.data
        #     )
        

        map_msg = OccupancyGrid()
        
        map_msg.header.frame_id = "map"
        map_msg.info.origin.position.x = map_merge.origin[0] 
        map_msg.info.origin.position.y = map_merge.origin[1]
        
        map_msg.info.resolution = map_merge.map_res
        
        map_msg.info.width = map_merge.width
        map_msg.info.height = map_merge.height

        map_msg.data = map_merge.data.flatten()
        pub.publish(map_msg)
        
        print("Published map")

        # map_msg.data = np.maximum(map1, np.maximum(map2, map3))
        #pub.publish(map_msg)
    except Exception as e:
        print("lol")
        print(e)
        pass

def listener():

    global pub, map_merge
    rospy.init_node('merge_map', anonymous=True)
    map_merge_topic = rospy.get_param('~map_merge_topic')
    costmap_sub_topic = rospy.get_param('~costmap_sub_topic')
    

    pub = rospy.Publisher(map_merge_topic, OccupancyGrid, queue_size=10)

    rospy.Subscriber("/carter1/"+ costmap_sub_topic, OccupancyGrid, map1_callback)
    rospy.Subscriber("/carter2/"+ costmap_sub_topic, OccupancyGrid, map2_callback)
    rospy.Subscriber("/carter3/"+ costmap_sub_topic, OccupancyGrid, map3_callback)

    map_merge_topic = rospy.get_param('~map_merge_topic')
    if map_merge_topic == "/map_merge_topic_thin":
        map_merge.data -= 1
        
    # rospy.Subscriber("/carter1/map", OccupancyGrid, map1_callback)
    # rospy.Subscriber("/carter2/map", OccupancyGrid, map2_callback)
    # rospy.Subscriber("/carter3/map", OccupancyGrid, map3_callback)

    rospy.Timer(rospy.Duration(0.5), post_map_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
