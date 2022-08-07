from __future__ import print_function
import rospy
import numpy as np
from std_msgs.msg import Float64
from convoy_services.srv import convoy_functions
from tools import *
from rrt import *
from potential_fields import *
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

sp = [0, 0] #the point where the robot is
sp_global = [0,0]
route = np.array([sp])
vel_array = []

def formation(num_robots, leader_des, v, l):
    """
    geometry of the swarm: following robots desired locations
    relatively to the leader
    """
    u = np.array([-v[1], v[0]])
    """ followers positions """
    des2 = leader_des - v*l*sqrt(3)/2 + u*l/2
    des3 = leader_des - v*l*sqrt(3)/2 - u*l/2
    des4 = leader_des - v*l*sqrt(3)
    des5 = leader_des - v*l*sqrt(3)   + u*l
    des6 = leader_des - v*l*sqrt(3)   - u*l
    des7 = leader_des - v*l*sqrt(3)*3/2 - u*l/2
    des8 = leader_des - v*l*sqrt(3)*3/2 + u*l/2
    des9 = leader_des - v*l*sqrt(3)*2
    dest_list = []
    if num_robots<=1: dest_list = []
    if num_robots==2: dest_list = [des4]
    if num_robots==3: dest_list = [des2, des3]
    if num_robots==4: dest_list = [des2, des3, des4]
    if num_robots==5: dest_list = [des2, des3, des4, des5]
    if num_robots==6: dest_list = [des2, des3, des4, des5, des6]
    if num_robots==7: dest_list = [des2, des3, des4, des5, des6, des7]
    if num_robots==8: dest_list = [des2, des3, des4, des5, des6, des7, des8]
    if num_robots==9: dest_list = [des2, des3, des4, des5, des6, des7, des8, des9]

    dest_array = np.stack( dest_list, axis=0 )
    retArray = Float32MultiArray()
    retArray.layout.dim = [MultiArrayDimension(f'dim{i}', dest_array.shape[i], int(dest_array.strides[i] / dest_array.dtype.itemsize)) 
                                                for i in range(dest_array.ndim)];
    return {"destinations": retArray}

# updates robot setpoint route, velocity from potential grid and gradient planner
def local_planner(obstacles, influence_radius, drone_vel):
    dims = tuple(map(lambda x: x.size, obstacles.layout.dim))
    obstacles_list = np.array(obstacles.data, dtype=Float32MultiArray).reshape(dims).astype(np.float32)

    obstacles_grid = grid_map(obstacles_list) # We can pass in the occupancy grid here directly instead of using the 4 point system
    f = combined_potential(obstacles_grid, sp_global, influence_radius)
    sp, vel = gradient_planner_next(sp, f, drone_vel)
    vel_array.append(norm(vel))
    route = np.vstack( [route, sp] )
    return {"sp": tuple(sp)}

def nextStepFromSingleWPT(P, drone_vel, ViconRate):
    V = drone_vel * 1.3 # [m/s], the setpoint should travel a bit faster than the robot
    freq = ViconRate # vicon rate is teh sample rate
    dt = 1./freq
    dx = V * dt # m/s * s = m
    vec = P[1]-P[0]
    dist = norm(vec)
    unit_vector = vec/dist
    return  {"nxt_pt": tuple(unit_vector*dx + P[0])}

def callback(req):
    if req.function_type == 0:
        return formation(req.num_robots, req.leader_des, req.v, req.l)
    elif req.function_type == 1:
        return nextStepFromSingleWPT(req.P, req.drone_vel, req.ViconRate)
    elif req.function_type == 2:
        return local_planner(req.obstacles, req.influence_radius, req.drone_vel)

def setup_service():
    rospy.init_node("convoy_service")
    service = rospy.Service("convoy_functions", convoy_functions, callback)
    print("Convoy service up")
    rospy.spin()

if __name__ == '__main__':
    setup_service()