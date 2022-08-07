import numpy as np
from layered_planner.tools import *
from layered_planner.rrt import *
from layered_planner.potential_fields import *


   
sp = [0, 0] #the point where the robot is
sp_global = [0,0]
route = np.array([sp])
f = 0
leader = False
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
    if num_robots<=1: return []
    if num_robots==2: return [des4]
    if num_robots==3: return [des2, des3]
    if num_robots==4: return [des2, des3, des4]
    if num_robots==5: return [des2, des3, des4, des5]
    if num_robots==6: return [des2, des3, des4, des5, des6]
    if num_robots==7: return [des2, des3, des4, des5, des6, des7]
    if num_robots==8: return [des2, des3, des4, des5, des6, des7, des8]
    if num_robots==9: return [des2, des3, des4, des5, des6, des7, des8, des9]
    
    return [des2, des3, des4]

# updates robot setpoint route, velocity from potential grid and gradient planner
def local_planner(obstacles, influence_radius, drone_vel):
        obstacles_grid = grid_map(obstacles) # We can pass in the occupancy grid here directly instead of using the 4 point system
        f = combined_potential(obstacles_grid, sp_global, influence_radius)
        sp, vel = gradient_planner_next(sp, f, drone_vel)
        vel_array.append(norm(vel))
        route = np.vstack( [route, sp] )


def nextStepFromSingleWPT(P, drone_vel, ViconRate):
    V = drone_vel * 1.3 # [m/s], the setpoint should travel a bit faster than the robot
    freq = ViconRate # vicon rate is teh sample rate
    dt = 1./freq
    dx = V * dt # m/s * s = m
    vec = P[1]-P[0]
    dist = norm(vec)
    unit_vector = vec/dist
    return  unit_vector*dx + P[0]